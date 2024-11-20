#include "py/mphal.h"
#include "py/runtime.h"
#include "py/stream.h"
#include "modmachine.h"
#include "modesp32.h"

#include "esp_task.h"
#include "driver/rmt_rx.h"

/*
import machine, esp32
p = machine.Pin(14)

r = esp32.RMT2(pin=p, buf=64, min_ns=3100, max_ns=5000000, resolution_hz=1000000)
r.read_pulses()
r.get_data() # returns None if no data
# r can be poll.poll()'ed 
*/

// dummy function to satisfy external reference (bitstream won't work)
void machine_bitstream_high_low(mp_hal_pin_obj_t pin, uint32_t *timing_ns, const uint8_t *buf, size_t len) {
}

// Forward declaration
extern const mp_obj_type_t esp32_rmt2_type;

// Python object impl structure
typedef struct _esp32_rmt2_obj_t {
    mp_obj_base_t base;
    gpio_num_t pin;
    bool continue_rx;
    rmt_channel_handle_t rx_channel;
    rmt_symbol_word_t *symbols;
    size_t symbols_size;
    size_t num_symbols;
    // rmt_rx_done_event_data_t rx_data;
    rmt_receive_config_t rx_config;
    rmt_rx_channel_config_t rx_ch_conf;

    // double buffering of received data
    bool recv_available;
    size_t recv_num_symbols;
    rmt_symbol_word_t *recv_symbols;
} esp32_rmt2_obj_t;

// Interrupt handler
static bool IRAM_ATTR rmt_recv_done(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *udata){
    esp32_rmt2_obj_t *self = udata;

    if (!self->recv_available) {
        self->recv_num_symbols = edata->num_symbols;
        memcpy(self->recv_symbols, edata->received_symbols, edata->num_symbols * sizeof(rmt_symbol_word_t));
        self->recv_available = true;
    }

    // mp_hal_wake_main_task_from_isr();

    if (self->continue_rx) {
        rmt_receive(self->rx_channel, self->symbols, self->symbols_size, &self->rx_config);
    }

    return false;
}

static mp_obj_t esp32_rmt2_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_pin,           MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_buf,                             MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 64} },
        { MP_QSTR_min_ns,        MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_max_ns,        MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_resolution_hz, MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    gpio_num_t pin_id = machine_pin_get_id(args[0].u_obj);
    int num_symbols = args[1].u_int;
    if ((num_symbols < 64) || ((num_symbols % 2) == 1)) {
        mp_raise_ValueError(MP_ERROR_TEXT("buf must be at least 64 and even"));
    }

    // RMT new driver does not have a way to specify a group clock divisor, see
    // https://github.com/espressif/esp-idf/issues/14760#issuecomment-2430770601 and
    // https://github.com/espressif/esp-idf/issues/11262
    // since signal_range_min_ns must fit in an 8-bit register, the value is limited
    // to 3190ns (80MHz clock * 3190 / 1000000000 [conversion to ns] == 255).

    int min_ns = args[2].u_int;
    if (min_ns <= 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("min_ns must be positive"));
    }
    int max_ns = args[3].u_int;
    if (max_ns <= min_ns) {
        mp_raise_ValueError(MP_ERROR_TEXT("max_ns must be bigger than min_ns"));
    }
    int resolution_hz = args[4].u_int;
    if (resolution_hz < 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("resolution_hz must be positive"));
    }

    esp32_rmt2_obj_t *self = mp_obj_malloc_with_finaliser(esp32_rmt2_obj_t, &esp32_rmt2_type);

    self->rx_channel = NULL;
    self->num_symbols = num_symbols;
    self->symbols_size = num_symbols * sizeof(rmt_symbol_word_t);
    self->symbols = m_realloc(NULL, self->symbols_size);
    self->recv_symbols = m_realloc(NULL, self->symbols_size);
    self->recv_available = false;

    self->rx_config.signal_range_min_ns = min_ns;
    self->rx_config.signal_range_max_ns = max_ns;
    self->rx_ch_conf.gpio_num = self->pin = pin_id;
    self->rx_ch_conf.clk_src = RMT_CLK_SRC_DEFAULT;
    self->rx_ch_conf.resolution_hz = resolution_hz;
    self->rx_ch_conf.mem_block_symbols = num_symbols;

    check_esp_err(rmt_new_rx_channel(&self->rx_ch_conf, &self->rx_channel));
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_recv_done,
    };
    // TODO call from task to move to different core?
    check_esp_err(rmt_rx_register_event_callbacks(self->rx_channel, &cbs, self));
    check_esp_err(rmt_enable(self->rx_channel));

    self->recv_num_symbols = 0;

    return MP_OBJ_FROM_PTR(self);
}

static void esp32_rmt2_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "RMT2 pin=%u buf=%u(x2) min_ns=%u max_ns=%u,resolution_hz=%u, last_recv=%u",
                    self->pin, self->num_symbols,
                    self->rx_config.signal_range_min_ns, self->rx_config.signal_range_max_ns,
                    self->rx_ch_conf.resolution_hz);
}


static mp_obj_t esp32_rmt2_deinit(mp_obj_t self_in) {
    esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (self->pin != -1) { // Check if channel has already been deinitialised.
        rmt_disable(self->rx_channel);
        rmt_del_channel(self->rx_channel);
        self->pin = -1;
    }
    m_free(self->symbols);
    self->symbols = 0;
    m_free(self->recv_symbols);
    self->recv_symbols = 0;
    self->recv_available = false;

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_rmt2_deinit_obj, esp32_rmt2_deinit);


static mp_obj_t esp32_rmt2_stop_read_pulses(mp_obj_t self_in) {
    esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);

    bool ret = self->continue_rx;
    self->continue_rx = false;
    self->recv_available = false;
    return ret ? mp_const_true : mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_rmt2_stop_read_pulses_obj, esp32_rmt2_stop_read_pulses);


static mp_obj_t esp32_rmt2_read_pulses(mp_obj_t self_in) {
    esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);

    self->continue_rx = true;
    self->recv_available = false;
    self->recv_num_symbols = 0;
    check_esp_err(rmt_receive(self->rx_channel, self->symbols, self->symbols_size, &self->rx_config));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_rmt2_read_pulses_obj, esp32_rmt2_read_pulses);


static mp_obj_t esp32_rmt2_get_data(mp_obj_t self_in) {
    esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (! self->recv_available) {
        return mp_const_none;
    }

    // convert RMT samples to list
    size_t len = self->recv_num_symbols;
    const rmt_symbol_word_t *data = self->recv_symbols;

    int odd = (data[len - 1].duration1 == 0) ? 1 : 0;
    mp_obj_t list = mp_obj_new_list(len * 2 - odd, NULL);
    mp_obj_list_t *list_in = MP_OBJ_TO_PTR(list);

    for (uint8_t i = 0; i < len; i++) {
        int n0 = (uint16_t) data[i].duration0;
        int n1 = (uint16_t) data[i].duration1;
        if (!data[i].level0) {
            n0 = -n0;
        }
        if (!data[i].level1) {
            n1 = -n1;
        }
        list_in->items[i * 2 + 0] = mp_obj_new_int(n0);
        if ((i < (len-1)) || (!odd)) {
            list_in->items[i * 2 + 1] = mp_obj_new_int(n1);
        }
    }

    self->recv_available = false;

    return list;
}

static MP_DEFINE_CONST_FUN_OBJ_1(esp32_rmt2_get_data_obj, esp32_rmt2_get_data);


static const mp_rom_map_elem_t esp32_rmt2_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&esp32_rmt2_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&esp32_rmt2_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_pulses), MP_ROM_PTR(&esp32_rmt2_read_pulses_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_data), MP_ROM_PTR(&esp32_rmt2_get_data_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_read_pulses), MP_ROM_PTR(&esp32_rmt2_stop_read_pulses_obj) },
};
static MP_DEFINE_CONST_DICT(esp32_rmt2_locals_dict, esp32_rmt2_locals_dict_table);


static mp_uint_t esp32_rmt2_stream_ioctl(
    mp_obj_t self_in, mp_uint_t request, uintptr_t arg, int *errcode) {
    if (request != MP_STREAM_POLL) {
        *errcode = MP_EINVAL;
        return MP_STREAM_ERROR;
    }
    esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return arg ^ (
               // If no data in the buffer, unset the Read ready flag
               (self->recv_available ? 0 : MP_STREAM_POLL_RD));
}

static const mp_stream_p_t esp32_rmt2_stream_p = {
    .ioctl = esp32_rmt2_stream_ioctl,
};

MP_DEFINE_CONST_OBJ_TYPE(
    esp32_rmt2_type,
    MP_QSTR_RMT2,
    MP_TYPE_FLAG_NONE,
    make_new, esp32_rmt2_make_new,
    print, esp32_rmt2_print,
    locals_dict, &esp32_rmt2_locals_dict,
    protocol, &esp32_rmt2_stream_p
    );
