#include "py/mphal.h"
#include "py/runtime.h"
#include "modmachine.h"
#include "modesp32.h"

#include "esp_task.h"
#include "driver/rmt_rx.h"

/*
import machine, esp32
p = machine.Pin(14)

total = 0
totalb = 0

def cb(l):
    global total, totalb
    total += 1
    totalb += len(l)
    print(len(l), l)

r = esp32.RMT2(pin=p, cb=cb, buf=128)
r.read_pulses()
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
    mp_obj_t cb;
    bool continue_rx;
    rmt_channel_handle_t rx_channel;
    rmt_symbol_word_t *symbols;
    size_t symbols_size;
    size_t num_symbols;
    // rmt_rx_done_event_data_t rx_data;
    rmt_receive_config_t rx_config;
    rmt_rx_channel_config_t rx_ch_conf;

    // double buffering of received data
    bool recv_locked;
    size_t recv_num_symbols;
    rmt_symbol_word_t *recv_symbols;
} esp32_rmt2_obj_t;

// Non-public function called by interrupt handler to finish data processing
static mp_obj_t rmt_recv_done_upperhalf(mp_obj_t self_in) {
    esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (self->cb == mp_const_none || !self->continue_rx) {
        self->recv_locked = false;
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

    self->recv_locked = false;

    // callback
    mp_sched_schedule(self->cb, list);

    return mp_const_none;
}

static MP_DEFINE_CONST_FUN_OBJ_1(rmt_recv_done_upperhalf_obj, rmt_recv_done_upperhalf);

// Interrupt handler
static bool IRAM_ATTR rmt_recv_done(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *udata){
    esp32_rmt2_obj_t *self = udata;

    if (!self->recv_locked) {
            self->recv_num_symbols = edata->num_symbols;
            memcpy(self->recv_symbols, edata->received_symbols, edata->num_symbols * sizeof(rmt_symbol_word_t));
            self->recv_locked = true;
    }

    mp_sched_schedule(MP_OBJ_FROM_PTR(&rmt_recv_done_upperhalf_obj), MP_OBJ_FROM_PTR(self));

    if (self->continue_rx) {
        rmt_receive(self->rx_channel, self->symbols, self->symbols_size, &self->rx_config);
    }

    return false;
}

static mp_obj_t esp32_rmt2_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_pin,       MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_cb,        MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_buf,       MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 64} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    gpio_num_t pin_id = machine_pin_get_id(args[0].u_obj);
    mp_obj_t cb = args[1].u_obj;
    if (cb != mp_const_none && !mp_obj_is_callable(cb)) {
        mp_raise_ValueError(MP_ERROR_TEXT("cb is not callable"));
    }
    int num_symbols = args[2].u_int;
    if ((num_symbols < 64) || ((num_symbols % 2) == 1)) {
        mp_raise_ValueError(MP_ERROR_TEXT("buf must be at least 64 and even"));
    }

    esp32_rmt2_obj_t *self = mp_obj_malloc_with_finaliser(esp32_rmt2_obj_t, &esp32_rmt2_type);

    self->cb = cb;
    self->rx_channel = NULL;
    self->num_symbols = num_symbols;
    self->symbols_size = num_symbols * sizeof(rmt_symbol_word_t);
    self->symbols = m_realloc(NULL, self->symbols_size);
    self->recv_symbols = m_realloc(NULL, self->symbols_size);

    // RMT new driver does not have a way to specify a group clock divisor, see
    // https://github.com/espressif/esp-idf/issues/14760#issuecomment-2430770601 and
    // https://github.com/espressif/esp-idf/issues/11262
    // since signal_range_min_ns must fit in an 8-bit register, the value is limited
    // to 3190ns (80MHz clock * 3190 / 1000000000 [conversion to ns] == 255).

    // FIXME configurable
    self->rx_config.signal_range_min_ns = 3100;
    // FIXME configurable
    self->rx_config.signal_range_max_ns = 5 * 1000 * 1000;

    self->rx_ch_conf.gpio_num = self->pin = pin_id;
    self->rx_ch_conf.clk_src = RMT_CLK_SRC_DEFAULT;
    // FIXME configurable
    self->rx_ch_conf.resolution_hz = 1000000;
    self->rx_ch_conf.mem_block_symbols = num_symbols;

    check_esp_err(rmt_new_rx_channel(&self->rx_ch_conf, &self->rx_channel));
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_recv_done,
    };
    // FIXME call from task to move to different core?
    check_esp_err(rmt_rx_register_event_callbacks(self->rx_channel, &cbs, self));
    check_esp_err(rmt_enable(self->rx_channel));

    self->recv_num_symbols = 0;

    return MP_OBJ_FROM_PTR(self);
}

static void esp32_rmt2_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "RMT2 pin=%u buf=%u(x2) %u %s",
                    self->pin, self->num_symbols,
                    self->recv_num_symbols, self->recv_locked ? "locked" : "unlocked");
}


static mp_obj_t esp32_rmt2_deinit(mp_obj_t self_in) {
    esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (self->pin != -1) { // Check if channel has already been deinitialised.
        rmt_disable(self->rx_channel);
        rmt_del_channel(self->rx_channel);
        self->pin = -1;
    }
    self->cb = mp_const_none;
    m_free(self->symbols);
    self->symbols = 0;
    m_free(self->recv_symbols);
    self->recv_symbols = 0;

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_rmt2_deinit_obj, esp32_rmt2_deinit);


static mp_obj_t esp32_rmt2_stop_read_pulses(mp_obj_t self_in) {
    esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);

    bool ret = self->continue_rx;
    self->continue_rx = false;
    return ret ? mp_const_true : mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_rmt2_stop_read_pulses_obj, esp32_rmt2_stop_read_pulses);


static mp_obj_t esp32_rmt2_read_pulses(mp_obj_t self_in) {
    esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);

    self->continue_rx = true;
    self->recv_locked = false;
    self->recv_num_symbols = 0;
    check_esp_err(rmt_receive(self->rx_channel, self->symbols, self->symbols_size, &self->rx_config));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_rmt2_read_pulses_obj, esp32_rmt2_read_pulses);


static const mp_rom_map_elem_t esp32_rmt2_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&esp32_rmt2_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&esp32_rmt2_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_pulses), MP_ROM_PTR(&esp32_rmt2_read_pulses_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_read_pulses), MP_ROM_PTR(&esp32_rmt2_stop_read_pulses_obj) },
};
static MP_DEFINE_CONST_DICT(esp32_rmt2_locals_dict, esp32_rmt2_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    esp32_rmt2_type,
    MP_QSTR_RMT2,
    MP_TYPE_FLAG_NONE,
    make_new, esp32_rmt2_make_new,
    print, esp32_rmt2_print,
    locals_dict, &esp32_rmt2_locals_dict
    );
