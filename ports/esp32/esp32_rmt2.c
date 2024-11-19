#include "py/mphal.h"
#include "py/runtime.h"
#include "modmachine.h"
#include "modesp32.h"

#include "esp_task.h"
#include "driver/rmt_rx.h"

void machine_bitstream_high_low(mp_hal_pin_obj_t pin, uint32_t *timing_ns, const uint8_t *buf, size_t len) {
}

// Forward declaration
extern const mp_obj_type_t esp32_rmt2_type;

typedef struct _esp32_rmt2_obj_t {
    mp_obj_base_t base;
    gpio_num_t pin;
} esp32_rmt2_obj_t;

static rmt_channel_handle_t rx_channel = NULL;
static rmt_symbol_word_t symbols[64];
static rmt_rx_done_event_data_t rx_data;

static rmt_receive_config_t rx_config = {
    .signal_range_min_ns = 3100,
    .signal_range_max_ns = 5 * 1000 * 1000,
};

static rmt_rx_channel_config_t rx_ch_conf = {
    .gpio_num = 0,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000,
    .mem_block_symbols = 64,
};

static mp_obj_t rmt_recv_done_upperhalf(mp_obj_t _) {
    // FIXME handle
    size_t len = rx_data.num_symbols;
    mp_printf(MP_PYTHON_PRINTER, "RMT2 tot=%u ", len);
    rmt_symbol_word_t *data = rx_data.received_symbols;
    for (uint8_t i = 0; i < len; i++) {
        int n0 = (uint16_t) data[i].duration0;
        int n1 = (uint16_t) data[i].duration1;
        if (!data[i].level0) {
            n0 = -n0;
        }
        if (!data[i].level1) {
            n1 = -n1;
        }
        mp_printf(MP_PYTHON_PRINTER, "%d %d ", n0, n1);
    }
    mp_printf(MP_PYTHON_PRINTER, "\n");
    return mp_const_none;
}

static MP_DEFINE_CONST_FUN_OBJ_1(rmt_recv_done_upperhalf_obj, rmt_recv_done_upperhalf);

static bool IRAM_ATTR rmt_recv_done(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *udata){
    rx_data = *edata;
    mp_sched_schedule(MP_OBJ_FROM_PTR(&rmt_recv_done_upperhalf_obj), mp_const_none);
    return false;
}


static esp_err_t rmt_do_install()
{
    rmt_new_rx_channel(&rx_ch_conf, &rx_channel);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_recv_done,
    };
    rmt_rx_register_event_callbacks(rx_channel, &cbs, NULL);
    return rmt_enable(rx_channel);
}


static mp_obj_t esp32_rmt2_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_pin,       MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    gpio_num_t pin_id = machine_pin_get_id(args[0].u_obj);

    esp32_rmt2_obj_t *self = mp_obj_malloc_with_finaliser(esp32_rmt2_obj_t, &esp32_rmt2_type);
    rx_ch_conf.gpio_num = self->pin = pin_id;

    check_esp_err(rmt_do_install());
    rx_data.num_symbols = 0;

    return MP_OBJ_FROM_PTR(self);
}

static void esp32_rmt2_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    // esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);

    mp_printf(print, "RMT2 %u", rx_data.num_symbols);
}


static mp_obj_t esp32_rmt2_deinit(mp_obj_t self_in) {
    esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (self->pin != -1) { // Check if channel has already been deinitialised.
        rmt_disable(rx_channel);
        rmt_del_channel(rx_channel);
        self->pin = -1;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_rmt2_deinit_obj, esp32_rmt2_deinit);


static mp_obj_t esp32_rmt2_stop_read_pulses(mp_obj_t self_in) {
    // esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);

    esp_err_t err = rmt_disable(rx_channel);
    rx_data.num_symbols = 0;
    return err == ESP_OK ? mp_const_true : mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_rmt2_stop_read_pulses_obj, esp32_rmt2_stop_read_pulses);


static mp_obj_t esp32_rmt2_read_pulses(mp_obj_t self_in) {
    // esp32_rmt2_obj_t *self = MP_OBJ_TO_PTR(self_in);

    rx_data.num_symbols = 0;
    check_esp_err(rmt_receive(rx_channel, symbols, sizeof(symbols), &rx_config));
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
