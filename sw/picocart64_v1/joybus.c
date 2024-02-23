#include "joybus.h"

#include "joybus_rx.pio.h"
#include "joybus_tx.pio.h"

#include <hardware/pio.h>
#include <pico/stdlib.h>

void joybus_port_init_rx(joybus_port_t *port_rx, uint pin, irq_handler_t handler) {
	PIO pio = pio1;
    int sm = pio_claim_unused_sm(pio, true);
    int offset = pio_add_program(pio, &joybus_rx_program);

    port_rx->pin = pin;
    port_rx->pio = pio;
    port_rx->sm = sm;
    port_rx->offset = offset;
    port_rx->config = joybus_rx_program_get_config(pio, sm, offset, pin);

    int8_t pio_system_irq_0 = (port_rx->pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0;
    irq_set_exclusive_handler(pio_system_irq_0, handler);
    pio_set_irq0_source_enabled(port_rx->pio, PIO_INTR_SM0_LSB, true);  // Route pio IRQ0 to system IRQ "pio 0"
    irq_set_enabled(pio_system_irq_0, true);

    joybus_rx_program_receive_init(port_rx->pio, port_rx->sm, port_rx->offset, port_rx->pin, &port_rx->config);
}

void joybus_port_init_tx(joybus_port_t *port_tx, uint pin, irq_handler_t handler) {
	PIO pio = pio1;
    int sm = pio_claim_unused_sm(pio, true);
    int offset = pio_add_program(pio, &joybus_tx_program);

    port_tx->pin = pin;
    port_tx->pio = pio;
    port_tx->sm = sm;
    port_tx->offset = offset;
    port_tx->config = joybus_tx_program_get_config(pio, sm, offset, pin);

    int8_t pio_system_irq_1 = (port_tx->pio == pio0) ? PIO0_IRQ_1 : PIO1_IRQ_1;
    irq_set_exclusive_handler(pio_system_irq_1, handler);
    pio_set_irq1_source_enabled(port_tx->pio, PIO_INTR_SM1_LSB, true);    // Route pio IRQ1 to system IRQ "pio 1"
    irq_set_enabled(pio_system_irq_1, true);

    joybus_tx_program_send_init(port_tx->pio, port_tx->sm, port_tx->offset, port_tx->pin, &port_tx->config);
}

void joybus_port_terminate_rx(joybus_port_t *port_rx) {
    pio_sm_set_enabled(port_rx->pio, port_rx->sm, false);
    pio_sm_unclaim(port_rx->pio, port_rx->sm);
    pio_remove_program(port_rx->pio, &joybus_rx_program, port_rx->offset);
    int8_t pio_system_irq_0 = (port_rx->pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0;
    irq_remove_handler(pio_system_irq_0, irq_get_exclusive_handler(pio_system_irq_0));
}

void joybus_port_terminate_tx(joybus_port_t *port_tx) {
    pio_sm_set_enabled(port_tx->pio, port_tx->sm, false);
    pio_sm_unclaim(port_tx->pio, port_tx->sm);
    pio_remove_program(port_tx->pio, &joybus_tx_program, port_tx->offset);
    int8_t pio_system_irq_1 = (port_tx->pio == pio0) ? PIO0_IRQ_1 : PIO1_IRQ_1;
    irq_remove_handler(pio_system_irq_1, irq_get_exclusive_handler(pio_system_irq_1));
}

void __no_inline_not_in_flash_func(joybus_send_bytes)(
    joybus_port_t *port_tx,
    uint8_t *bytes,
    uint len
) {
    // Wait for line to be high before sending anything.
    while (!gpio_get(port_tx->pin)) {
        tight_loop_contents();
    }

    joybus_tx_program_send_init(port_tx->pio, port_tx->sm, port_tx->offset, port_tx->pin, &port_tx->config);

    for (int i = 0; i < len; i++) {
        joybus_send_byte(port_tx, bytes[i], i == len - 1);
    }
}

void __no_inline_not_in_flash_func(joybus_send_byte)(joybus_port_t *port_tx, uint8_t byte, bool stop) {
    uint32_t data_shifted = (byte << 24) | (stop << 23);
    pio_sm_put_blocking(port_tx->pio, port_tx->sm, data_shifted);
}
