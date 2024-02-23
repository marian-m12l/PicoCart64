#ifndef _JOYBUS_JOYBUS_H
#define _JOYBUS_JOYBUS_H

#include <hardware/pio.h>
#include <pico/stdlib.h>

typedef struct {
    uint pin;
    PIO pio;
    uint sm;
    uint offset;
    pio_sm_config config;
} joybus_port_t;

#ifdef __cplusplus
extern "C" {
#endif

void joybus_port_init_rx(joybus_port_t *port_rx, uint pin, irq_handler_t handler);
void joybus_port_init_tx(joybus_port_t *port_tx, uint pin, irq_handler_t handler);
void joybus_port_terminate_rx(joybus_port_t *port_rx);
void joybus_port_terminate_tx(joybus_port_t *port_tx);
void joybus_send_bytes(joybus_port_t *port_tx, uint8_t *bytes, uint len);
void joybus_send_byte(joybus_port_t *port_tx, uint8_t byte, bool stop);

#ifdef __cplusplus
}
#endif

#endif
