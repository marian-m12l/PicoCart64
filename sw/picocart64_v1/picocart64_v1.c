/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Konrad Beckmann
 */

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/irq.h"

#include "stdio_async_uart.h"

#include "n64_cic.h"
#include "git_info.h"
#include "n64_pi_task.h"
#include "picocart64_pins.h"
#include "sram.h"
#include "eeprom.h"
#include "joybus.h"
#include "utils.h"

#define UART_TX_PIN (28)
#define UART_RX_PIN (29)		/* not available on the pico */
#define UART_ID     uart0
#define BAUD_RATE   115200

#define ENABLE_N64_PI 1

// Priority 0 = lowest, 3 = highest
#define CIC_TASK_PRIORITY     (3UL)
#define SECOND_TASK_PRIORITY  (1UL)
#define EEPROM_TASK_PRIORITY  (1UL)

static StaticTask_t cic_task;
static StaticTask_t second_task;
static StaticTask_t eeprom_task;
static StackType_t cic_task_stack[4 * 1024 / sizeof(StackType_t)];
static StackType_t second_task_stack[4 * 1024 / sizeof(StackType_t)];
static StackType_t eeprom_task_stack[4 * 1024 / sizeof(StackType_t)];

uint32_t g_flash_jedec_id;

/*

Profiling results:

Time between ~N64_READ and bit output on AD0

133 MHz old code:
    ROM:  1st _980_ ns, 2nd 500 ns
    SRAM: 1st  500  ns, 2nd 510 ns

133 MHz new code:
    ROM:  1st _300_ ns, 2nd 280 ns
    SRAM: 1st  320  ns, 2nd 320 ns

266 MHz new code:
    ROM:  1st  180 ns, 2nd 180 ns (sometimes down to 160, but only worst case matters)
    SRAM: 1st  160 ns, 2nd 160 ns

*/

// FreeRTOS boilerplate
void vApplicationGetTimerTaskMemory(StaticTask_t ** ppxTimerTaskTCBBuffer, StackType_t ** ppxTimerTaskStackBuffer, uint32_t * pulTimerTaskStackSize)
{
	static StaticTask_t xTimerTaskTCB;
	static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void save_to_flash(void)
{
	sram_save_to_flash();
	eeprom_save_to_flash();
}

void cic_task_entry(__unused void *params)
{
	printf("cic_task_entry\n");

	// Load SRAM backup from external flash
	// TODO: How do we detect if it's uninitialized (config area in flash?),
	//       or maybe we don't have to care?
	sram_load_from_flash();

	// Load EEPROM from flash
	eeprom_load_from_flash();

	n64_cic_hw_init();
	// n64_cic_reset_parameters();
	// n64_cic_set_parameters(params);
	// n64_cic_set_dd_mode(false);

	// TODO: Performing the write to flash in a separate task is the way to go
	n64_cic_task(save_to_flash);
}

void second_task_entry(__unused void *params)
{
	uint32_t count = 0;

	printf("second_task_entry\n");

	while (true) {
		vTaskDelay(1000);
		count++;

		// Set to 1 to print stack watermarks.
		// Printing is synchronous and interferes with the CIC emulation.
#if 0
		// printf("Second task heartbeat: %d\n", count);
		// vPortYield();

		if (count > 10) {
			printf("watermark: %d\n", uxTaskGetStackHighWaterMark(NULL));
			vPortYield();

			printf("watermark second_task: %d\n", uxTaskGetStackHighWaterMark((TaskHandle_t) & second_task));
			vPortYield();

			printf("watermark cic_task: %d\n", uxTaskGetStackHighWaterMark((TaskHandle_t) & cic_task));
			vPortYield();
		}
#endif

	}
}


// S_DAT pin
#define JOYBUS_PIN 26

// Populated by IRQ handler with data incoming from pio program / console S_DAT
volatile uint32_t incoming[4];
volatile uint8_t incoming_length = 0;

// Joybus abstraction
joybus_port_t joybus_rx_port;
joybus_port_t joybus_tx_port;

#define PROBE 0x00
#define RESET 0xFF
#define READ_EEPROM 0x04
#define WRITE_EEPROM 0x05

#define EEPROM_4K 0x8000
#define EEPROM_16K 0xC000

typedef struct __attribute__((packed)) {
    uint16_t device;
    uint8_t status;
} n64_status_t;

n64_status_t eeprom_status = (n64_status_t){
  .device = EEPROM_16K,
  .status = 0x00,
};

static void pio_rx_irq_func(void) {
  // Read all incoming words from pio RX FIFO
  incoming_length = 0;
  while (!pio_sm_is_rx_fifo_empty(joybus_rx_port.pio, joybus_rx_port.sm)) {
    incoming[incoming_length++] = pio_sm_get_blocking(joybus_rx_port.pio, joybus_rx_port.sm);
  }
  pio_interrupt_clear(joybus_rx_port.pio, 0);
}

static void pio_tx_irq_func(void) {
  // Done transmitting, switch to rx program
  joybus_port_terminate_tx(&joybus_tx_port);
  pio_interrupt_clear(joybus_tx_port.pio, 1);
  joybus_port_init_rx(&joybus_rx_port, JOYBUS_PIN, pio_rx_irq_func);
}

void send_response(uint8_t* data, uint8_t length) {
  // Switch to tx program
  joybus_port_terminate_rx(&joybus_rx_port);
  joybus_port_init_tx(&joybus_tx_port, JOYBUS_PIN, pio_tx_irq_func);
  joybus_send_bytes(&joybus_tx_port, data, length);
}

uint32_t swap32(uint32_t data) {
  return ((data>>24)&0xff) |
          ((data<<8)&0xff0000) |
          ((data>>8)&0xff00) |
          ((data<<24)&0xff000000);
}

void eeprom_task_entry(__unused void *params)
{
	printf("eeprom_task_entry\n");

	while (true) {
		// TODO delay ?? vTaskDelay(1000);
		
		// TODO yield?? vPortYield();

		//tight_loop_contents();
		vPortYield();

		if (incoming_length > 0) {
			// Read and handle incoming command
			uint8_t command = incoming[0] & 0xff;
			switch (command) {
			case RESET:
			case PROBE:
				send_response((uint8_t *)&eeprom_status, sizeof(n64_status_t));
				break;
			case READ_EEPROM:
				send_response(&eeprom[incoming[1]*8], 8);
				break;
			case WRITE_EEPROM:
				send_response(0, 1);
				uint8_t block = (incoming[1] >> 24);
				uint8_t* address = (uint8_t*)(eeprom) + block*8;
				uint32_t data1 = swap32((incoming[1] << 8) | (incoming[2] >> 24));
				uint32_t data2 = swap32((incoming[2] << 8) | (incoming[3] & 0xff));
				memcpy(address, &data1, 4);
				memcpy(address + 4, &data2, 4);
				break;
			}

			incoming_length = 0;
		}
	}
}

void vLaunch(void)
{
	xTaskCreateStatic(cic_task_entry, "CICThread", configMINIMAL_STACK_SIZE, NULL, CIC_TASK_PRIORITY, cic_task_stack, &cic_task);
	xTaskCreateStatic(second_task_entry, "SecondThread", configMINIMAL_STACK_SIZE, NULL, SECOND_TASK_PRIORITY, second_task_stack, &second_task);
	xTaskCreateStatic(eeprom_task_entry, "EEPROMThread", configMINIMAL_STACK_SIZE, NULL, EEPROM_TASK_PRIORITY, eeprom_task_stack, &eeprom_task);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();
}

#include "rom_vars.h"

uint32_t flash_get_jedec_id(void)
{
	const uint8_t read_jedec_id = 0x9f;
	uint8_t txbuf[4] = { read_jedec_id };
	uint8_t rxbuf[4] = { 0 };
	txbuf[0] = read_jedec_id;
	flash_do_cmd(txbuf, rxbuf, 4);

	return rxbuf[1] | (rxbuf[2] << 8) | (rxbuf[3] << 16);
}

int main(void)
{
	// First, let's probe the Flash ID
	g_flash_jedec_id = flash_get_jedec_id();

	// Overclock!
	// The external flash should be rated to 133MHz,
	// but since it's used with a 2x clock divider,
	// 266 MHz is safe in this regard.

	// set_sys_clock_khz(133000, true);
	set_sys_clock_khz(266000, true);	// Required for SRAM @ 200ns

	// Init GPIOs before starting the second core and FreeRTOS
	for (int i = 0; i <= 27; i++) {
		gpio_init(i);
		gpio_set_dir(i, GPIO_IN);
		gpio_set_pulls(i, false, false);
	}

	// Set up ROM mapping table
	if (memcmp(picocart_header, "picocartcompress", 16) == 0) {
		// Copy rom compressed map from flash into RAM
		memcpy(rom_mapping, flash_rom_mapping, MAPPING_TABLE_LEN * sizeof(uint16_t));
	} else {
		for (int i = 0; i < MAPPING_TABLE_LEN; i++) {
			rom_mapping[i] = i;
		}
	}

	// Enable pull up on N64_CIC_DIO since there is no external one.
	gpio_pull_up(N64_CIC_DIO);
	

  	joybus_port_init_rx(&joybus_rx_port, JOYBUS_PIN, pio_rx_irq_func);

	// Init UART on pin 28/29
	stdio_async_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);
	printf("PicoCart64 Boot (git rev %08x)\r\n", GIT_REV);

#if ENABLE_N64_PI
	// Launch the N64 PI implementation in the second core
	// Note! You have to power reset the pico after flashing it with a jlink,
	//       otherwise multicore doesn't work properly.
	//       Alternatively, attach gdb to openocd, run `mon reset halt`, `c`.
	//       It seems this works around the issue as well.
	multicore_launch_core1(n64_pi_run);
#endif

	// Start FreeRTOS on Core0
	vLaunch();

	return 0;
}
