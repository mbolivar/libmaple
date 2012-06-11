/**
 * @file examples/test-usart-dma.cpp
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 *
 * Simple test of DMA used with a USART receiver.
 *
 * Configures a USART receiver for use with DMA.  Received bytes are
 * placed into a buffer, with an interrupt firing when the buffer is
 * full.  At that point, the USART transmitter will print the contents
 * of the byte buffer.  The buffer is continually filled and refilled
 * in this manner.
 *
 * This example isn't very robust; don't use it in production.  In
 * particular, since the buffer keeps filling (DMA_CIRC_MODE is set),
 * if you keep typing after filling the buffer, you'll overwrite
 * earlier bytes; this may happen before those earlier bytes are done
 * printing.
 *
 * This code is released into the public domain.
 */

#include <libmaple/dma.h>
#include <libmaple/usart.h>
#include <libmaple/gpio.h>
#include <libmaple/stm32.h>

#include <wirish/wirish.h>

#define BAUD 9600

#define USART USART2
#define USART_HWSER Serial2
#define USART_DMA_DEV DMA1
#define USART_RX_DMA_REQ DMA_REQ_SRC_USART2_RX

#if STM32_MCU_SERIES == STM32_SERIES_F1
#define USART_RX_DMA_TUBE DMA_CH6
#elif ((STM32_MCU_SERIES == STM32_SERIES_F2) ||   \
       (STM32_MCU_SERIES == STM32_SERIES_F4))
#define USART_RX_DMA_TUBE DMA_S5
#else
#error "Unknown STM32 series; can't pick a tube"
#endif

#define BUF_SIZE 8
uint8 rx_buf[BUF_SIZE];

// DMA mode flags:
//
// - DMA_MINC_MODE so we fill up rx_buf,
// - DMA_CIRC_MODE so we go back to the beginning of rx_buf when it's
//   full, and
// - DMA_TRNS_CMPLT so we get interrupted when the transfer completes.
uint32 mode_flags = (DMA_MINC_MODE |
                     DMA_CIRC_MODE |
                     DMA_TRNS_CMPLT);

// DMA tube configuration for USART RX.
dma_tube_config usart_tube_config = {
    &USART->regs->DR, // .tube_per_addr
    DMA_SIZE_8BITS,   // .tube_per_size
    rx_buf,           // .tube_mem_addr
    DMA_SIZE_8BITS,   // .tube_mem_size
    BUF_SIZE,         // .tube_nr_xfers
    mode_flags,       // .tube_mode
    USART_RX_DMA_REQ, // .tube_req_src
    NULL,             // .series_data
};

dma_irq_cause irq_cause;

volatile uint32 irq_fired = 0;

void init_usart(void);
void init_dma_xfer(void);
void rx_dma_irq(void);

void setup(void) {
    pinMode(BOARD_LED_PIN, OUTPUT);

    init_dma_xfer();
    init_usart();
}

void loop(void) {
    toggleLED();
    delay(100);

    if (irq_fired) {
        USART_HWSER.println("** IRQ **");
        irq_fired = 0;
    }
    USART_HWSER.print("[");
    USART_HWSER.print(millis());
    USART_HWSER.print("]\tISR bits: 0x");
    uint8 isr_bits = dma_get_isr_bits(USART_DMA_DEV, USART_RX_DMA_TUBE);
    USART_HWSER.print(isr_bits, HEX);
    USART_HWSER.print("\tBuffer contents: ");
    for (int i = 0; i < BUF_SIZE; i++) {
        USART_HWSER.print('\'');
        USART_HWSER.print(rx_buf[i]);
        USART_HWSER.print('\'');
        if (i < BUF_SIZE - 1) USART_HWSER.print(", ");
    }
    USART_HWSER.println();
    if (isr_bits == 0x7) {
        USART_HWSER.println("** Clearing ISR bits.");
        dma_clear_isr_bits(USART_DMA_DEV, USART_RX_DMA_TUBE);
    }
}

/* Configure USART receiver for use with DMA */
void init_usart(void) {
    USART_HWSER.begin(BAUD);
    USART->regs->CR3 = USART_CR3_DMAR;
}

/* Configure DMA transmission */
void init_dma_xfer(void) {
    dma_init(USART_DMA_DEV);
    dma_tube_cfg(USART_DMA_DEV, USART_RX_DMA_TUBE, &usart_tube_config);
    dma_attach_interrupt(USART_DMA_DEV, USART_RX_DMA_TUBE, rx_dma_irq);
    dma_enable(USART_DMA_DEV, USART_RX_DMA_TUBE);
}

void rx_dma_irq(void) {
    irq_fired = true;
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
    setup();

    while (true) {
        loop();
    }
    return 0;
}
