/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Michael Hope.
 * Copyright (c) 2012 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file libmaple/stm32f1/dma.c
 * @author Marti Bolivar <mbolivar@leaflabs.com>;
 *         Original implementation by Michael Hope
 * @brief STM32F1 DMA support.
 */

#include <libmaple/dma.h>
#include <libmaple/bitband.h>

/*
 * Devices
 */

static dma_dev dma1 = {
    .regs     = DMA1_BASE,
    .clk_id   = RCC_DMA1,
    .handlers = {{ .handler = NULL, .irq_line = NVIC_DMA_CH1 },
                 { .handler = NULL, .irq_line = NVIC_DMA_CH2 },
                 { .handler = NULL, .irq_line = NVIC_DMA_CH3 },
                 { .handler = NULL, .irq_line = NVIC_DMA_CH4 },
                 { .handler = NULL, .irq_line = NVIC_DMA_CH5 },
                 { .handler = NULL, .irq_line = NVIC_DMA_CH6 },
                 { .handler = NULL, .irq_line = NVIC_DMA_CH7 }},
};
/** STM32F1 DMA1 device */
dma_dev *DMA1 = &dma1;

#if defined(STM32_HIGH_DENSITY) || defined(STM32_XL_DENSITY)
static dma_dev dma2 = {
    .regs     = DMA2_BASE,
    .clk_id   = RCC_DMA2,
    .handlers = {{ .handler = NULL, .irq_line = NVIC_DMA2_CH1   },
                 { .handler = NULL, .irq_line = NVIC_DMA2_CH2   },
                 { .handler = NULL, .irq_line = NVIC_DMA2_CH3   },
                 { .handler = NULL, .irq_line = NVIC_DMA2_CH_4_5 },
                 { .handler = NULL, .irq_line = NVIC_DMA2_CH_4_5 }}, /* !@#$ */
};
/** STM32F1 DMA2 device */
dma_dev *DMA2 = &dma2;
#endif

/*
 * Routines
 */

int dma_tube_cfg(dma_dev *dev, dma_channel channel, dma_tube_config *cfg) {
    dma_channel_reg_map *channel_regs;

    /* Make sure config specifies a request source that dev/channel
     * can actually serve. If not, the request is impossible. If so,
     * we can otherwise safely ignore tube_req_src on this series. */
    if (config->tube_req_src != (dma_request_src)((dev->clk_id << 3) |
                                                  channel)) {
        return DMA_TUBE_CFG_ECOMPAT;
    }

    dma_disable(dev, channel);

    channel_regs = dma_channel_regs(dev, channel);
    channel_regs->CCR = ((config->tube_mem_size << 10) |
                         (config->tube_per_size << 8) |
                         config->tube_mode);
    channel_regs->CNDTR = config->tube_nr_xfers;
    channel_regs->CMAR = (uint32)config->tube_mem_addr;
    channel_regs->CPAR = (uint32)config->tube_per_addr;
    return DMA_TUBE_CFG_SUCCESS;
}

void dma_set_priority(dma_dev *dev,
                      dma_channel channel,
                      dma_priority priority) {
    dma_channel_reg_map *channel_regs;
    uint32 ccr;

    ASSERT_FAULT(!dma_is_channel_enabled(dev, channel));

    channel_regs = dma_channel_regs(dev, channel);
    ccr = channel_regs->CCR;
    ccr &= ~DMA_CCR_PL;
    ccr |= (priority << 12);
    channel_regs->CCR = ccr;
}

void dma_set_num_transfers(dma_dev *dev,
                           dma_channel channel,
                           uint16 num_transfers) {
    dma_channel_reg_map *channel_regs;

    ASSERT_FAULT(!dma_is_channel_enabled(dev, channel));

    channel_regs = dma_channel_regs(dev, channel);
    channel_regs->CNDTR = num_transfers;
}

void dma_attach_interrupt(dma_dev *dev, dma_channel channel,
                          void (*handler)(void)) {
    dev->handlers[channel - 1].handler = handler;
    nvic_irq_enable(dev->handlers[channel - 1].irq_line);
}

void dma_detach_interrupt(dma_dev *dev, dma_channel channel) {
    /* Don't use nvic_irq_disable()! Think about DMA2 channels 4 and 5. */
    dma_channel_regs(dev, channel)->CCR &= ~0xF;
    dev->handlers[channel - 1].handler = NULL;
}

void dma_enable(dma_dev *dev, dma_channel channel) {
    dma_channel_reg_map *chan_regs = dma_channel_regs(dev, channel);
    bb_peri_set_bit(&chan_regs->CCR, DMA_CCR_EN_BIT, 1);
}

void dma_disable(dma_dev *dev, dma_channel channel) {
    dma_channel_reg_map *chan_regs = dma_channel_regs(dev, channel);
    bb_peri_set_bit(&chan_regs->CCR, DMA_CCR_EN_BIT, 0);
}

dma_irq_cause dma_get_irq_cause(dma_dev *dev, dma_channel channel) {
    /* Grab and clear the ISR bits. */
    uint8 status_bits = dma_get_isr_bits(dev, channel);
    dma_clear_isr_bits(dev, channel);

    /* If the channel global interrupt flag is cleared, then
     * something's very wrong. */
    ASSERT(status_bits & 0x1);
    /* If GIF is set, then some other flag should be set, barring
     * something unexpected (e.g. the user making an unforeseen IFCR
     * write). */
    ASSERT(status_bits != 0x1);

    /* ISR flags get set even if the corresponding interrupt enable
     * bits in the channel's configuration register are cleared, so we
     * can't use a switch here.
     *
     * Don't change the order of these if statements. */
    if (status_bits & 0x8) {
        return DMA_TRANSFER_ERROR;
    } else if (status_bits & 0x2) {
        return DMA_TRANSFER_COMPLETE;
    } else if (status_bits & 0x4) {
        return DMA_TRANSFER_HALF_COMPLETE;
    }

    /* If we get here, one of our assumptions has been violated, but
     * the debug level is too low for the above ASSERTs() to have had
     * any effect. In order to fail fast, mimic the DMA controller's
     * behavior when an error occurs. */
    dma_disable(dev, channel);
    return DMA_TRANSFER_ERROR;
}

void dma_set_mem_addr(dma_dev *dev, dma_channel channel, __io void *addr) {
    dma_channel_reg_map *chan_regs;

    ASSERT_FAULT(!dma_is_channel_enabled(dev, channel));

    chan_regs = dma_channel_regs(dev, channel);
    chan_regs->CMAR = (uint32)addr;
}

void dma_set_per_addr(dma_dev *dev, dma_channel channel, __io void *addr) {
    dma_channel_reg_map *chan_regs;

    ASSERT_FAULT(!dma_is_channel_enabled(dev, channel));

    chan_regs = dma_channel_regs(dev, channel);
    chan_regs->CPAR = (uint32)addr;
}

/**
 * @brief Deprecated. Use dma_tube_cfg() instead.
 *
 * Set up a DMA transfer.
 *
 * The channel will be disabled before being reconfigured.  The
 * transfer will have low priority by default.  You may choose another
 * priority before the transfer begins using dma_set_priority(), as
 * well as performing any other configuration you desire.  When the
 * channel is configured to your liking, enable it using dma_enable().
 *
 * @param dev DMA device.
 * @param channel DMA channel.
 * @param peripheral_address Base address of peripheral data register
 *                           involved in the transfer.
 * @param peripheral_size Peripheral data transfer size.
 * @param memory_address Base memory address involved in the transfer.
 * @param memory_size Memory data transfer size.
 * @param mode Logical OR of dma_mode_flags
 *
 * @see dma_tube_cfg()
 *
 * @sideeffect Disables the given DMA channel.
 * @see dma_xfer_size
 * @see dma_mode_flags
 * @see dma_set_num_transfers()
 * @see dma_set_priority()
 * @see dma_attach_interrupt()
 * @see dma_enable()
 */
__deprecated
void dma_setup_transfer(dma_dev       *dev,
                        dma_channel    channel,
                        __io void     *peripheral_address,
                        dma_xfer_size  peripheral_size,
                        __io void     *memory_address,
                        dma_xfer_size  memory_size,
                        uint32         mode) {
    dma_channel_reg_map *channel_regs = dma_channel_regs(dev, channel);

    dma_disable(dev, channel);  /* can't write to CMAR/CPAR otherwise */
    channel_regs->CCR = (memory_size << 10) | (peripheral_size << 8) | mode;
    channel_regs->CMAR = (uint32)memory_address;
    channel_regs->CPAR = (uint32)peripheral_address;
}

/*
 * IRQ handlers
 */

static __always_inline void dispatch_handler(dma_dev *dev, int channel) {
    void (*handler)(void) = dev->handlers[channel - 1].handler;
    if (handler) {
        handler();
        dma_clear_isr_bits(dev, channel); /* in case handler doesn't */
    }
}

void __irq_dma1_channel1(void) {
    dispatch_handler(DMA1, DMA_CH1);
}

void __irq_dma1_channel2(void) {
    dispatch_handler(DMA1, DMA_CH2);
}

void __irq_dma1_channel3(void) {
    dispatch_handler(DMA1, DMA_CH3);
}

void __irq_dma1_channel4(void) {
    dispatch_handler(DMA1, DMA_CH4);
}

void __irq_dma1_channel5(void) {
    dispatch_handler(DMA1, DMA_CH5);
}

void __irq_dma1_channel6(void) {
    dispatch_handler(DMA1, DMA_CH6);
}

void __irq_dma1_channel7(void) {
    dispatch_handler(DMA1, DMA_CH7);
}

#if defined(STM32_HIGH_DENSITY) || defined(STM32_XL_DENSITY)
void __irq_dma2_channel1(void) {
    dispatch_handler(DMA2, DMA_CH1);
}

void __irq_dma2_channel2(void) {
    dispatch_handler(DMA2, DMA_CH2);
}

void __irq_dma2_channel3(void) {
    dispatch_handler(DMA2, DMA_CH3);
}

void __irq_dma2_channel4_5(void) {
    if ((DMA2_BASE->CCR4 & DMA_CCR_EN) && (DMA2_BASE->ISR & DMA_ISR_GIF4)) {
        dispatch_handler(DMA2, DMA_CH4);
    }
    if ((DMA2_BASE->CCR5 & DMA_CCR_EN) && (DMA2_BASE->ISR & DMA_ISR_GIF5)) {
        dispatch_handler(DMA2, DMA_CH5);
    }
}
#endif
