/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2010, 2011, 2012 LeafLabs, LLC.
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
 * @file libmaple/adc.c
 * @author Marti Bolivar <mbolivar@leaflabs.com>,
 *         Perry Hung <perry@leaflabs.com>
 * @brief Analog to digital converter routines
 */

#include <libmaple/adc.h>
#include <libmaple/libmaple.h>
#include <libmaple/rcc.h>

#include "adc_private.h"

/**
 * @brief Initialize an ADC peripheral.
 *
 * Initializes the RCC clock line for the given peripheral.  Resets
 * ADC device registers.
 *
 * @param dev ADC peripheral to initialize
 */
void adc_init(const adc_dev *dev) {
    rcc_clk_enable(dev->clk_id);
    rcc_reset_dev(dev->clk_id);
}

/**
 * @brief Set external event select for regular group
 * @param dev ADC device
 * @param event Event used to trigger the start of conversion.
 * @see adc_extsel_event
 */
void adc_set_extsel(const adc_dev *dev, adc_extsel_event event) {
    uint32 cr2 = dev->regs->CR2;
    cr2 &= ~ADC_CR2_EXTSEL;
    cr2 |= event;
    dev->regs->CR2 = cr2;
}

/**
 * @brief Set the sample rate for all channels on an ADC device.
 *
 * Don't call this during conversion.
 *
 * @param dev adc device
 * @param smp_rate sample rate to set
 * @see adc_smp_rate
 */
void adc_set_sample_rate(const adc_dev *dev, adc_smp_rate smp_rate) {
    uint32 adc_smpr1_val = 0, adc_smpr2_val = 0;
    int i;

    for (i = 0; i < 10; i++) {
        if (i < 8) {
            /* ADC_SMPR1 determines sample time for channels [10,17] */
            adc_smpr1_val |= smp_rate << (i * 3);
        }
        /* ADC_SMPR2 determines sample time for channels [0,9] */
        adc_smpr2_val |= smp_rate << (i * 3);
    }

    dev->regs->SMPR1 = adc_smpr1_val;
    dev->regs->SMPR2 = adc_smpr2_val;
}

/**
 * @brief Enable scan mode for an ADC.
 *
 * In scan mode, the ADC converts all channels in a regular or
 * injected sequence. After each conversion is done, the ADC
 * automatically starts converting the next channel in the sequence.
 *
 * Scan mode is disabled by default.
 *
 * @see adc_disable_scan()
 */
void adc_enable_scan(const adc_dev *dev) {
    bb_peri_set_bit(&dev->regs->CR1, ADC_CR1_SCAN_BIT, 1);
}

/**
 * @brief Disable scan mode for an ADC.
 *
 * This is the default setting.
 *
 * @see adc_enable_scan()
 */
void adc_disable_scan(const adc_dev *dev) {
    bb_peri_set_bit(&dev->regs->CR1, ADC_CR1_SCAN_BIT, 0);
}

/**
 * @brief Enable continuous mode for an ADC.
 *
 * In this mode, the ADC will automatically perform conversions
 * continuously until taken out of this mode or disabled.
 *
 * Continuous mode is disabled by default.
 *
 * @see adc_disable_continuous()
 */
void adc_enable_continuous(const adc_dev *dev) {
    bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_CONT_BIT, 1);
}

/**
 * @brief Disable continuous mode for an ADC.
 *
 * This is the default setting.
 *
 * @see adc_enable_continuous()
 */
void adc_disable_continuous(const adc_dev *dev) {
    bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_CONT_BIT, 0);
}

#define BITS_PER_SQ 5
#define SQs_PER_SQR 6
/**
 * @brief Set the sequence of channels to convert.
 *
 * This sets the (regular) sequence of up to 16 channels to convert.
 *
 * @param dev ADC device
 * @param channels ADC channels to convert; these can repeat and may
 *                 be in any order.
 * @param len Length of `channels', from 1 to 16.
 * @see adc_start_conv()
 */
void adc_set_conv_seq(const adc_dev *dev, const uint8 *channels, uint8 len) {
    const uint8 *end;
    __io uint32 *sqr = &dev->regs->SQR3; /* Next SQR to write to */
    uint32 val = 0;                      /* SQR we're building */
    unsigned sq = 0;                     /* SQ in sqr to set next */
    ASSERT(0 < len && len <= 16);

    end = channels + len;
    do {
        val |= *channels << (BITS_PER_SQ * sq++);
        if (sq == SQs_PER_SQR) {
            /* Finished building sqr in val. Set it and move on. This
             * relies on the ADC_SQRx being contiguous. */
            *sqr-- = val;
            val = 0;
            sq = 0;
        }
    } while (++channels < end);
    /* Write the last SQR, in case SQs_PER_SQR doesn't divide len. */
    *sqr = val;
    adc_set_reg_seqlen(dev, len);
}

/**
 * @brief Perform a single synchronous software triggered conversion on a
 *        channel.
 * @param dev ADC device to use for reading.
 * @param channel channel to convert
 * @return conversion result
 */
uint16 adc_read(const adc_dev *dev, uint8 channel) {
    adc_set_conv_seq(dev, &channel, 1);
    adc_start_conv(dev);
    while (!adc_is_conv_complete(dev))
        ;

    return (uint16)adc_get_data(dev);
}

/**
 * @brief Attach an interrupt handler and enable its interrupts.
 *
 * This function sets `handler' as the function to be called when any
 * ADC interrupts for `dev' occur. At most one ADC interrupt handler
 * may be attached at any time. Subsequent calls to this function will
 * overwrite any previously attached handler.
 *
 * When `handler' is called, its argument will point to a struct
 * adc_callback_data. The .irq_flags field in this struct will be a
 * logical OR of adc_interrupt_id values encoding the reason(s) for
 * the call. Its .arg field will be the `arg' argument to this
 * function.
 *
 * The interrupt bits set in the adc_callback_data's .irq_flags will
 * always be a subset of those set in the `interrupt_flags' argument
 * to this function. That is, interrupts not given here in the
 * `interrupt_flags' argument will never cause `handler' to be
 * called. This has the effect that any enabled ADC interrupts not
 * specified in `interrupt_flags' will be ignored.
 *
 * This function additionally enables the ADC interrupts specified by
 * `interrupt_flags'.
 *
 * @param dev ADC device whose interrupts to attach to.
 * @param interrupt_flags Logical or of adc_interrupt_id values
 *                        specifying interrupts to enable.
 * @param handler Interrupt handler to call when any interrupt in
 *                interrupt_flags occurs.
 * @param arg Value to store in .arg field of handler's callback data.
 * @see enum adc_interrupt_id
 * @see struct adc_callback_data
 */
void adc_attach_interrupt(const adc_dev *dev, uint32 interrupt_flags,
                          void (*handler)(adc_callback_data*), void *arg) {
    struct adc_private_data *priv = dev->priv;
    priv->handler = handler;
    priv->handler_flags = interrupt_flags;
    priv->cb_data.arg = arg;
    adc_enable_interrupts(dev, interrupt_flags);
}

/**
 * @brief Disable ADC interrupts and detach interrupt handlers.
 *
 * This function disables all interrupts for `dev', and unsets any
 * handler attached with adc_attach_interrupt().
 *
 * @param dev ADC device whose handler to detach.
 */
void adc_detach_interrupt(const adc_dev *dev) {
    struct adc_private_data *priv;
    adc_disable_interrupts(dev, ADC_ALL_INTERRUPTS);
    priv = dev->priv;
    priv->handler = NULL;
    priv->handler_flags = 0;
}

#ifdef ADC_SR_OVR
#define ADC_OVERRUN_FLAG ADC_CR1_OVRIE
#else
#define ADC_OVERRUN_FLAG 0
#endif

/* The CR1 bits and SR bits for ADC interrupts don't occur in the same
 * order (WTF, ST?), so we need to convert between them.
 *
 * Since getting the interrupt flags out of the SR during IRQ handling
 * is the hot path, adc_interrupt_id is defined in terms of ADC_SR
 * bits. This function converts those values to CR1 bits. */
static uint32 flags_to_cr1_bits(uint32 flags) {
    uint32 cr1_bits = 0;
    if (flags & ADC_CONV_INTERRUPT) {
        cr1_bits |= ADC_CR1_EOCIE;
    }
    if (flags & ADC_INJ_CONV_INTERRUPT) {
        cr1_bits |= ADC_CR1_JEOCIE;
    }
    if (flags & ADC_WATCHDOG_INTERRUPT) {
        cr1_bits |= ADC_CR1_AWDIE;
    }
    if (flags & ADC_OVERRUN_INTERRUPT) {
        cr1_bits |= ADC_OVERRUN_FLAG;
    }
    return cr1_bits;
}

/**
 * @brief Enable ADC interrupts
 * @param dev ADC device
 * @param interrupt_flags Logical or of adc_interrupt_id values to enable.
 * @see adc_disable_interrupt()
 * @see adc_attach_interrupt()
 */
void adc_enable_interrupts(const adc_dev *dev, uint32 interrupt_flags) {
    uint32 flags_to_set = flags_to_cr1_bits(interrupt_flags);
    uint32 cr1 = dev->regs->CR1;
    cr1 |= flags_to_set;
    dev->regs->CR1 = cr1;
    _adc_enable_dev_irq(dev);
}

/**
 * @brief Disable ADC interrupts.
 * @param dev ADC device
 * @param interrupt_flags Logical or of adc_interrupt_id values to disable.
 * @brief adc_enable_interrupt()
 */
void adc_disable_interrupts(const adc_dev *dev, uint32 interrupt_flags) {
    /* Don't use nvic_irq_disable()! IRQs are shared among ADCs. */
    uint32 flags_to_clear = flags_to_cr1_bits(interrupt_flags);
    uint32 cr1 = dev->regs->CR1;
    cr1 &= ~flags_to_clear;
    dev->regs->CR1 = cr1;
}
