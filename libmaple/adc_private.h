/******************************************************************************
 * The MIT License
 *
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

#ifndef _LIBMAPLE_ADC_PRIVATE_H_
#define _LIBMAPLE_ADC_PRIVATE_H_

#include <libmaple/libmaple.h>
#include <libmaple/adc.h>

struct adc_private_data {
    void (*handler)(adc_callback_data*);
    uint32 handler_flags;
    adc_callback_data cb_data;
};

/* IRQ handler for adc_attach_interrupt() */
static __always_inline void adc_irq(const adc_dev *dev) {
    struct adc_private_data *priv = dev->priv;
    uint32 irq_flags = dev->regs->SR & priv->handler_flags;

    if (!irq_flags) {
        /* The user isn't interested in this IRQ. */
        return;
    } else if (priv->handler) {
        priv->cb_data.irq_flags = irq_flags;
        priv->handler(&priv->cb_data);
    }
}

extern void _adc_enable_dev_irq(const adc_dev *dev);

#endif  /* _LIBMAPLE_ADC_PRIVATE_H_ */
