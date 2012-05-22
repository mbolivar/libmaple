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

/**
 * @file libmaple/stm32f2/dma.c
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief STM32F2 DMA support.
 */

#include <libmaple/dma.h>
#include <libmaple/util.h>

/*
 * Devices
 */

static dma_dev dma1 = {
    .regs = DMA1_BASE,
    .clk_id = RCC_DMA1,
    .handlers = {{ .handler = NULL, .irq_line = NVIC_DMA1_STREAM0 },
                 { .handler = NULL, .irq_line = NVIC_DMA1_STREAM1 },
                 { .handler = NULL, .irq_line = NVIC_DMA1_STREAM2 },
                 { .handler = NULL, .irq_line = NVIC_DMA1_STREAM3 },
                 { .handler = NULL, .irq_line = NVIC_DMA1_STREAM4 },
                 { .handler = NULL, .irq_line = NVIC_DMA1_STREAM5 },
                 { .handler = NULL, .irq_line = NVIC_DMA1_STREAM6 },
                 { .handler = NULL, .irq_line = NVIC_DMA1_STREAM7 }},
};
/** STM32F2 DMA1 device */
dma_dev *DMA1 = &dma1;

static dma_dev dma2 = {
    .regs = DMA2_BASE,
    .clk_id = RCC_DMA2,
    .handlers = {{ .handler = NULL, .irq_line = NVIC_DMA2_STREAM0 },
                 { .handler = NULL, .irq_line = NVIC_DMA2_STREAM1 },
                 { .handler = NULL, .irq_line = NVIC_DMA2_STREAM2 },
                 { .handler = NULL, .irq_line = NVIC_DMA2_STREAM3 },
                 { .handler = NULL, .irq_line = NVIC_DMA2_STREAM4 },
                 { .handler = NULL, .irq_line = NVIC_DMA2_STREAM5 },
                 { .handler = NULL, .irq_line = NVIC_DMA2_STREAM6 },
                 { .handler = NULL, .irq_line = NVIC_DMA2_STREAM7 }},
};
/** STM32F2 DMA2 device */
dma_dev *DMA2 = &dma2;

/*
 * Routines
 */

#define ASSERT_NOT_ENABLED(dev, tube) ASSERT(!dma_is_enabled(dev, tube))

/* See the comments in the F2 dma_request_src for how this works. */
static int tube_is_supported_by(dma_dev *dev, dma_tube stream,
                                dma_request_src req_src) {
    uint32 src = req_src;
    rcc_clk_id src_dev_clk = (rcc_clk_id)GET_BITS(src, 9, 3);
    uint32 streams_supporting_req = GET_BITS(src, 17, 10);
    return (src_dev_clk == dev->clk_id) && (streams_supporting_req & stream);
}

int dma_tube_cfg(dma_dev *dev, dma_tube tube, dma_tube_config *cfg) {
    dma_tube_reg_map *tregs;
    uint32 msize, psize;
    uint32 req_chsel_bits;

    /* Check well-formedness of `cfg'. */
    ASSERT(cfg->series_data == 0);
    ASSERT(0 <= cfg->tube_nr_xfers && cfg->tube_nr_xfers <= 65535);

    /* Ensure `tube' can satisfy `cfg'. */
    if (!tube_is_compatible(dev, tube, cfg->tube_req_src)) {
        return DMA_TUBE_CFG_ECOMPAT;
    }

    /* Ensure PSIZE, MSIZE, and NDT will be consistent.
     *
     * RM0033 specifies that PSIZE, MSIZE, and NDT must be such that
     * the last transfer completes; i.e. that if PSIZE < MSIZE, then
     * NDT is a multiple of MSIZE/PSIZE.  See e.g. Table 27. */
    msize = (1 << (uint32)cfg->tube_mem_size);
    psize = (1 << (uint32)cfg->tube_per_size);
    if ((psize < msize) && (cfg->tube_nr_xfers % (msize / psize))) {
        return DMA_TUBE_CFG_ENDATA;
    }

    /* Everything seems fine; disable `tube' according to RM0033
     * procedure. */
    dma_disable(dev, tube);
    dma_clear_isr_bits(dev, tube);

    /* Reconfigure `tube'. */
    req_chsel_bits = cfg->tube_req_src & 0x7;
    tregs = dma_tube_reg_map(dev, tube);
    tregs->SCR = ((req_chsel_bits << 25) |
                  (cfg->tube_mem_size << 13) |
                  (cfg->tube_per_size << 11) |
                  (cfg->tube_mode & ~DMA_FIFO_ERR));
    tregs->SNDTR = cfg->tube_nr_xfers;
    tregs->SM0AR = (uint32)cfg->tube_mem_addr;
    tregs->SPAR = (uint32)cfg->tube_per_addr;
    /* Provide a "good-enough" FIFO configuration.  ST really wants us
     * to use the FIFO (e.g., direct mode won't work unless MSIZE ==
     * PSIZE), so just do something reasonable by default.  Callers
     * can adjust this before enabling the transfer if they want
     * something else. */
    tregs->SFCR = ((cfg->tube_mode & DMA_FIFO_ERR) |
                   DMA_SFCR_DMDIS |
                   DMA_SCFR_FTH_FULL);

    return DMA_TUBE_CFG_SUCCESS;
}

void dma_set_priority(dma_dev *dev, dma_stream stream, dma_priority priority) {
    dma_tube_reg_map *tregs = dma_tube_regs(dev, tube);
    uint32 scr;
    ASSERT_NOT_ENABLED(dev, tube);
    scr = tregs->SCR;
    scr &= ~DMA_SCR_PL;
    scr |= (priority << 16);
    tregs->SCR = scr;
}

void dma_set_num_transfers(dma_dev *dev, dma_tube tube, uint16 num_transfers) {
    dma_tube_reg_map *tregs = dma_tube_regs(dev, tube);
    ASSERT_NOT_ENABLED(dev, tube);
    tregs->SNDTR = num_transfers;
}

/**
 * @brief Set memory 0 or memory 1 address.
 *
 * This is a general function for setting one of the two memory
 * addresses available on the double-buffered STM32F2 DMA controllers.
 *
 * @param dev     DMA device
 * @param tube    Tube on dev.
 * @param n       If 0, set memory 0 address. If 1, set memory 1 address.
 * @param address Address to set
 */
void dma_set_mem_n_addr(dma_dev *dev, dma_tube tube, int n,
                        __io void *address) {
    dma_tube_reg_map *tregs = dma_tube_regs(dev, tube);
    uint32 addr = (uint32)address;

    ASSERT_NOT_ENABLED(dev, tube);
    if (n) {
        tregs->SM0AR = addr;
    } else {
        tregs->SM1AR = addr;
    }
}

void dma_set_per_addr(dma_dev *dev, dma_tube tube, __io void *address) {
    dma_tube_reg_map *tregs = dma_tube_regs(dev, tube);
    ASSERT_NOT_ENABLED(dev, tube);
    tregs->SPAR = (uint32)address;
}

/**
 * @brief Enable a stream's FIFO.
 *
 * You may only call this function when the stream is disabled.
 *
 * @param dev  DMA device
 * @param tube Stream whose FIFO to enable.
 */
void dma_enable_fifo(dma_dev *dev, dma_tube tube) {
    ASSERT_NOT_ENABLED(dev, tube);
    bb_peri_set_bit(&(dma_tube_regs(dev, tube)->SFCR), DMA_SFCR_DMDIS_BIT, 1);
}

/**
 * @brief Disable a stream's FIFO.
 *
 * You may only call this function when the stream is disabled.
 *
 * @param dev  DMA device
 * @param tube Stream whose FIFO to disable.
 */
void dma_disable_fifo(dma_dev *dev, dma_tube tube) {
    ASSERT_NOT_ENABLED(dev, tube);
    bb_peri_set_bit(&(dma_tube_regs(dev, tube)->SFCR), DMA_SFCR_DMDIS_BIT, 0);
}

void dma_attach_interrupt(dma_dev *dev, dma_tube tube,
                          void (*handler)(void)) {
    dev->handlers[tube].handler = handler;
    nvic_irq_enable(dev->handlers[tube].irq_line);
}

void dma_detach_interrupt(dma_dev *dev, dma_tube tube) {
    nvic_irq_disable(dev->handlers[tube].irq_line);
    dev->handlers[tube].handler = NULL;
}

void dma_enable(dma_dev *dev, dma_tube tube) {
    dma_tube_reg_map *tregs = dma_tube_regs(dev, tube);
    bb_peri_set_bit(&tregs->SCR, DMA_SCR_EN_BIT, 1);
}

void dma_disable(dma_dev *dev, dma_tube tube) {
    dma_tube_reg_map *tregs = dma_tube_regs(dev, tube);
    bb_peri_set_bit(&tregs->SCR, DMA_SCR_EN_BIT, 0);
    /* The stream might not get disabled immediately, so wait. */
    while (tregs->SCR & DMA_SCR_EN)
        ;
}

dma_irq_cause dma_get_irq_cause(dma_dev *dev, dma_tube tube) {
    /* FIXME stub */
    return DMA_TRANSFER_ERROR;
}
