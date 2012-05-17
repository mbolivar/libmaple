/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2012 LeafLabs, LLC
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
 * @file libmaple/stm32f2/include/series/dma.h
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief STM32F2 DMA series header
 */

#ifndef _LIBMAPLE_STM32F2_DMA_H_
#define _LIBMAPLE_STM32F2_DMA_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <libmaple/dma_common.h>
#include <libmaple/libmaple_types.h>

/*
 * Register map and base pointers
 */

/**
 * @brief STM32F2 DMA register map type.
 */
typedef struct dma_reg_map {
    /* Isn't it nice how on F1, it's CCR1, but on F2, it's S1CR? */

    /* Global DMA registers */
    __io uint32 LISR;   /**< Low interrupt status register */
    __io uint32 HISR;   /**< High interrupt status register */
    __io uint32 LIFCR;  /**< Low interrupt flag clear register */
    __io uint32 HIFCR;  /**< High interrupt flag clear register */
    /* Stream 0 registers */
    __io uint32 S0CR;   /**< Stream 0 control register */
    __io uint32 S0NDTR; /**< Stream 0 number of data register */
    __io uint32 S0PAR;  /**< Stream 0 peripheral address register */
    __io uint32 S0M0AR; /**< Stream 0 memory 0 address register */
    __io uint32 S0M1AR; /**< Stream 0 memory 1 address register */
    __io uint32 S0FCR;  /**< Stream 0 FIFO control register */
    /* Stream 1 registers */
    __io uint32 S1CR;   /**< Stream 1 control register */
    __io uint32 S1NDTR; /**< Stream 1 number of data register */
    __io uint32 S1PAR;  /**< Stream 1 peripheral address register */
    __io uint32 S1M0AR; /**< Stream 1 memory 0 address register */
    __io uint32 S1M1AR; /**< Stream 1 memory 1 address register */
    __io uint32 S1FCR;  /**< Stream 1 FIFO control register */
    /* Stream 2 registers */
    __io uint32 S2CR;   /**< Stream 2 control register */
    __io uint32 S2NDTR; /**< Stream 2 number of data register */
    __io uint32 S2PAR;  /**< Stream 2 peripheral address register */
    __io uint32 S2M0AR; /**< Stream 2 memory 0 address register */
    __io uint32 S2M1AR; /**< Stream 2 memory 1 address register */
    __io uint32 S2FCR;  /**< Stream 2 FIFO control register */
    /* Stream 3 registers */
    __io uint32 S3CR;   /**< Stream 3 control register */
    __io uint32 S3NDTR; /**< Stream 3 number of data register */
    __io uint32 S3PAR;  /**< Stream 3 peripheral address register */
    __io uint32 S3M0AR; /**< Stream 3 memory 0 address register */
    __io uint32 S3M1AR; /**< Stream 3 memory 1 address register */
    __io uint32 S3FCR;  /**< Stream 3 FIFO control register */
    /* Stream 4 registers */
    __io uint32 S4CR;   /**< Stream 4 control register */
    __io uint32 S4NDTR; /**< Stream 4 number of data register */
    __io uint32 S4PAR;  /**< Stream 4 peripheral address register */
    __io uint32 S4M0AR; /**< Stream 4 memory 0 address register */
    __io uint32 S4M1AR; /**< Stream 4 memory 1 address register */
    __io uint32 S4FCR;  /**< Stream 4 FIFO control register */
    /* Stream 5 registers */
    __io uint32 S5CR;   /**< Stream 5 control register */
    __io uint32 S5NDTR; /**< Stream 5 number of data register */
    __io uint32 S5PAR;  /**< Stream 5 peripheral address register */
    __io uint32 S5M0AR; /**< Stream 5 memory 0 address register */
    __io uint32 S5M1AR; /**< Stream 5 memory 1 address register */
    __io uint32 S5FCR;  /**< Stream 5 FIFO control register */
    /* Stream 6 registers */
    __io uint32 S6CR;   /**< Stream 6 control register */
    __io uint32 S6NDTR; /**< Stream 6 number of data register */
    __io uint32 S6PAR;  /**< Stream 6 peripheral address register */
    __io uint32 S6M0AR; /**< Stream 6 memory 0 address register */
    __io uint32 S6M1AR; /**< Stream 6 memory 1 address register */
    __io uint32 S6FCR;  /**< Stream 6 FIFO control register */
    /* Stream 7 registers */
    __io uint32 S7CR;   /**< Stream 7 control register */
    __io uint32 S7NDTR; /**< Stream 7 number of data register */
    __io uint32 S7PAR;  /**< Stream 7 peripheral address register */
    __io uint32 S7M0AR; /**< Stream 7 memory 0 address register */
    __io uint32 S7M1AR; /**< Stream 7 memory 1 address register */
    __io uint32 S7FCR;  /**< Stream 7 FIFO control register */
} dma_reg_map;

/** DMA controller 1 register map base pointer */
#define DMA1_BASE                       ((struct dma_reg_map*)0x40026000)
/** DMA controller 2 register map base pointer */
#define DMA2_BASE                       ((struct dma_reg_map*)0x40026400)

/**
 * @brief STM32F2 DMA stream (i.e. tube) register map type.
 * Provides access to an individual stream's registers.
 * @see dma_tube_regs()
 */
typedef struct dma_tube_reg_map {
    __io uint32 SCR;   /**< Stream configuration register */
    __io uint32 SNDTR; /**< Stream number of data register */
    __io uint32 SPAR;  /**< Stream peripheral address register */
    __io uint32 SM0AR; /**< Stream memory 0 address register */
    __io uint32 SM1AR; /**< Stream memory 1 address register */
    __io uint32 SFCR;  /**< Stream FIFO control register */
} dma_tube_reg_map;

/** DMA1 stream 0 register map base pointer */
#define DMA1S0_BASE ((struct dma_tube_reg_map*)0x40026010)
/** DMA1 stream 1 register map base pointer */
#define DMA1S1_BASE ((struct dma_tube_reg_map*)0x40026028)
/** DMA1 stream 2 register map base pointer */
#define DMA1S2_BASE ((struct dma_tube_reg_map*)0x40026040)
/** DMA1 stream 3 register map base pointer */
#define DMA1S3_BASE ((struct dma_tube_reg_map*)0x40026058)
/** DMA1 stream 4 register map base pointer */
#define DMA1S4_BASE ((struct dma_tube_reg_map*)0x40026070)
/** DMA1 stream 5 register map base pointer */
#define DMA1S5_BASE ((struct dma_tube_reg_map*)0x40026088)
/** DMA1 stream 6 register map base pointer */
#define DMA1S6_BASE ((struct dma_tube_reg_map*)0x400260A0)
/** DMA1 stream 7 register map base pointer */
#define DMA1S7_BASE ((struct dma_tube_reg_map*)0x400260B8)

/** DMA2 stream 0 register map base pointer */
#define DMA2S0_BASE ((struct dma_tube_reg_map*)0x40026410)
/** DMA2 stream 1 register map base pointer */
#define DMA2S1_BASE ((struct dma_tube_reg_map*)0x40026028)
/** DMA2 stream 2 register map base pointer */
#define DMA2S2_BASE ((struct dma_tube_reg_map*)0x40026040)
/** DMA2 stream 3 register map base pointer */
#define DMA2S3_BASE ((struct dma_tube_reg_map*)0x40026058)
/** DMA2 stream 4 register map base pointer */
#define DMA2S4_BASE ((struct dma_tube_reg_map*)0x40026070)
/** DMA2 stream 5 register map base pointer */
#define DMA2S5_BASE ((struct dma_tube_reg_map*)0x40026088)
/** DMA2 stream 6 register map base pointer */
#define DMA2S6_BASE ((struct dma_tube_reg_map*)0x400260A0)
/** DMA2 stream 7 register map base pointer */
#define DMA2S7_BASE ((struct dma_tube_reg_map*)0x400260B8)

/*
 * Register bit definitions
 */

/* TODO */

/*
 * Devices
 */

extern dma_dev *DMA1;
extern dma_dev *DMA2;

/*
 * Other types needed by, or useful for, <libmaple/dma.h>
 */

/**
 * @brief DMA streams
 * This is also the dma_tube type for STM32F2.
 * @see dma_tube
 */
typedef enum dma_stream {
    DMA_S0 = 0,
    DMA_S1 = 1,
    DMA_S2 = 2,
    DMA_S3 = 3,
    DMA_S4 = 4,
    DMA_S5 = 5,
    DMA_S6 = 6,
    DMA_S7 = 7,
} dma_stream;

/** STM32F2 dma_tube (=dma_stream) */
#define dma_tube dma_stream

/**
 * @brief STM32F2 Mode flags for dma_tube_config.
 * @see dma_tube_config
 */
typedef enum dma_mode_flags {
    /* FIXME: MEM2MEM and DIR equivalents;
     * per->mem must be the default (for F1 compatibility). */

    DMA_MINC_MODE  = 1 << 10,   /**< Auto-increment memory address */
    DMA_PINC_MODE  = 1 << 9,    /**< Auto-increment peripheral address. */
    DMA_CIRC_MODE  = 1 << 8,    /**< Circular mode. */
    DMA_TRNS_CMPLT = 1 << 4,    /**< Interrupt on transfer completion. */
    DMA_HALF_TRNS  = 1 << 3,    /**< Interrupt on half-transfer. */
    DMA_TRNS_ERR   = 1 << 2,    /**< Interrupt on transfer error. */
    DMA_DIRECT_ERR = 1 << 1,    /**< Interrupt on direct mode error. */
    DMA_FIFO_ERR   = 1 << 31,   /**< Interrupt on FIFO error. */
} dma_mode_flags;

/**
 * @brief STM32F2 DMA request sources.
 *
 * IMPORTANT:
 *
 * 1. On STM32F2, a particular dma_request_src is always tied to a
 * single DMA controller, but often can be supported by multiple
 * streams. For example, DMA requests from ADC1 (DMA_REQ_SRC_ADC1) can
 * only be handled by DMA2, but they can go to either stream 0 or
 * stream 4 (though not any other stream). If you try to use a request
 * source with the wrong DMA controller or the wrong stream on
 * STM32F2, dma_tube_cfg() will fail.
 *
 * 2. A single stream can only handle a single request source at a
 * time. If you change a stream's request source later, it will stop
 * serving requests from the old source. However, for some streams,
 * some sources conflict with one another (when they correspond to the
 * same channel on that stream), and on STM32F2, Terrible Super-Bad
 * Things will happen if two conflicting request sources are active at
 * the same time.
 *
 * @see struct dma_tube_config
 * @see dma_tube_cfg()
 */
typedef enum dma_request_src {
    /* These are constructed like so (though this may change, so user
     * code shouldn't depend on it):
     *
     * Bits 0,1,2: Channel associated with request source
     *
     * Bits 3--9: rcc_clk_id of DMA controller associated with request source
     *
     * Bits 10--17: Bit mask of streams which can handle that request
     *              source.  (E.g., bit 10 set means stream 0 can
     *              handle the source, bit 11 set means stream 1 can,
     *              etc.)
     *
     * This is used for error checking in dma_tube_cfg().
     */
#define _DMA_STM32F2_REQ_SRC(stream_mask, clk_id, channel) \
    (((stream_mask) << 10) | ((clk_id) << 3) | (channel))
#define _DMA_S(n) (1U << (n))

    /* DMA1 request sources */
#define _DMA_1_REQ_SRC(stream_mask, channel) \
    _DMA_STM32F2_REQ_SRC(stream_mask, RCC_DMA1, channel)

    /* Channel 0 */
    DMA_REQ_SRC_SPI3_RX = _DMA_1_REQ_SRC(_DMA_S(0) | DMA_S(2), 0),
    DMA_REQ_SRC_SPI2_RX = _DMA_1_REQ_SRC(_DMA_S(3), 0),
    DMA_REQ_SRC_SPI2_TX = _DMA_1_REQ_SRC(_DMA_S(4), 0),
    DMA_REQ_SRC_SPI3_TX = _DMA_1_REQ_SRC(_DMA_S(5) | _DMA_S(7), 0),

    /* Channel 1 */
    DMA_REQ_SRC_I2C1_RX = _DMA_1_REQ_SRC(_DMA_S(0) | _DMA_S(5), 1),
    DMA_REQ_SRC_TIM7_UP = _DMA_1_REQ_SRC(_DMA_S(2) | _DMA_S(4), 1),
    DMA_REQ_SRC_I2C1_TX = _DMA_1_REQ_SRC(_DMA_S(6) | _DMA_S(7), 1),

    /* Channel 2 */
    DMA_REQ_SRC_TIM4_CH1 = _DMA_1_REQ_SRC(_DMA_S(0), 2),
    DMA_REQ_SRC_TIM4_CH2 = _DMA_1_REQ_SRC(_DMA_S(3), 2),
    DMA_REQ_SRC_TIM4_UP  = _DMA_1_REQ_SRC(_DMA_S(6), 2),
    DMA_REQ_SRC_TIM4_CH3 = _DMA_1_REQ_SRC(_DMA_S(7), 2),

    /* Channel 3 */
    DMA_REQ_SRC_TIM2_UP  = _DMA_1_REQ_SRC(_DMA_S(1) | _DMA_S(7), 3),
    DMA_REQ_SRC_TIM2_CH3 = _DMA_1_REQ_SRC(_DMA_S(1), 3),
    DMA_REQ_SRC_I2C3_RX  = _DMA_1_REQ_SRC(_DMA_S(2), 3),
    DMA_REQ_SRC_I2C3_TX  = _DMA_1_REQ_SRC(_DMA_S(4), 3),
    DMA_REQ_SRC_TIM2_CH1 = _DMA_1_REQ_SRC(_DMA_S(5), 3),
    DMA_REQ_SRC_TIM2_CH2 = _DMA_1_REQ_SRC(_DMA_S(6), 3),
    DMA_REQ_SRC_TIM2_CH4 = _DMA_1_REQ_SRC(_DMA_S(6) | _DMA_S(7), 3),

    /* Channel 4 */
    DMA_REQ_SRC_UART5_RX  = _DMA_1_REQ_SRC(_DMA_S(0), 4),
    DMA_REQ_SRC_USART3_RX = _DMA_1_REQ_SRC(_DMA_S(1), 4),
    DMA_REQ_SRC_UART4_RX  = _DMA_1_REQ_SRC(_DMA_S(2), 4),
    DMA_REQ_SRC_USART3_TX = _DMA_1_REQ_SRC(_DMA_S(3), 4),
    DMA_REQ_SRC_UART4_TX  = _DMA_1_REQ_SRC(_DMA_S(4), 4),
    DMA_REQ_SRC_USART2_RX = _DMA_1_REQ_SRC(_DMA_S(5), 4),
    DMA_REQ_SRC_USART2_TX = _DMA_1_REQ_SRC(_DMA_S(6), 4),
    DMA_REQ_SRC_UART5_TX  = _DMA_1_REQ_SRC(_DMA_S(7), 4),

    /* Channel 5 */
    DMA_REQ_SRC_TIM3_CH4  = _DMA_1_REQ_SRC(_DMA_S(2), 5),
    DMA_REQ_SRC_TIM3_UP   = _DMA_1_REQ_SRC(_DMA_S(2), 5),
    DMA_REQ_SRC_TIM3_CH1  = _DMA_1_REQ_SRC(_DMA_S(4), 5),
    DMA_REQ_SRC_TIM3_TRIG = _DMA_1_REQ_SRC(_DMA_S(4), 5),
    DMA_REQ_SRC_TIM3_CH2  = _DMA_1_REQ_SRC(_DMA_S(5), 5),
    DMA_REQ_SRC_TIM3_CH3  = _DMA_1_REQ_SRC(_DMA_S(7), 5),

    /* Channel 6 */
    DMA_REQ_SRC_TIM5_CH3  = _DMA_1_REQ_SRC(_DMA_S(0), 6),
    DMA_REQ_SRC_TIM5_UP   = _DMA_1_REQ_SRC(_DMA_S(0) | _DMA_S(6), 6),
    DMA_REQ_SRC_TIM5_CH4  = _DMA_1_REQ_SRC(_DMA_S(1) | _DMA_S(3), 6),
    DMA_REQ_SRC_TIM5_TRIG = _DMA_1_REQ_SRC(_DMA_S(1) | _DMA_S(3), 6),
    DMA_REQ_SRC_TIM5_CH1  = _DMA_1_REQ_SRC(_DMA_S(2), 6),
    DMA_REQ_SRC_TIM5_CH2  = _DMA_1_REQ_SRC(_DMA_S(4), 6),

    /* Channel 7 */
    DMA_REQ_SRC_TIM6_UP   = _DMA_1_REQ_SRC(_DMA_S(1), 7),
    DMA_REQ_SRC_I2C2_RX   = _DMA_1_REQ_SRC(_DMA_S(2) | _DMA_S(3), 7),
    DMA_REQ_SRC_USART3_TX = _DMA_1_REQ_SRC(_DMA_S(4), 7),
    DMA_REQ_SRC_DAC1      = _DMA_1_REQ_SRC(_DMA_S(5), 7),
    DMA_REQ_SRC_DAC2      = _DMA_1_REQ_SRC(_DMA_S(6), 7),
    DMA_REQ_SRC_I2C2_TX   = _DMA_1_REQ_SRC(_DMA_S(7), 7),
#undef _DMA_1_REQ_SRC

    /* DMA2 request sources */
#define _DMA_2_REQ_SRC(stream_mask, channel) \
    _DMA_STM32F2_REQ_SRC(stream_mask, RCC_DMA2, channel)

    /* Channel 0 */
    DMA_REQ_SRC_ADC1     = _DMA_2_REQ_SRC(_DMA_S(0) | _DMA_S(4), 0),
    /* You can use these "DMA_REQ_SRC_TIMx_CHx_ALTERNATE" if you know
     * what you're doing, but the other ones (for channels 6 and 7),
     * are better, in that they don't conflict with one another. */
    DMA_REQ_SRC_TIM8_CH1_ALTERNATE = _DMA_2_REQ_SRC(_DMA_S(2), 0),
    DMA_REQ_SRC_TIM8_CH2_ALTERNATE = _DMA_2_REQ_SRC(_DMA_S(2), 0),
    DMA_REQ_SRC_TIM8_CH3_ALTERNATE = _DMA_2_REQ_SRC(_DMA_S(2), 0),
    DMA_REQ_SRC_TIM1_CH1_ALTERNATE = _DMA_2_REQ_SRC(_DMA_S(6), 0),
    DMA_REQ_SRC_TIM1_CH2_ALTERNATE = _DMA_2_REQ_SRC(_DMA_S(6), 0),
    DMA_REQ_SRC_TIM1_CH3_ALTENRATE = _DMA_2_REQ_SRC(_DMA_S(6), 0),

    /* Channel 1 */
    DMA_REQ_SRC_DCMI = _DMA_2_REQ_SRC(_DMA_S(1) | _DMA_S(7), 1),
    DMA_REQ_SRC_ADC2 = _DMA_2_REQ_SRC(_DMA_S(2) | _DMA_S(3), 1),

    /* Channel 2 */
    DMA_REQ_SRC_ADC3     = _DMA_2_REQ_SRC(_DMA_S(0) | _DMA_S(1), 2),
    DMA_REQ_SRC_CRYP_OUT = _DMA_2_REQ_SRC(_DMA_S(5), 2),
    DMA_REQ_SRC_CRYP_IN  = _DMA_2_REQ_SRC(_DMA_S(6), 2),
    DMA_REQ_SRC_HASH_IN  = _DMA_2_REQ_SRC(_DMA_S(7), 2),

    /* Channel 3 */
    DMA_REQ_SRC_SPI1_RX = _DMA_2_REQ_SRC(_DMA_S(0) | _DMA_S(2), 3),
    DMA_REQ_SRC_SPI1_TX = _DMA_2_REQ_SRC(_DMA_S(3) | _DMA_S(5), 3),

    /* Channel 4 */
    DMA_REQ_SRC_USART1_RX = _DMA_2_REQ_SRC(_DMA_S(2) | _DMA_S(5), 4),
    DMA_REQ_SRC_SDIO      = _DMA_2_REQ_SRC(_DMA_S(3) | _DMA_S(6), 4),
    DMA_REQ_SRC_USART1_TX = _DMA_2_REQ_SRC(_DMA_S(7), 4),

    /* Channel 5 */
    DMA_REQ_SRC_USART6_RX = _DMA_2_REQ_SRC(_DMA_S(1) | _DMA_S(2), 5),
    DMA_REQ_SRC_USART6_TX = _DMA_2_REQ_SRC(_DMA_S(6) | _DMA_S(7), 5),

    /* Channel 6 */
    DMA_REQ_SRC_TIM1_TRIG = _DMA_2_REQ_SRC(_DMA_S(0) | _DMA_S(4), 6),
    DMA_REQ_SRC_TIM1_CH1  = _DMA_2_REQ_SRC(_DMA_S(1) | _DMA_S(3), 6),
    DMA_REQ_SRC_TIM1_CH2  = _DMA_2_REQ_SRC(_DMA_S(3), 6),
    DMA_REQ_SRC_TIM1_CH4  = _DMA_2_REQ_SRC(_DMA_S(4), 6),
    DMA_REQ_SRC_TIM1_COM  = _DMA_2_REQ_SRC(_DMA_S(4), 6),
    DMA_REQ_SRC_TIM1_UP   = _DMA_2_REQ_SRC(_DMA_S(5), 6),
    DMA_REQ_SRC_TIM1_CH3  = _DMA_2_REQ_SRC(_DMA_S(6), 6),

    /* Channel 7 */
    DMA_REQ_SRC_TIM8_UP   = _DMA_2_REQ_SRC(_DMA_S(1), 7),
    DMA_REQ_SRC_TIM8_CH1  = _DMA_2_REQ_SRC(_DMA_S(2), 7),
    DMA_REQ_SRC_TIM8_CH2  = _DMA_2_REQ_SRC(_DMA_S(3), 7),
    DMA_REQ_SRC_TIM8_CH3  = _DMA_2_REQ_SRC(_DMA_S(4), 7),
    DMA_REQ_SRC_TIM8_CH4  = _DMA_2_REQ_SRC(_DMA_S(7), 7),
    DMA_REQ_SRC_TIM8_TRIG = _DMA_2_REQ_SRC(_DMA_S(7), 7),
    DMA_REQ_SRC_TIM8_COM  = _DMA_2_REQ_SRC(_DMA_S(7), 7),
#undef _DMA_2_REQ_SRC
#undef _DMA_S
} dma_request_src;

/*
 * Tube conveniences
 */

static inline dma_tube_reg_map* dma_tube_regs(dma_dev *dev,
                                              dma_tube tube) {
    ASSERT(DMA_S0 <= tube && tube <= DMA_S7);
    switch (dev->clk_id) {
    case RCC_DMA1:
        return DMA1S0_BASE + (int)tube;
    case RCC_DMA2:
        return DMA2S0_BASE + (int)tube;
    default:
        /* Can't happen */
        ASSERT(0);
        return 0;
    }
}

static inline uint8 dma_is_enabled(dma_dev *dev, dma_tube tube) {
    return dma_tube_regs(dev, tube)->SCR & DMA_SCR_EN;
}

/* F2-only; available because of double-buffering. */
void dma_set_mem_n_addr(dma_dev *dev, dma_tube tube, int n,
                        __io void *address);

/**
 * @brief Set memory 0 address.
 * Availability: STM32F2.
 *
 * @param dev DMA device
 * @param tube Tube whose memory 0 address to set
 * @param addr Address to use as memory 0
 */
static __always_inline void
dma_set_mem0_addr(dma_dev *dev, dma_tube tube, __io void *addr) {
    dma_set_mem_n_addr(dev, tube, 0, addr);
}

/**
 * @brief Set memory 1 address.
 * Availability: STM32F2.
 *
 * @param dev DMA device
 * @param tube Tube whose memory 1 address to set
 * @param addr Address to use as memory 1
 */
static __always_inline void
dma_set_mem1_addr(dma_dev *dev, dma_tube tube, __io void *addr) {
    dma_set_mem_n_addr(dev, tube, 1, addr);
}

/* Assume the user means SM0AR in a non-double-buffered configuration. */
static __always_inline void
dma_set_mem_addr(dma_dev *dev, dma_tube tube, __io void *addr) {
    dma_set_mem0_addr(dev, tube, addr);
}

void dma_enable_fifo(dma_dev *dev, dma_tube tube);
void dma_disable_fifo(dma_dev *dev, dma_tube tube);

/*
 * TODO:
 * - Double-buffer configuration function
 * - FIFO configuration function
 * - MBURST/PBURST configuration function
 */

/*
 * ISR/IFCR conveniences.
 */

/* Helper macro for reading LISR/HISR and writing LIFCR/HIFCR. For
 * these registers,
 *
 * S0, S4:  bits start at bit 0
 * S1, S5:                    6
 * S2, S6:                   16
 * S3, S7:                   22
 *
 * I can't imagine why ST didn't just use a byte for each group. The
 * bits fit, and it would have made functions like these simpler and
 * faster.  Oh well. */
#define _DMA_IRQ_BIT_SHIFT(stream) ({ int _shifts[] = {0, 6, 16, 22};   \
                                      _shifts[(stream) & ~0x4]; })

static inline uint8 dma_get_isr_bits(dma_dev *dev, dma_tube tube) {
    dma_reg_map *regs = dev->regs;
    __io uint32 *isr = tube > DMA_S3 ? &regs->LISR : &regs->HISR;
    return (*isr >> _DMA_IRQ_BIT_SHIFT((int)tube)) & 0x3F;
}

static inline void dma_clear_isr_bits(dma_dev *dev, dma_tube tube) {
    dma_reg_map *regs = dev->regs;
    __io uint32 *isr = tube > DMA_S3 ? &regs->LIFCR : &regs->HIFCR;
    return *isr = 0x3F << _DMA_IRQ_BIT_SHIFT((int)tube);
}

#undef _DMA_IRQ_BIT_SHIFT

#ifdef __cplusplus
} // extern "C"
#endif

#endif
