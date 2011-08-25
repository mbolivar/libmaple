/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs, LLC.
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
 * @file SPI.h
 * @brief Arduino-style SPI library
 */

#ifndef _LIB_SPI_H_
#define _LIB_SPI_H_

#include "wirish.h"
#include "HardwareSPI.h"

#define SPI_MODE0                       SPI_MODE_0
#define SPI_MODE1                       SPI_MODE_1
#define SPI_MODE2                       SPI_MODE_2
#define SPI_MODE3                       SPI_MODE_3

#define SPI_CLOCK_DIV2                  SPI_BAUD_PCLK_DIV_2
#define SPI_CLOCK_DIV4                  SPI_BAUD_PCLK_DIV_4
#define SPI_CLOCK_DIV8                  SPI_BAUD_PCLK_DIV_8
#define SPI_CLOCK_DIV16                 SPI_BAUD_PCLK_DIV_16
#define SPI_CLOCK_DIV32                 SPI_BAUD_PCLK_DIV_32
#define SPI_CLOCK_DIV64                 SPI_BAUD_PCLK_DIV_64
#define SPI_CLOCK_DIV128                SPI_BAUD_PCLK_DIV_128
#define SPI_CLOCK_DIV256                SPI_BAUD_PCLK_DIV_256

class SPIClass : public HardwareSPI {
public:
    // DO NOT INSTANTIATE THIS YOURSELF!
    SPIClass();

    void begin(void);

    // Don't use these while communication is ongoing
    void setBitOrder(uint8);
    void setDataMode(uint8);
    void setClockDivider(uint32 divider);

    using HardwareSPI::transfer;
    using HardwareSPI::end;
private:
    SPIClass(const SPIClass&);
    SPIClass& operator=(const SPIClass&);
};

extern SPIClass SPI;

#endif
