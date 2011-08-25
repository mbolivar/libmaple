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

#include "SPI.h"

// The Arduino Uno sends the SPI peripheral onto the same pins as
// the Maple's SPI1, so this choice is for compatibility
#define SPI_NUM 1
#define SPI_DEV SPI1

#define MODE_MASK (SPI_CR1_CPOL | SPI_CR1_CPHA)

SPIClass SPI;

static void cr1_reconf(uint32 mask, uint32 val);

SPIClass::SPIClass()
    : HardwareSPI(SPI_NUM) {
    // Turn on the clock, or a call to e.g. setBitOrder() before a
    // call to begin() won't work as expected.
    spi_init(SPI_DEV);
}

void SPIClass::begin(void) {
    // TODO should we match the Arduino clock divider?
    HardwareSPI::begin();
    // Arduino expects manual NSS toggling with OUTPUT mode
    pinMode(this->nssPin(), OUTPUT);
    digitalWrite(this->nssPin(), HIGH);
}

void SPIClass::setBitOrder(uint8 bitOrder) {
    cr1_reconf(SPI_CR1_LSBFIRST,
                    bitOrder == LSBFIRST ? SPI_CR1_LSBFIRST : 0);
}

void SPIClass::setDataMode(uint8 mode) {
    cr1_reconf(MODE_MASK, (uint32)mode & MODE_MASK);
}

void SPIClass::setClockDivider(uint32 divider) {
    cr1_reconf(SPI_CR1_BR, divider & SPI_CR1_BR);
}

static void cr1_reconf(uint32 mask, uint32 val) {
    uint32 cr1 = SPI_DEV->regs->CR1;
    cr1 &= ~mask;
    cr1 |= val;
    SPI_DEV->regs->CR1 = cr1;
}
