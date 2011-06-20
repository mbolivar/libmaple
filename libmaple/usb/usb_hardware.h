/* *****************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ****************************************************************************/

#include "rcc.h"
#include "usb_type.h"

#ifndef _USB_HARDWARE_H_
#define _USB_HARDWARE_H_

/* macro'd register and peripheral definitions */
#define EXC_RETURN 0xFFFFFFF9
#define DEFAULT_CPSR 0x61000000

#define FLASH ((u32)0x40022000)

#define USB_PACKET_BUFFER ((u32)0x40006000)

#define SCS      0xE000E000
#define SCB      (SCS+0xD00)
#define STK      (SCS+0x10)

#define SCB_VTOR (SCB+0x08)
#define STK_CTRL (STK+0x00)

#define USB_HP_IRQ  ((u8)0x13)
#define USB_LP_IRQ  ((u8)0x14)

/* AIRCR  */
#define AIRCR_RESET         0x05FA0000
#define AIRCR_RESET_REQ     (AIRCR_RESET | (u32)0x04);

/* temporary copyage of example from kiel */
#define __VAL(__TIMCLK, __PERIOD) ((__TIMCLK/1000000UL)*__PERIOD)
#define __PSC(__TIMCLK, __PERIOD)  (((__VAL(__TIMCLK, __PERIOD)+49999UL)/50000UL) - 1)
#define __ARR(__TIMCLK, __PERIOD) ((__VAL(__TIMCLK, __PERIOD)/(__PSC(__TIMCLK, __PERIOD)+1)) - 1)

#define SET_REG(addr,val)  do { *(vu32*)(addr)=val; } while (0)
#define GET_REG(addr)      do { *(vu32*)(addr); } while (0)

#if defined(__cplusplus)
extern "C" {
#endif

/* todo: there must be some major misunderstanding in how we access regs. The direct access approach (GET_REG)
   causes the usb init to fail upon trying to activate RCC_APB1 |= 0x00800000. However, using the struct approach
   from ST, it works fine...temporarily switching to that approach */
typedef struct
{
  vu32 CR;
  vu32 CFGR;
  vu32 CIR;
  vu32 APB2RSTR;
  vu32 APB1RSTR;
  vu32 AHBENR;
  vu32 APB2ENR;
  vu32 APB1ENR;
  vu32 BDCR;
  vu32 CSR;
} RCC_RegStruct;
#define pRCC ((RCC_RegStruct *) RCC_BASE)

#if defined(__cplusplus)
}
#endif

#endif
