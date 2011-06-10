/*
 * Copyright (C) 2011 by James Bowman <jamesb@excamera.com>
 * Gameduino library for arduino.
 *
 */

#ifndef _GD_H_INCLUDED
#define _GD_H_INCLUDED

// define SS_PIN before including "GD.h" to override this
#ifndef SS_PIN
#define SS_PIN 9
#endif

#ifdef BOARD_maple

#include "wirish.h"

typedef const unsigned char prog_uchar;
#define Serial SerialUSB
#define pgm_read_byte_near(x) (*(prog_uchar*)x)
#define pgm_read_byte(x) (*(prog_uchar*)x)
#define PROGMEM const

extern HardwareSPI SPI;

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#endif

class GDClass {
public:
  static void begin();
  static void end();
  static void __start(unsigned int addr);
  static void __wstart(unsigned int addr);
  static void __end(void);
  static byte rd(unsigned int addr);
  static void wr(unsigned int addr, byte v);
  static unsigned int rd16(unsigned int addr);
  static void wr16(unsigned int addr, unsigned int v);
  static void fill(int addr, byte v, unsigned int count);
  static void copy(unsigned int addr, prog_uchar *src, int count);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  static void copy(unsigned int addr, uint_farptr_t src, int count);
  static void microcode(uint_farptr_t src, int count);
  static void uncompress(unsigned int addr, uint_farptr_t src);
#endif

  static void setpal(int pal, unsigned int rgb);
  static void sprite(int spr, int x, int y, byte image, byte palette, byte rot = 0, byte jk = 0);
  static void sprite2x2(int spr, int x, int y, byte image, byte palette, byte rot = 0, byte jk = 0);
  static void waitvblank();
  static void microcode(prog_uchar *src, int count);
  static void uncompress(unsigned int addr, prog_uchar *src);

  static void voice(int v, byte wave, unsigned int freq, byte lamp, byte ramp);
  static void ascii();
  static void putstr(int x, int y, const char *s);

  static void screenshot(unsigned int frame);

  void __wstartspr(unsigned int spr = 0);
  void xsprite(int ox, int oy, char x, char y, byte image, byte palette, byte rot = 0, byte jk = 0);
  void xhide();

  byte spr;   // Current sprite, incremented by xsprite/xhide above
};

extern GDClass GD;

#define RGB(r,g,b) ((((r) >> 3) << 10) | (((g) >> 3) << 5) | ((b) >> 3))
#define TRANSPARENT (1 << 15) // transparent for chars and sprites

#define RAM_PIC     0x0000    // Screen Picture, 64 x 64 = 4096 bytes
#define RAM_CHR     0x1000    // Screen Characters, 256 x 16 = 4096 bytes
#define RAM_PAL     0x2000    // Screen Character Palette, 256 x 8 = 2048 bytes

#define IDENT         0x2800
#define REV           0x2801
#define FRAME         0x2802
#define VBLANK        0x2803
#define SCROLL_X      0x2804
#define SCROLL_Y      0x2806
#define JK_MODE       0x2808
#define J1_RESET      0x2809
#define SPR_DISABLE   0x280a
#define SPR_PAGE      0x280b
#define IOMODE        0x280c

#define BG_COLOR      0x280e
#define SAMPLE_L      0x2810
#define SAMPLE_R      0x2812

#define MODULATOR     0x2814

#define SCREENSHOT_Y  0x281e

#define PALETTE16A 0x2840   // 16-color palette RAM A, 32 bytes
#define PALETTE16B 0x2860   // 16-color palette RAM B, 32 bytes
#define PALETTE4A  0x2880   // 4-color palette RAM A, 8 bytes
#define PALETTE4B  0x2888   // 4-color palette RAM A, 8 bytes
#define COMM       0x2890   // Communication buffer
#define COLLISION  0x2900   // Collision detection RAM, 256 bytes
#define VOICES     0x2a00   // Voice controls
#define J1_CODE    0x2b00   // J1 coprocessor microcode RAM
#define SCREENSHOT 0x2c00   // screenshot line RAM

#define RAM_SPR     0x3000    // Sprite Control, 512 x 4 = 2048 bytes
#define RAM_SPRPAL  0x3800    // Sprite Palettes, 4 x 256 = 2048 bytes
#define RAM_SPRIMG  0x4000    // Sprite Image, 64 x 256 = 16384 bytes

#ifndef GET_FAR_ADDRESS // at some point this will become official... https://savannah.nongnu.org/patch/?6352
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define GET_FAR_ADDRESS(var)                          \
({                                                    \
    uint_farptr_t tmp;                                \
                                                      \
    __asm__ __volatile__(                             \
                                                      \
            "ldi    %A0, lo8(%1)"           "\n\t"    \
            "ldi    %B0, hi8(%1)"           "\n\t"    \
            "ldi    %C0, hh8(%1)"           "\n\t"    \
            "clr    %D0"                    "\n\t"    \
        :                                             \
            "=d" (tmp)                                \
        :                                             \
            "p"  (&(var))                             \
    );                                                \
    tmp;                                              \
})
#else
#define GET_FAR_ADDRESS(var) (var)
#endif
#endif

#endif

