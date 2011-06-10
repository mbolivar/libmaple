#include "string.h"
#include "wirish.h"
#include "GD.h"

#define NSS 9

HardwareSPI spi1(1);

void setup() {
   pinMode(NSS, OUTPUT);
   digitalWrite(NSS, HIGH);
   spi1.begin(SPI_4_5MHZ, MSBFIRST, 0);
}

void loop() {
   digitalWrite(NSS, LOW);
   spi1.send(0x80);  // write to address 0
   spi1.send(0x00);
   char s[80];
   sprintf(s, "Good evening Maple %8d", millis());
   spi1.send((uint8*)s, strlen(s));
   digitalWrite(NSS, HIGH);
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
