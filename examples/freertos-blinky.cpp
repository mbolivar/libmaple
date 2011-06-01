#include "wirish.h"
#include "libraries/FreeRTOS/MapleFreeRTOS.h"

static void vLEDFlashTask( void *pvParameters ) {
    for(;;) {
        vTaskDelay(2000);
        digitalWrite(BOARD_LED_PIN, HIGH);
        vTaskDelay(200);
        digitalWrite(BOARD_LED_PIN, LOW);
    }
}

void setup() {
    // initialize the digital pin as an output:
    pinMode(BOARD_LED_PIN, OUTPUT);

    xTaskCreate(vLEDFlashTask,
                "LEDx",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
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
