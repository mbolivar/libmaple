// This example shows how to use the libmaple proper API for ADC
// interrupts.

#include <libmaple/adc.h>

#include "wirish.h"

// Pin number we're reading ADC data from; change it if you want.
const int adcPin = 0;

// -- ADC data and interrupt handler ---------------------------------

const adc_dev *dev; // ADC device for adcPin, set in setup()
uint8 channel;      // ADC channel for adcPin, set in setup()

volatile uint32 result = 0; // Most recently converted result
volatile bool gotResult = false; // (Nothing yet.)

void adcInterruptHandler(adc_callback_data *cb_data) {
    // Make sure we got called because conversion is over. Not
    // strictly necessary, but good practice.
    ASSERT(cb_data->irq_flags & ADC_CONV_INTERRUPT);

    // Store the converted result.
    gotResult = true;
    result = adc_get_data(dev);

    // Start the next conversion.
    adc_start_conv(dev);
}

// -- setup() and loop() ---------------------------------------------

void setup(void) {
    // Get the ADC device and channel for adcPin.
    dev = PIN_MAP[adcPin].adc_device;
    channel = PIN_MAP[adcPin].adc_channel;

    // Make sure adcPin can do ADC conversion.
    ASSERT(dev);

    // Set the pin mode, and configure dev for interrupt-based
    // conversion.
    pinMode(adcPin, INPUT_ANALOG);
    // We're only converting one channel.
    adc_set_reg_seq(dev, &channel, 1);
    // Set up our interrupt handler. Our handler doesn't need any
    // extra information, so we set the `arg' parameter to NULL.
    adc_attach_interrupt(dev, ADC_CONV_INTERRUPT, adcInterruptHandler, NULL);

    // Start the first conversion!
    adc_start_conv(dev);

    // Wait until the first result comes in before moving to loop().
    while (!gotResult)
        ;
}

void loop(void) {
    // Print the most recently converted result. This will keep
    // changing, since the interrupt handler starts a fresh conversion
    // each time.
    SerialUSB.print("Latest ADC data: ");
    SerialUSB.println(result);
}

// -- Boilerplate ----------------------------------------------------

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
