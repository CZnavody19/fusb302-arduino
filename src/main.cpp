//
// USB Power Delivery Sink Using FUSB302B
// Copyright (c) 2020 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include <Arduino.h>

#include "pd_debug.h"
#include "pd_sink.h"

using namespace usb_pd;

mcu_hal usb_pd::hal;
static pd_sink power_sink;

static void sink_callback(callback_event event);

void setup() {
    // Initialize USB PD controller
    hal.init();
    power_sink.set_event_callback(sink_callback);
    power_sink.init();
}

void loop() {
    // Work in regular loop
    power_sink.poll();
}

// Called when the USB PD controller triggers an event
void sink_callback(callback_event event) {
    int voltage = 5000;

    switch (event) {
    case callback_event::source_caps_changed:
        DEBUG_LOG("Caps changed: %d\r\n", power_sink.num_source_caps);

        // Take maximum voltage
        for (int i = 0; i < power_sink.num_source_caps; i++) {
            if (power_sink.source_caps[i].voltage > voltage) {
                voltage = power_sink.source_caps[i].voltage;
            }
        }

        // Limit voltage to 20V as the voltage regulator was likely selected to handle 20V max
        if (voltage > 20000) {
            voltage = 20000;
        }

        power_sink.request_power(voltage);
        break;

    case callback_event::power_ready:
        DEBUG_LOG("Voltage: %d\r\n", power_sink.active_voltage);
        break;

    case callback_event::protocol_changed:
        break;

    default:
        break;
    }
}
