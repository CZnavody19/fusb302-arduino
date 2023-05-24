//
// USB Power Delivery Sink Using FUSB302B
// Copyright (c) 2020 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include "pd_debug.h"
#include "pd_sink.h"

#include <algorithm>

using namespace usb_pd;

mcu_hal usb_pd::hal;

static pd_sink power_sink;

static uint8_t selected_capability = 0;

// Modes:
// 0: voltage selectable by button
// 100: maximum voltage
// others: voltage in V
static uint16_t desired_mode = 100;

static void sink_callback(callback_event event);
static void on_source_caps_changed();

int main() {
    hal.init();

    DEBUG_LOG("Saved mode: %d\r\n", desired_mode);

    power_sink.set_event_callback(sink_callback);
    power_sink.init();

    // Work in regular loop
    while (true) {
        hal.poll();
        power_sink.poll();
    }
}

// Called when the USB PD controller triggers an event
void sink_callback(callback_event event) {
#if defined(PD_DEBUG)
    int index = static_cast<int>(event);
    const char* const event_names[] = {"protocol_changed", "source_caps_changed", "power_accepted", "power_rejected",
                                       "power_ready"};

    DEBUG_LOG("Event: ", 0);
    DEBUG_LOG(event_names[index], 0);
    DEBUG_LOG("\r\n", 0);
#endif

    switch (event) {
    case callback_event::source_caps_changed:
        DEBUG_LOG("Caps changed: %d\r\n", power_sink.num_source_caps);
        on_source_caps_changed();
        break;

    case callback_event::power_ready:
        DEBUG_LOG("Voltage: %d\r\n", power_sink.active_voltage);
        break;

    case callback_event::protocol_changed:
        if (power_sink.protocol() == pd_protocol::usb_20)
            selected_capability = 0;
        break;

    default:
        break;
    }
}

// Called when the source advertises new capabilities
// Be careful with debug output. If one of the capbilities is not
// requested in time, the power suplly will reset.
void on_source_caps_changed() {
    int voltage = 5000;

    // Take maximum voltage
    for (int i = 0; i < power_sink.num_source_caps; i++)
        voltage = std::max(voltage, static_cast<int>(power_sink.source_caps[i].voltage));

    // Limit voltage to 20V as the voltage regulator was likely selected to handle 20V max
    if (voltage > 20000)
        voltage = 20000;

    power_sink.request_power(voltage);
}

static const uint16_t voltages[] = {0, 9, 12, 15, 20, 100};
