//
// USB Power Delivery Sink Using FUSB302B
// Copyright (c) 2020 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//
// Hardware abstraction layer
//

#include <Arduino.h>
#include <Wire.h>
#include "hal.h"

#include "pd_debug.h"

namespace usb_pd {

constexpr uint16_t fusb302_int_n_pin = 12;
constexpr uint8_t fusb302_i2c_addr = 0x22;

void mcu_hal::init() {
    DEBUG_INIT();

    Wire.begin();
}

void mcu_hal::pd_ctrl_read(uint8_t reg, int data_len, uint8_t* data) {
    Wire.beginTransmission(fusb302_i2c_addr);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(fusb302_i2c_addr, data_len);

    int i = 0;
    while (Wire.available() && i < data_len) {
        data[i] = Wire.read();
        i++;
    }

    if (i != data_len)
        DEBUG_LOG("NACK read %d\r\n", reg);
}

void mcu_hal::pd_ctrl_write(uint8_t reg, int data_len, const uint8_t* data, bool end_with_stop) {
    Wire.beginTransmission(fusb302_i2c_addr);
    Wire.write(reg);

    for (int i = 0; i < data_len; i++) {
        Wire.write(data[i]);
    }

    if (end_with_stop) {
        Wire.endTransmission();
    } else {
        // Send a repeated start if not ending with stop
        Wire.endTransmission(false);
    }

    if (Wire.endTransmission() != 0)
        DEBUG_LOG("NACK write %d\r\n", reg);
}

uint32_t mcu_hal::millis() {
    return ::millis();
}

void mcu_hal::delay(uint32_t ms) {
    ::delay(ms);
}

bool mcu_hal::has_expired(uint32_t timeout) {
    return (int32_t)timeout - (int32_t)::millis() <= 0;
}

} // namespace usb_pd
