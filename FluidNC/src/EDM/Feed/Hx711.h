// FluidNC/src/EDM/Feed/Hx711.h
//
// ESP32-COUPLED HARDWARE (NOT host-testable; NOT in the native test build_src_filter).
// Uses the FluidNC Pin class, delay_us() and a portMUX critical section; verified
// by the ESP32 firmware build + on-target bench (P4). The pure sign-extend and
// raw->Newtons math lives in Hx711Calibration.h and IS host-tested.
//
// Bit-bang HX711 24-bit load-cell ADC driver. Non-blocking: dataReady() polls
// DOUT (low == conversion ready) so the wire-feed task never busy-waits; the
// actual 24+gain clock burst runs inside a short critical section.
#pragma once

#include "Pin.h"
#include "ITensionSensor.h"
#include "Hx711Calibration.h"
#include "Configuration/Configurable.h"

#include <cstdint>

namespace EDM { namespace feed {

class Hx711 : public ITensionSensor, public Configuration::Configurable {
public:
    Hx711() = default;

    // Configure DOUT (input) and SCK (output, idle low). Call once after parse.
    void begin();

    // ITensionSensor: DOUT low means a sample is ready (non-blocking poll).
    bool    dataReady() const override { return !_dout.read(); }
    // Clock out one 24-bit sample (MSB first) + gain-select pulses; sign-extend.
    int32_t readRaw() override;

    // Average n raw samples and adopt the result as the new tare offset.
    void tare(int n = 16);

    float                   toNewtons(int32_t raw) const { return _cal.toNewtons(raw); }
    void                    setCalibration(int32_t off, float cpn) { _cal = _cal.withCalibration(off, cpn); }
    const Hx711Calibration& cal() const { return _cal; }

    // Configuration::Configurable
    void group(Configuration::HandlerBase& handler) override;
    void afterParse() override;
    void validate() override;

private:
    Pin              _dout;
    Pin              _sck;
    int              _gain_pulses = 1;  // 128 gain -> 1 extra pulse (see afterParse)
    int              _channel_gain = 128;
    Hx711Calibration _cal;
};

}}  // namespace EDM::feed
