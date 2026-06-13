// FluidNC/src/EDM/Feed/Hx711.cpp
//
// ESP32-COUPLED HARDWARE (NOT host-testable; NOT in the native test build_src_filter).
// Verified by the ESP32 firmware build + on-target bench (P4).

#include "Hx711.h"

#include "Driver/delay_usecs.h"  // delay_us(int32_t)
#include "Assertion.h"           // Assert()

#include <freertos/FreeRTOS.h>   // portMUX_TYPE, portENTER/EXIT_CRITICAL, portMUX_INITIALIZER_UNLOCKED

namespace EDM { namespace feed {

// One spinlock guarding the bit-bang clock burst. The whole burst is ~50us
// (26 * 2us) so the critical section is kept well under 60us; there is a single
// HX711 in the machine, so a file-static lock is sufficient.
static portMUX_TYPE s_hx711_mux = portMUX_INITIALIZER_UNLOCKED;

void Hx711::begin() {
    // DOUT: input with pull-up so a disconnected/idle line reads high (not ready)
    // rather than floating low and faking a perpetual data-ready.
    _dout.setAttr(Pin::Attr::Input | Pin::Attr::PullUp);
    _sck.setAttr(Pin::Attr::Output);
    _sck.synchronousWrite(false);  // SCK idle low
}

int32_t Hx711::readRaw() {
    uint32_t v = 0;

    portENTER_CRITICAL(&s_hx711_mux);
    // 24 data bits, MSB first.
    for (int i = 0; i < 24; ++i) {
        _sck.synchronousWrite(true);
        delay_us(1);
        v = (v << 1) | (_dout.read() ? 1u : 0u);
        _sck.synchronousWrite(false);
        delay_us(1);
    }
    // Extra pulses select the next conversion's channel/gain.
    for (int i = 0; i < _gain_pulses; ++i) {
        _sck.synchronousWrite(true);
        delay_us(1);
        _sck.synchronousWrite(false);
        delay_us(1);
    }
    portEXIT_CRITICAL(&s_hx711_mux);

    return hx711SignExtend(v);
}

void Hx711::tare(int n) {
    if (n < 1) {
        n = 1;
    }
    int64_t sum = 0;
    for (int i = 0; i < n; ++i) {
        sum += readRaw();
    }
    int32_t avg = int32_t(sum / n);
    _cal        = _cal.withCalibration(avg, _cal.counts_per_N);
}

void Hx711::group(Configuration::HandlerBase& handler) {
    handler.item("dout_pin", _dout);
    handler.item("sck_pin", _sck);
    handler.item("tare_offset", _cal.offset);
    handler.item("scale_counts_per_N", _cal.counts_per_N);
    handler.item("channel_gain", _channel_gain);
}

void Hx711::afterParse() {
    // Map the HX711 channel/gain selection to the number of extra SCK pulses
    // appended after the 24 data bits:
    //   gain 128 (chan A) -> 1 pulse, gain 32 (chan B) -> 2, gain 64 (chan A) -> 3.
    switch (_channel_gain) {
        case 64:
            _gain_pulses = 3;
            break;
        case 32:
            _gain_pulses = 2;
            break;
        case 128:
        default:
            _gain_pulses = 1;
            break;
    }
}

void Hx711::validate() {
    Assert(_dout.defined(), "HX711 dout_pin must be configured");
    Assert(_sck.defined(), "HX711 sck_pin must be configured");
    Assert(_channel_gain == 128 || _channel_gain == 64 || _channel_gain == 32,
           "HX711 channel_gain must be 128, 64 or 32");
}

}}  // namespace EDM::feed
