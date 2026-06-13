// FluidNC/src/EDM/Can/Mcp2518fdDriver.h
#pragma once
#include <cstdint>
#include "EDM/Can/CanBus.h"

namespace EDM {

// CanBus implementation for a Microchip MCP2518FD over SPI.
// Bit timing fixed to the PSU contract: 1 Mbps arbitration, 5 Mbps data.
class Mcp2518fdDriver : public CanBus {
public:
    struct Pins { int sck, miso, mosi, cs, intr; };
    struct Config {
        Pins     pins;
        uint8_t  spi_host    = 1;          // HSPI/VSPI selection
        uint32_t spi_hz      = 17000000;   // up to 20 MHz
        uint32_t osc_hz      = 40000000;   // external crystal on the module
    };

    explicit Mcp2518fdDriver(const Config& cfg) : _cfg(cfg) {}

    bool init() override;
    bool send(const CanFrame& f) override;
    void onReceive(RxHandler h) override { _rx = std::move(h); }
    void poll() override;   // drain RX FIFO; call from CAN-RX task

private:
    Config    _cfg;
    RxHandler _rx;

    // SPI primitives
    void     reset();
    uint32_t readReg(uint16_t addr);
    void     writeReg(uint16_t addr, uint32_t val);
    void     readRam(uint16_t addr, uint8_t* buf, uint8_t n);
    void     writeRam(uint16_t addr, const uint8_t* buf, uint8_t n);

    static uint8_t lenToDlc(uint8_t len);
    static uint8_t dlcToLen(uint8_t dlc);
};

}  // namespace EDM
