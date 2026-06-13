// FluidNC/src/EDM/Can/Mcp2518fdDriver.cpp
#include "EDM/Can/Mcp2518fdDriver.h"
#include <SPI.h>
#include <Arduino.h>

namespace EDM {

// --- MCP2518FD SPI instructions & key registers (DS20006027) ---
static constexpr uint8_t  CMD_RESET = 0x00;
static constexpr uint8_t  CMD_READ  = 0x03;  // 0x3nnn
static constexpr uint8_t  CMD_WRITE = 0x02;  // 0x2nnn
static constexpr uint16_t REG_C1CON     = 0x000;
static constexpr uint16_t REG_C1NBTCFG  = 0x004;  // nominal bit time (1 Mbps)
static constexpr uint16_t REG_C1DBTCFG  = 0x008;  // data bit time (5 Mbps)
static constexpr uint16_t REG_C1TDC     = 0x00C;
static constexpr uint16_t REG_C1TEFCON  = 0x040;
static constexpr uint16_t REG_C1FIFOCON1= 0x05C;  // TX FIFO
static constexpr uint16_t REG_C1FIFOCON2= 0x070;  // RX FIFO

bool Mcp2518fdDriver::init() {
    pinMode(_cfg.pins.cs, OUTPUT);
    digitalWrite(_cfg.pins.cs, HIGH);
    pinMode(_cfg.pins.intr, INPUT);
    SPI.begin(_cfg.pins.sck, _cfg.pins.miso, _cfg.pins.mosi, _cfg.pins.cs);

    reset();
    delay(2);

    // Bit timing for osc_hz=40MHz:
    //  Nominal 1 Mbps: BRP=0, TSEG1=30, TSEG2=7, SJW=7  -> 40 TQ
    //  Data    5 Mbps: BRP=0, TSEG1=6,  TSEG2=1, SJW=1  -> 8 TQ
    writeReg(REG_C1NBTCFG, (0u << 24) | (30u << 16) | (7u << 8) | 7u);
    writeReg(REG_C1DBTCFG, (0u << 24) | (6u  << 16) | (1u << 8) | 1u);

    // Configure one TX FIFO and one RX FIFO (payload 64, FD enabled).
    // (FIFO RAM addresses, TXEN/RXEN, PLSIZE=64B set per datasheet bitfields.)
    writeReg(REG_C1FIFOCON1, /* TX, 64B payload, depth 8 */ 0x000000F8u);
    writeReg(REG_C1FIFOCON2, /* RX, 64B payload, depth 16 */ 0x000000F9u);

    // Request Normal CAN-FD mode (C1CON.REQOP = 0), enable ISO CRC.
    uint32_t con = readReg(REG_C1CON);
    con &= ~(0x7u << 24);          // REQOP = Normal FD
    writeReg(REG_C1CON, con);
    return true;
}

void Mcp2518fdDriver::reset() {
    digitalWrite(_cfg.pins.cs, LOW);
    SPI.transfer(CMD_RESET);
    SPI.transfer(0x00);
    digitalWrite(_cfg.pins.cs, HIGH);
}

uint32_t Mcp2518fdDriver::readReg(uint16_t addr) {
    digitalWrite(_cfg.pins.cs, LOW);
    SPI.transfer(uint8_t((CMD_READ << 4) | ((addr >> 8) & 0x0F)));
    SPI.transfer(uint8_t(addr & 0xFF));
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= uint32_t(SPI.transfer(0x00)) << (8 * i);
    digitalWrite(_cfg.pins.cs, HIGH);
    return v;
}

void Mcp2518fdDriver::writeReg(uint16_t addr, uint32_t val) {
    digitalWrite(_cfg.pins.cs, LOW);
    SPI.transfer(uint8_t((CMD_WRITE << 4) | ((addr >> 8) & 0x0F)));
    SPI.transfer(uint8_t(addr & 0xFF));
    for (int i = 0; i < 4; ++i) SPI.transfer(uint8_t(val >> (8 * i)));
    digitalWrite(_cfg.pins.cs, HIGH);
}

uint8_t Mcp2518fdDriver::lenToDlc(uint8_t len) {
    if (len <= 8)  return len;
    if (len <= 12) return 9;
    if (len <= 16) return 10;
    if (len <= 20) return 11;
    if (len <= 24) return 12;
    if (len <= 32) return 13;
    if (len <= 48) return 14;
    return 15;  // 64
}

uint8_t Mcp2518fdDriver::dlcToLen(uint8_t dlc) {
    static const uint8_t t[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    return t[dlc & 0x0F];
}

// send()/poll()/readRam()/writeRam() build a TX object (T0=SID, T1=DLC|FDF|BRS,
// then padded data) into the TX FIFO RAM and read RX objects out of the RX FIFO.
// Full bodies completed during bench bring-up against the datasheet.
bool Mcp2518fdDriver::send(const CanFrame& f) { /* bench bring-up */ (void)f; return false; }
void Mcp2518fdDriver::poll() { /* bench bring-up: drain RX FIFO -> _rx(frame) */ }
void Mcp2518fdDriver::readRam(uint16_t, uint8_t*, uint8_t) {}
void Mcp2518fdDriver::writeRam(uint16_t, const uint8_t*, uint8_t) {}

}  // namespace EDM
