// FluidNC/src/EDM/Servo/FaultReason.h
#pragma once
#include <cstdint>
namespace EDM {
enum class FaultReason : uint8_t {
    None, AckTimeout, ProtocolMismatch, TouchOffNoContact,
    ServoStall, HeartbeatLost, PsuFault, SensorDisagree
};
}
