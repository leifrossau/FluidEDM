// FluidNC/src/EDM/Diel/IDielLink.h
#pragma once
#include <cstdint>
#include "EDM/Diel/DielProtocol.h"
namespace EDM { namespace diel {
struct DielEvent { enum Kind:uint8_t{ FaultEvt } kind = FaultEvt; DielFault fault; };
class IDielLink {
public:
  virtual ~IDielLink() = default;
  virtual uint16_t setDiel(const SetDiel& s) = 0;     // returns seq
  virtual bool latestStats(DielStats& out) const = 0;
  virtual bool popEvent(DielEvent& out) = 0;
  virtual bool isConnected() const = 0;                // heartbeat fresh
  virtual bool protocolCompatible() const = 0;
  virtual bool present() const = 0;                    // a module responded
};
}}  // namespace EDM::diel
