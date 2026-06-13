// FluidNC/src/EDM/Can/CanBus.h
#pragma once
#include <functional>
#include <vector>
#include "EDM/Can/CanFrame.h"

namespace EDM {

// Abstract CAN-FD transport. Implemented by Mcp2518fdDriver (target) and
// FakeCanBus (host tests). RX is delivered via a callback set by the owner.
class CanBus {
public:
    using RxHandler = std::function<void(const CanFrame&)>;
    virtual ~CanBus() = default;
    virtual bool init()                     = 0;
    virtual bool send(const CanFrame& f)    = 0;
    virtual void onReceive(RxHandler h)     = 0;
    // Drains hardware RX FIFO, invoking the handler per frame. Called from the
    // CAN-RX task on target; a no-op for fakes that inject directly.
    virtual void poll()                     {}
};

namespace test_support {

// Host-only fake: records sent frames; lets a test inject received frames.
class FakeCanBus : public CanBus {
public:
    bool init() override { return true; }
    bool send(const CanFrame& f) override { sent.push_back(f); return true; }
    void onReceive(RxHandler h) override { _h = std::move(h); }
    void inject(const CanFrame& f) { if (_h) _h(f); }

    std::vector<CanFrame> sent;
private:
    RxHandler _h;
};

}  // namespace test_support
}  // namespace EDM
