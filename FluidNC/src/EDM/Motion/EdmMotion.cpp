// FluidNC/src/EDM/Motion/EdmMotion.cpp
//
// ESP32-COUPLED (NOT host-testable; NOT in the native test build_src_filter).
// See EdmMotion.h for the overview. This file uses the FluidNC planner/protocol/
// kinematics/state machinery, all of which only exist in the ESP32 build, so it is
// intentionally excluded from [tests_common]; it is verified by the firmware build
// and an on-target dry-run (P4).
//
// The emit loop runs on the gcode thread. Each pass it:
//   1) pumps realtime + auto-cycle-start so the planner drains and realtime
//      commands (feed-hold, reset, alarm) are serviced,
//   2) bails on abort / alarm / controller fault / stop (M5),
//   3) freezes dt while feed-held so a paused servo does not accumulate phantom
//      advance,
//   4) reads v_cmd (um/s) from the controller and steps the PathSampler by dt,
//   5) submits the resulting short segment to plan_buffer_line() once there is room
//      in a deliberately shallow planner buffer (so retracts react quickly).

#include "EDM/Motion/EdmMotion.h"

#include "EDM/Motion/Pose4.h"
#include "EDM/Motion/ContourBuffer.h"
#include "EDM/Motion/PathSampler.h"
#include "EDM/Control/EdmController.h"
#include "EDM/EdmSpindle.h"

#include "Planner.h"    // plan_buffer_line, plan_check_full_buffer, plan_get_block_buffer_available
#include "Protocol.h"   // protocol_execute_realtime, protocol_buffer_synchronize, protocol_auto_cycle_start, send_alarm
#include "System.h"     // sys, get_mpos, MAX_N_AXIS
#include "State.h"      // State, state_is
#include "Types.h"      // X_AXIS, Y_AXIS, U_AXIS, V_AXIS, axis_t
#include "NutsBolts.h"  // get_ms, delay_ms
#include "Machine/MachineConfig.h"  // config, _kinematics, _arcTolerance
#include "Spindles/Spindle.h"       // extern Spindles::Spindle* spindle

#include <cmath>
#include <cstring>

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

namespace EDM { namespace motion {

// Keep the planner shallow so a retract command can take effect within a couple of
// segments instead of after the whole buffer drains.
static constexpr int EDM_MAX_INFLIGHT = 3;

// ---------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------

// Returns the active EDM controller, or nullptr if the active spindle is not the
// EDM spindle. Identity is by name() so we never static_cast the wrong type.
static EDM::EdmController* activeEdmController() {
    if (!spindle || std::strcmp(spindle->name(), "EDM") != 0) {
        return nullptr;
    }
    return static_cast<Spindles::EdmSpindle*>(spindle)->controller();
}

// X/Y -> lower guide, U/V -> upper guide. Indices confirmed in Types.h
// (X_AXIS=0, Y_AXIS=1, U_AXIS=6, V_AXIS=7).
static Pose4 poseFromMpos(const float* m) {
    return Pose4{ m[X_AXIS], m[Y_AXIS], m[U_AXIS], m[V_AXIS] };
}
static Pose4 poseFromTarget(const float* t) {
    return Pose4{ t[X_AXIS], t[Y_AXIS], t[U_AXIS], t[V_AXIS] };
}

// Build a full MAX_N_AXIS target: copy the current machine position into every axis
// (so unrelated axes hold position), then overwrite the four EDM axes from the pose.
static void fanOut(const Pose4& p, const float* mpos, float* t) {
    for (size_t i = 0; i < MAX_N_AXIS; ++i) {
        t[i] = mpos[i];
    }
    t[X_AXIS] = p.x;
    t[Y_AXIS] = p.y;
    t[U_AXIS] = p.u;
    t[V_AXIS] = p.v;
}

// Shared emit loop. Pre-validation of vertices is done by the callers (line vs arc
// know their own vertex set); here we drive the sampler and feed the planner.
// Returns true on normal block completion, false otherwise.
static bool runContour(EDM::EdmController* ctl, const ContourBuffer& contour, const plan_line_data_t* pl_data) {
    SamplerConfig cfg{};
    SamplerState  st{};
    PathSampler   sampler(contour, cfg);

    uint32_t prev = get_ms();

    for (;;) {
        // Drain the planner and service realtime commands.
        protocol_auto_cycle_start();
        protocol_execute_realtime();

        if (sys.abort()) {
            ctl->requestStop();
            return false;
        }
        if (state_is(State::Alarm)) {
            ctl->requestStop();
            return false;
        }

        const EdmState es = ctl->reportedState();
        // Hard fault: stop feeding, let queued motion finish, raise a critical alarm.
        // BreakRelief is NOT an abort -- the controller drives the retract via v_cmd.
        if (es == EdmState::StallFault || es == EdmState::Fault) {
            protocol_buffer_synchronize();
            send_alarm(ExecAlarm::AbortCycle);  // TODO(P4): consider a dedicated EDM fault alarm code.
            return false;
        }
        // Idle means M5/stop landed mid-line: flush what is queued and report not-done.
        if (es == EdmState::Idle) {
            protocol_buffer_synchronize();
            return false;
        }

        // Feed-hold (or any non-running, non-idle state): do not accumulate dt while
        // motion is paused, otherwise the servo would command a large phantom step
        // the instant the hold releases. Reset the clock and wait.
        if (!state_is(State::Cycle) && !state_is(State::Idle)) {
            prev = get_ms();
            delay_ms(1);
            continue;
        }

        const uint32_t now = get_ms();
        const float    dt  = float(now - prev) * 1e-3f;
        prev               = now;

        const int32_t v = ctl->vCmdUmPerS();
        SampleResult  r = sampler.step(st, v, dt);

        // Submit this tick's segment FIRST. When the sampler reports block_done it
        // also returns the final approach segment (want_emit includes block_done),
        // so checking done before emitting would drop the last segment and stop the
        // machine up to seg_max short of the programmed endpoint.
        if (r.emit != Emit::None) {
            // Wait until the (deliberately shallow) planner buffer has room. in-flight
            // count is derived from plan_get_block_buffer_available():
            //   in_flight = (planner_blocks - 1) - available_blocks.
            for (;;) {
                const int free_blocks = plan_get_block_buffer_available();
                const int in_flight   = (config->_planner_blocks - 1) - free_blocks;
                if (!plan_check_full_buffer() && in_flight < EDM_MAX_INFLIGHT) {
                    break;
                }
                protocol_execute_realtime();
                if (sys.abort()) {
                    ctl->requestStop();
                    return false;
                }
                if (state_is(State::Alarm)) {
                    ctl->requestStop();
                    return false;
                }
                delay_ms(1);
            }

            float t[MAX_N_AXIS];
            fanOut(r.target, get_mpos(), t);

            plan_line_data_t pl    = *pl_data;
            pl.feed_rate           = r.feed_mm_min;
            pl.motion.rapidMotion  = 0;
            pl.motion.noFeedOverride = 1;  // servo owns the rate; gcode feed-override must not scale it.
            pl.motion.inverseTime  = 0;
            pl.limits_checked      = true;  // vertices were pre-validated; skip per-segment soft-limit.

            if (!plan_buffer_line(t, &pl)) {
                // Zero-length segments are excluded by the sampler's seg_min, so a
                // false return here means the segment violated a soft limit.
                ctl->requestStop();
                send_alarm(ExecAlarm::SoftLimit);
                return false;
            }
        }

        // Now that the final segment is queued, complete on forward arrival.
        // (st.done is set by the sampler, so the loop never iterates again to
        // produce a spurious zero-length segment.)
        if (r.block_done && v >= 0) {
            protocol_buffer_synchronize();
            return true;
        }

        delay_ms(1);
    }
}

// ---------------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------------

bool runCut(float* target, plan_line_data_t* pl_data, float* /*position*/) {
    EDM::EdmController* ctl = activeEdmController();
    if (!ctl) {
        return false;
    }

    ContourBuffer contour;
    contour.reset(poseFromMpos(get_mpos()));
    contour.appendLine(poseFromTarget(target));

    // Pre-validate every vertex once against soft limits, so the emit loop can submit
    // segments with limits_checked = true (segment endpoints lie between vertices, so
    // validating the vertices bounds the whole path).
    for (size_t i = 0; i < contour.count(); ++i) {
        float t[MAX_N_AXIS];
        fanOut(contour.vertexPose(i), get_mpos(), t);
        if (config->_kinematics->invalid_line(t)) {
            ctl->requestStop();
            send_alarm(ExecAlarm::SoftLimit);
            return false;
        }
    }

    return runContour(ctl, contour, pl_data);
}

bool runCutArc(float* target, plan_line_data_t* pl_data, float* position,
               float* offset, float radius, size_t axis_0, size_t axis_1,
               bool is_clockwise, uint32_t rotations) {
    EDM::EdmController* ctl = activeEdmController();
    if (!ctl) {
        return false;
    }

    // Mirror mc_arc()'s center + angular-travel computation (MotionControl.cpp).
    const float center_x = position[axis_0] + offset[axis_0];
    const float center_y = position[axis_1] + offset[axis_1];

    // Radius vector from center to current position, and from center to target.
    const float radii[2] = { -offset[axis_0], -offset[axis_1] };
    const float rt[2]    = { target[axis_0] - center_x, target[axis_1] - center_y };

    // CCW angle between current and target about the center (single atan2, as mc_arc).
    float angular_travel = atan2f(radii[0] * rt[1] - radii[1] * rt[0], radii[0] * rt[0] + radii[1] * rt[1]);
    if (is_clockwise) {
        if (angular_travel >= -static_cast<float>(ARC_ANGULAR_TRAVEL_EPSILON)) {
            angular_travel -= 2 * static_cast<float>(M_PI);
        }
        if (rotations > 1) {
            angular_travel -= (rotations - 1) * 2 * static_cast<float>(M_PI);
        }
    } else {
        if (angular_travel <= static_cast<float>(ARC_ANGULAR_TRAVEL_EPSILON)) {
            angular_travel += 2 * static_cast<float>(M_PI);
        }
        if (rotations > 1) {
            angular_travel += (rotations - 1) * 2 * static_cast<float>(M_PI);
        }
    }

    ContourBuffer contour;
    contour.reset(poseFromMpos(get_mpos()));
    contour.appendArc(poseFromTarget(target), center_x, center_y, radius, angular_travel, config->_arcTolerance);

    // Pre-validate every contour vertex against soft limits (see runCut).
    for (size_t i = 0; i < contour.count(); ++i) {
        float t[MAX_N_AXIS];
        fanOut(contour.vertexPose(i), get_mpos(), t);
        if (config->_kinematics->invalid_line(t)) {
            ctl->requestStop();
            send_alarm(ExecAlarm::SoftLimit);
            return false;
        }
    }

    return runContour(ctl, contour, pl_data);
}

}}  // namespace EDM::motion
