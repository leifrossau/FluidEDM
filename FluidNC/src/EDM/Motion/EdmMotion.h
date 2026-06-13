// FluidNC/src/EDM/Motion/EdmMotion.h
//
// ESP32-COUPLED (NOT host-testable; NOT in the native test build_src_filter).
// Bridges the PURE EDM motion stack (ContourBuffer + PathSampler) to the FluidNC
// planner. Called from the gcode thread (GCode.cpp) when the active spindle is the
// EDM spindle and cutting (M3). Verified by the ESP32 firmware build + on-target
// dry-run (P4); never compiled into the native unit-test build.
//
// runCut() / runCutArc() are drop-in replacements for mc_linear() / mc_arc() that,
// instead of streaming the whole move into the planner at the programmed feed,
// servo-feed the path forward (and retract) under closed-loop gap control: each
// loop pass reads the controller's commanded velocity (v_cmd, um/s), advances the
// PathSampler by the elapsed dt, and submits short segments to plan_buffer_line()
// while keeping the planner buffer shallow so retracts stay responsive.
#pragma once

#include "Planner.h"  // plan_line_data_t

#include <cstddef>
#include <cstdint>

namespace EDM { namespace motion {

// Servo-fed linear cut. Mirrors mc_linear(target, pl_data, position).
// Returns true when the block completed normally, false on stop/abort/fault/soft-limit.
bool runCut(float* target, plan_line_data_t* pl_data, float* position);

// Servo-fed arc cut. Mirrors mc_arc()'s offset/radius/cw geometry. axis_0/axis_1 are
// the circle-plane axis indices (X/Y for the canonical G17 plane); the helical/linear
// axis from mc_arc is intentionally dropped (EDM moves only in the X/Y + U/V planes).
bool runCutArc(float* target, plan_line_data_t* pl_data, float* position,
               float* offset, float radius, size_t axis_0, size_t axis_1,
               bool is_clockwise, uint32_t rotations);

}}  // namespace EDM::motion
