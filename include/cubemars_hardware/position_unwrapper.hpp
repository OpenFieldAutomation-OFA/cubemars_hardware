#ifndef CUBEMARS_HARDWARE__POSITION_UNWRAPPER_HPP_
#define CUBEMARS_HARDWARE__POSITION_UNWRAPPER_HPP_

#include <cstdint>
#include <cstdlib>
#include <initializer_list>

namespace cubemars_hardware
{

// The motor status frame reports position as an int16 in 0.1 deg units.
//
// ASSUMPTION (not confirmed by repo docs or hardware): when the value leaves its range the
// firmware either
//   (a) wraps at the nominal +/-3200 deg limit, i.e. +32000 -> -32000 (period 64000 counts), or
//   (b) wraps as a plain int16, i.e. +32767 -> -32768 (period 65536 counts).
// Both are handled: on every frame the raw step is corrected by whichever candidate period
// yields the smallest |step|. Real motion between two frames is tiny compared with either
// period, so the right candidate is unambiguous. If the firmware instead SATURATES (clamps),
// no wrap is visible and the position will stall at the limit; this cannot be fixed in software.
constexpr std::int32_t kPositionWrapPeriodNominal = 64000;
constexpr std::int32_t kPositionWrapPeriodInt16 = 65536;

// Returns the wrap-corrected signed step from prev_raw to raw (in 0.1 deg counts).
inline std::int32_t unwrap_position_delta(std::int16_t prev_raw, std::int16_t raw)
{
  const std::int32_t delta = static_cast<std::int32_t>(raw) - static_cast<std::int32_t>(prev_raw);
  std::int32_t best = delta;
  for (const std::int32_t period : {kPositionWrapPeriodNominal, kPositionWrapPeriodInt16}) {
    for (const std::int32_t candidate : {delta - period, delta + period}) {
      if (std::abs(candidate) < std::abs(best)) {
        best = candidate;
      }
    }
  }
  return best;
}

// Accumulates raw int16 frames into a continuous multi-turn position (in 0.1 deg counts).
struct PositionUnwrapper
{
  bool initialized{false};
  std::int16_t prev_raw{0};
  double counts{0.0};

  void reset() { initialized = false; prev_raw = 0; counts = 0.0; }

  // Feed one frame, in order. The first frame after reset() initializes the position.
  double update(std::int16_t raw)
  {
    if (!initialized) {
      initialized = true;
      counts = raw;
    } else {
      counts += unwrap_position_delta(prev_raw, raw);
    }
    prev_raw = raw;
    return counts;
  }
};

}  // namespace cubemars_hardware

#endif  // CUBEMARS_HARDWARE__POSITION_UNWRAPPER_HPP_
