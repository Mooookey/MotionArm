#pragma once
#include <cstdint>

// A counter belongs to each debounced press edge, never to the held level.
// Priming adopts the initial level without an event, including a held button
// at boot. Unsigned elapsed time and counters also tolerate uint32 wraparound.
struct DebouncedButton {
  uint8_t pin;
  bool raw = false;
  bool stable = false;
  bool primed = false;
  uint32_t changed = 0;
  uint32_t presses = 0;

  bool update(uint32_t now, bool pressed) {
    if (pressed != raw) {
      raw = pressed;
      changed = now;
    }
    if (uint32_t(now - changed) < 30) return false;
    if (!primed) {
      primed = true;
      stable = raw;
      return false;
    }
    if (stable == raw) return false;
    stable = raw;
    if (stable) ++presses;
    return true;
  }
};
