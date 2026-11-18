#include "../wearable/DebouncedButton.h"
#include <cassert>
#include <iostream>
#include <cstdint>

void prime(DebouncedButton &b) {
  assert(!b.update(0,false));
  assert(!b.update(29,false));
  assert(!b.update(30,false));
  assert(b.primed && !b.stable && b.presses==0);
}

int main() {
  // A and B share exactly the same press-edge semantics. Which virtual action
  // an edge requests is exclusively a backend concern, not a firmware arm mode.
  for (uint8_t pin:{25,26}) {
    DebouncedButton button{pin}; prime(button);
    assert(!button.update(100,true));
    assert(!button.update(129,true));
    assert(button.update(130,true));
    assert(button.stable && button.presses==1);
    for(uint32_t t=140;t<5000;t+=10) assert(!button.update(t,true));
    assert(button.presses==1); // Long holds count only once.
    assert(!button.update(5000,false));
    assert(!button.update(5029,false));
    assert(button.update(5030,false));
    assert(!button.stable && button.presses==1); // Release is not a click.
    assert(!button.update(5100,true));
    assert(button.update(5130,true));
    assert(button.presses==2);
  }

  // A held button at boot is adopted without an event; its first release is
  // also event-free. Only a subsequent real debounced press increments.
  DebouncedButton held{25};
  assert(!held.update(0,true));assert(!held.update(30,true));
  assert(held.primed && held.stable && held.presses==0);
  assert(!held.update(1000,true));
  assert(!held.update(1010,false));assert(held.update(1040,false));
  assert(held.presses==0);
  assert(!held.update(1100,true));assert(held.update(1130,true));
  assert(held.presses==1);

  // Sub-30ms bounce cannot create extra start/pause or gripper events.
  DebouncedButton bounced{26};prime(bounced);
  assert(!bounced.update(100,true));assert(!bounced.update(110,false));
  assert(!bounced.update(120,true));assert(!bounced.update(149,true));
  assert(bounced.update(150,true));assert(bounced.presses==1);
  assert(!bounced.update(160,false));assert(!bounced.update(170,true));
  assert(!bounced.update(200,true));assert(bounced.presses==1);

  // Edge counters wrap naturally; backend consumes modulo-uint32 deltas.
  DebouncedButton wrapped{25};prime(wrapped);wrapped.presses=UINT32_MAX;
  assert(!wrapped.update(100,true));assert(wrapped.update(130,true));
  assert(wrapped.presses==0);
  // Millisecond wrap must not bypass or stall the30ms debounce window.
  DebouncedButton timeWrapped{26};timeWrapped.primed=true;
  assert(!timeWrapped.update(UINT32_MAX-15,true));
  assert(!timeWrapped.update(13,true));assert(timeWrapped.update(14,true));
  assert(timeWrapped.presses==1);
  std::cout<<"Button regression passed: A/B press edges,30ms debounce, hold once, release, boot held, bounce, counter/time wrap.\n";
}
