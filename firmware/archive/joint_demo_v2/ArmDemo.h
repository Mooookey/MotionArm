#pragma once

// These are nominal PCA9685 command values. Check the combined poses in air first.
namespace demo {
struct Pose {
  int base, shoulder, elbow;
};

constexpr Pose readyPose{ 1610, 1030, 1050 };
constexpr Pose pickupReady{ 1010, 1030, 1050 };
// Estimated extra 15 degrees down: 600/45 and 520/45 us per degree.
// These endpoints extend beyond measured samples and require an unloaded check.
constexpr Pose pick{ 1010, 830, 877 };
constexpr Pose lift = pickupReady;
// User measured 1610 -> 2000 us as approximately 45 degrees counterclockwise.
constexpr int transferBaseUs = 2000;
constexpr Pose turn{ transferBaseUs, 1030, 1050 };
constexpr Pose place{ transferBaseUs, 830, 877 };
constexpr uint32_t liftMs = 500;
// Twice the angular distance at half the previous average angular speed.
constexpr uint32_t turnMs = 1440;
constexpr uint32_t approachMs = turnMs / 2;
constexpr int openUs = kGripperOpenUs;
constexpr int closedUs = 1630;
constexpr uint32_t openMs = 350;
constexpr uint32_t closeMs = 160;
constexpr uint32_t carryLiftMs = liftMs;
constexpr uint32_t carryTurnMs = turnMs;
constexpr uint32_t carryLowerMs = liftMs;
// User chose a bounded three-second hold for this demonstration only.
// Manual gripper commands retain their original 700 ms output limit.
constexpr uint32_t demoGripLimitMs = 3000;
constexpr bool carryFitsDeadline =
    closeMs + carryLiftMs + carryTurnMs + carryLowerMs < demoGripLimitMs - 50;
static_assert(carryFitsDeadline, "Carry must finish before the demo grip deadline.");

enum class Grip { Keep, Open, Close };
struct Phase {
  Pose pose;
  uint32_t duration;
  Grip grip;
};

constexpr Phase toPick[]{{pickupReady, approachMs, Grip::Keep}};
constexpr Phase dry[]{{pickupReady, openMs, Grip::Open}, {pick, liftMs, Grip::Keep},
                      {lift, liftMs, Grip::Keep},
                      {turn, turnMs, Grip::Keep}, {place, liftMs, Grip::Keep},
                      {place, openMs, Grip::Open}, {turn, liftMs, Grip::Keep},
                      {lift, turnMs, Grip::Keep}};
constexpr Phase carry[]{{pickupReady, openMs, Grip::Open}, {pick, liftMs, Grip::Keep},
                        {pick, closeMs, Grip::Close},
                        {lift, carryLiftMs, Grip::Keep}, {turn, carryTurnMs, Grip::Keep},
                        {place, carryLowerMs, Grip::Keep}, {place, openMs, Grip::Open},
                        {turn, liftMs, Grip::Keep}, {lift, turnMs, Grip::Keep}};

enum class State { Idle, Ready, Standby, Stopped };
State state = State::Idle;
const Phase *phases = nullptr;
size_t phaseCount = 0, index = 0;
State finishedState = State::Idle;
bool dryComplete = false, setsDry = false;
Grip activeGrip = Grip::Keep;
uint32_t gripStarted = 0, phaseStarted = 0, lastUpdate = 0;
Pose from{};

const char *name() {
  if (phases) return "MOVING";
  switch (state) {
    case State::Ready: return "READY";
    case State::Standby: return "PICK_READY";
    case State::Stopped: return "STOPPED";
    default: return "IDLE";
  }
}

bool ownsOutputs() { return state != State::Idle; }

Pose applied() { return {parkedUs[14], parkedUs[15], parkedUs[13]}; }

bool write(int channel, int pulse) {
  if (parkedUs[channel] == pulse) return true;
  if (writePulse(channel, pulse)) {
    parkedUs[channel] = pulse;
    return true;
  }
  // A failed transaction is ambiguous: keep the channel tracked for OFF.
  if (!parkedUs[channel]) parkedUs[channel] = pulse;
  ready = false;
  Serial.println("DEMO I2C ERROR. Support both arms and cut servo power.");
  return false;
}

bool writePose(Pose pose) {
  return write(14, pose.base) && write(15, pose.shoulder) && write(13, pose.elbow);
}

void setGrip(Grip grip, uint32_t now) {
  activeGrip = grip;
  gripStarted = now;
  if (!write(12, grip == Grip::Close ? closedUs : openUs)) {
    phases = nullptr;
    dryComplete = false;
    state = State::Stopped;
    activeGrip = Grip::Keep;
    write(12, 0);
  }
}

void stop(uint32_t now, bool releaseObject) {
  phases = nullptr;
  dryComplete = false;
  state = State::Stopped;
  if (releaseObject && (activeGrip == Grip::Close || parkedUs[12] == closedUs)) {
    setGrip(Grip::Open, now);
  }
}

void reset() {
  phases = nullptr;
  activeGrip = Grip::Keep;
  dryComplete = false;
  state = State::Idle;
}

void beginPhase(uint32_t now) {
  from = applied();
  phaseStarted = now;
  lastUpdate = now;
  if (phases[index].grip != Grip::Keep) setGrip(phases[index].grip, now);
}

template <size_t N>
void begin(const Phase (&sequence)[N], State finish, uint32_t now, bool isDry = false) {
  phases = sequence;
  phaseCount = N;
  index = 0;
  finishedState = finish;
  setsDry = isDry;
  beginPhase(now);
}

int interpolate(int a, int b, float fraction) {
  const float delta = (b - a) * fraction;
  return a + int(delta + (delta >= 0 ? 0.5f : -0.5f));
}

void update(uint32_t now) {
  if (activeGrip == Grip::Close && now - gripStarted >= demoGripLimitMs - 50) {
    Serial.println("DEMO grip deadline. Opening above the catch surface.");
    stop(now, true);
  } else if (activeGrip == Grip::Open && now - gripStarted >= openMs) {
    activeGrip = Grip::Keep;
    if (!write(12, 0)) stop(now, false);
  }
  if (!phases) return;
  if (!ready) {
    stop(now, true);
    return;
  }
  const Phase step = phases[index];
  const uint32_t elapsed = now - phaseStarted;
  if (elapsed < step.duration && now - lastUpdate < 20) return;
  lastUpdate = now;
  const float fraction = elapsed >= step.duration ? 1.0f : float(elapsed) / step.duration;
  const float eased = fraction * fraction * (3.0f - 2.0f * fraction);
  if (!writePose({interpolate(from.base, step.pose.base, eased),
                  interpolate(from.shoulder, step.pose.shoulder, eased),
                  interpolate(from.elbow, step.pose.elbow, eased)})) {
    stop(now, true);
    return;
  }
  if (elapsed < step.duration) return;
  if (++index == phaseCount) {
    phases = nullptr;
    state = finishedState;
    dryComplete = dryComplete || setsDry;
    Serial.printf("DEMO done: %s\n", name());
  } else {
    beginPhase(now);
  }
}

void command(const char *mode, uint32_t now) {
  if (!ready || selected || phases) {
    Serial.println("DEMO unavailable: check PCA and finish the current command.");
    return;
  }
  if (!strcmp(mode, "ARM") && state == State::Idle) {
    for (const Joint &joint : joints) {
      if (parkedUs[joint.channel]) {
        Serial.println("Support the arm and clear manual outputs before DEMO ARM.");
        return;
      }
    }
    state = State::Ready;
    if (!writePose(readyPose)) {
      stop(now, false);
      return;
    }
    setGrip(Grip::Open, now);
  } else if (!strcmp(mode, "APPROACH") && state == State::Ready) {
    begin(toPick, State::Standby, now);
  } else if ((!strcmp(mode, "APPROACH") && state == State::Standby) ||
             (!strcmp(mode, "ARM") && state == State::Ready)) {
    Serial.println("Already at requested ready pose. Check STATUS.");
    return;
  } else if (!strcmp(mode, "DRY") && state == State::Standby) {
    begin(dry, State::Standby, now, true);
  } else if (!strcmp(mode, "GRIP") && state == State::Standby && dryComplete) {
    if (!carryFitsDeadline) {
      Serial.println("GRIP unavailable: carry exceeds demo grip deadline.");
      return;
    }
    begin(carry, State::Standby, now);
  } else {
    Serial.println("DEMO order: ARM, APPROACH, DRY, GRIP. Check STATUS.");
    return;
  }
  Serial.printf("DEMO %s: %s\n", mode, name());
}
}  // namespace demo
