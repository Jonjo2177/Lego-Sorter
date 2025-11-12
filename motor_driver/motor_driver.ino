// Multi-Stepper Conveyor Controller (A4988/DRV8825)
// Full-step recommended: set MS1/MS2/MS3 LOW on each driver.

// ======= CONFIG =======
struct StepperCfg {
  uint8_t dirPin, stepPin, enPin;
  unsigned usStart;     // launch half-period (μs high + μs low = 2*us)
  unsigned usCruise;    // cruise half-period
  int      rampSteps;   // steps to accelerate over
  bool     dirHigh;     // initial direction
};

// Example: two motors. Add more entries as needed.
const StepperCfg MOTOR_CFGS[] = {
  // dir, step, en,   usStart, usCruise, rampSteps, dirHigh
  {4,    7,    10,    7000,    1200,     300,       false},  // M0
  {13,    12,    11,    6000,    1500,     250,       true },  // M1
};
const size_t N_MOTORS = sizeof(MOTOR_CFGS) / sizeof(MOTOR_CFGS[0]);

// ======= ENGINE =======
struct Stepper {
  // pins
  uint8_t dirPin, stepPin, enPin;

  // timing profile
  unsigned usStart, usCruise;
  int      rampSteps;

  // runtime
  bool     enabled = false;
  bool     holding = false;     // true if paused but coils energized
  bool     ramping = false;
  unsigned currentUs;           // current half-period (μs)
  int      rampIdx = 0;         // 0..rampSteps
  unsigned long nextDue = 0;    // micros() timestamp for next step edge

  // public API
  void begin(bool dirHigh) {
    pinMode(dirPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(enPin,   OUTPUT);
    digitalWrite(stepPin, LOW);
    digitalWrite(dirPin,  dirHigh ? HIGH : LOW);
    enableDriver();
    preEnergize();
  }

  void setDir(bool dirHigh) {
    digitalWrite(dirPin, dirHigh ? HIGH : LOW);
  }

  void enableDriver() {
    digitalWrite(enPin, LOW);   // A4988/DRV8825: LOW = enabled
    enabled = true;
  }
  void disableDriver() {
    digitalWrite(enPin, HIGH);  // disable outputs (freewheel)
    enabled = false;
  }

  void preEnergize() { delay(200); }

  void startWithRamp() {
    holding   = false;
    enableDriver();
    ramping   = true;
    rampIdx   = 0;
    currentUs = usStart;
    nextDue   = micros();       // start now
  }

  void resume(bool withRamp) {
    holding = false;
    enableDriver();
    if (withRamp) {
      startWithRamp();
    } else {
      ramping   = false;
      currentUs = usCruise;
      nextDue   = micros();
    }
  }

  // Stop options:
  // - hold=true  : stop stepping but keep coils energized (motor locks)
  // - hold=false : disable driver (motor free-spins)
  void stop(bool hold) {
    if (hold) {
      holding = true;   // just stop stepping
    } else {
      holding = false;
      disableDriver();
    }
  }

  // Call frequently from loop(); handles step timing.
  void service() {
    if (!enabled || holding) return;

    unsigned long now = micros();
    if (now < nextDue) return;

    // Emit one STEP pulse (min pulse width ~2-3 μs for A4988/DRV8825).
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(3);
    digitalWrite(stepPin, LOW);

    // Update ramp if active
    if (ramping) {
      // Linear ramp: from usStart -> usCruise across rampSteps
      // Compute next half-period based on next ramp index
      rampIdx++;
      if (rampIdx >= rampSteps) {
        ramping   = false;
        currentUs = usCruise;
      } else {
        unsigned delta = usStart - usCruise; // assume usStart > usCruise
        currentUs = usStart - (unsigned long)delta * rampIdx / rampSteps;
      }
    }

    // Schedule next step edge after a full period (HIGH+LOW = 2*currentUs)
    nextDue = now + (unsigned long)currentUs * 2;
  }
};

// ======= STORAGE =======
Stepper motors[N_MOTORS];

// ======= SETUP/LOOP =======
void setup() {
  Serial.begin(9600);

  // Build motor instances from config
  for (size_t i = 0; i < N_MOTORS; ++i) {
    motors[i].dirPin    = MOTOR_CFGS[i].dirPin;
    motors[i].stepPin   = MOTOR_CFGS[i].stepPin;
    motors[i].enPin     = MOTOR_CFGS[i].enPin;
    motors[i].usStart   = MOTOR_CFGS[i].usStart;
    motors[i].usCruise  = MOTOR_CFGS[i].usCruise;
    motors[i].rampSteps = MOTOR_CFGS[i].rampSteps;

    motors[i].begin(MOTOR_CFGS[i].dirHigh);
    motors[i].startWithRamp();   // strong launch for each motor
  }
}

void loop() {
  // Service all motors (non-blocking). Call as often as possible.
  for (size_t i = 0; i < N_MOTORS; ++i) {
    motors[i].service();
  }

  // ---- Demo behavior (optional): stop/resume sequences ----
  // Example: after 5 seconds, pause M0 (hold), after 7 seconds, resume with ramp
  static unsigned long t0 = millis();
  unsigned long t = millis() - t0;

  if (t > 5000 && t < 5200) {
    motors[0].stop(true);             // stop M0, hold torque
    Serial.println("M0 paused (holding).");
  }
  if (t > 7000 && t < 7200) {
    motors[0].resume(true);           // smooth ramp resume
    Serial.println("M0 resumed with ramp.");
  }

  // After 10s, fully disable M1; after 12s, resume instantly
  if (t > 10000 && t < 10200) {
    motors[1].stop(false);            // disable driver (free-spin)
    Serial.println("M1 stopped and disabled.");
  }
  if (t > 12000 && t < 12200) {
    motors[1].resume(false);          // instant resume at cruise
    Serial.println("M1 resumed instantly.");
  }

  // (Remove demo once integrated; the service loop itself is the important part.)
}
