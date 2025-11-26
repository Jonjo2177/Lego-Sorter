//===========================================================
// Multi-Stepper Conveyor Sorter (A4988/DRV8825 + 2 Servos)
//===========================================================
// - Full-step recommended: MS1/MS2/MS3 = LOW on each driver.
// - Serial protocol from PC vision: "RED\n", "GREEN\n", "BLUE\n"
// - Non-blocking state machine: pauses conveyor, diverts, resumes.
// - Pins avoid conflicts with Serial (0/1) and Servos (9/10).
//
// Hardware map (Arduino UNO suggested):
//   M0 (Conveyor): DIR=2, STEP=3, EN=8
//   M1 (Feeder/Aux): DIR=4, STEP=5, EN=12
//   Servos: ARM1=9, ARM2=10
//   LEDs: RED=11, GREEN=6, BLUE=7
//
// Baud must match PC sender (e.g., 9600).
//===========================================================

#include <Arduino.h>
#include <Servo.h>

// ======= CONFIG =======
struct StepperCfg {
  uint8_t dirPin, stepPin, enPin;
  unsigned usStart;     // launch half-period (μs high + μs low = 2*us)
  unsigned usCruise;    // cruise half-period
  int      rampSteps;   // steps to accelerate over
  bool     dirHigh;     // initial direction level
};

// Example: two motors (add more entries if needed).
const StepperCfg MOTOR_CFGS[] = {
  // dir, step, en,  usStart, rampSteps, dirHigh
  {4,    7,    0,   7000,     1500,     300,       false},  // M0 = conveyor
  {6,    5,    1,  6000,     2000,     250,       true }   // M1 = feeder/aux

};

const size_t N_MOTORS = sizeof(MOTOR_CFGS) / sizeof(MOTOR_CFGS[0]);

// ======= ENGINE =======
struct Stepper {
  // pins
  uint8_t dirPin = 0, stepPin = 0, enPin = 0;

  // timing profile
  unsigned usStart = 0, usCruise = 0;
  int      rampSteps = 0;

  // runtime
  bool     enabled  = false;
  bool     holding  = false;     // true if paused but coils energized
  bool     ramping  = false;
  unsigned currentUs = 0;        // current half-period (μs)
  int      rampIdx = 0;          // 0..rampSteps
  unsigned long nextDue = 0;     // micros() timestamp for next step edge

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
    // A4988/DRV8825: ENABLE is active-LOW
    digitalWrite(enPin, LOW);
    enabled = true;
  }

  void disableDriver() {
    // Disable outputs (freewheel)
    digitalWrite(enPin, HIGH);
    enabled = false;
  }

  void preEnergize() { delay(200); }  // brief coil settle (ok at startup)

  void startWithRamp() {
    holding   = false;
    enableDriver();
    ramping   = true;
    rampIdx   = 0;
    currentUs = usStart;
    nextDue   = micros();  // start now
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
      holding = true;    // just stop stepping
    } else {
      holding = false;
      disableDriver();
    }
  }

  // Call frequently from loop(); handles step timing non-blocking.
  void service() {
    if (!enabled || holding) return;

    unsigned long now = micros();
    if ((long)(now - nextDue) < 0) return;

    // Emit one STEP pulse (min ~2-3 μs for A4988/DRV8825).
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(3);
    digitalWrite(stepPin, LOW);

    // Update ramp if active
    if (ramping) {
      if (rampSteps <= 0 || usStart <= usCruise) {
        // Degenerate profile -> drop to cruise
        ramping   = false;
        currentUs = usCruise;
      } else {
        rampIdx++;
        if (rampIdx >= rampSteps) {
          ramping   = false;
          currentUs = usCruise;
        } else {
          unsigned delta = usStart - usCruise; // assume usStart > usCruise
          currentUs = usStart - (unsigned long)delta * rampIdx / (unsigned long)rampSteps;
        }
      }
    }

    // Schedule next step edge after a full period (HIGH+LOW = 2*currentUs)
    nextDue = now + (unsigned long)currentUs * 2UL;
  }
};

// ======= STORAGE =======
Stepper motors[N_MOTORS];

// ======= SERVOS & LEDS =======
Servo ARM1, ARM2;
const int SERVO1_PIN = 9;   // do not reuse for anything else
const int SERVO2_PIN = 10;

/*
const int RED_LED_PIN   = 11;
const int GREEN_LED_PIN = 6;
const int BLUE_LED_PIN  = 7;
*/

// Servo neutral and positions (tune to your diverter geometry)
const int SERVO1_NEUTRAL = 180;
const int SERVO2_NEUTRAL = 180;

// Example divert positions (edit to match your rig)
const int SERVO1_RED_POS   = 180;
const int SERVO2_RED_POS   = 40;

const int SERVO1_GREEN_POS = 180;
const int SERVO2_GREEN_POS = 120;

const int SERVO1_BLUE_POS  = 40;
const int SERVO2_BLUE_POS  = 180;

// ======= SERIAL / PROTOCOL =======
String incomingColor = "";   // accumulate until '\n'
String Color = "";           // last complete command
bool   newData = false;

// ======= SORTER STATE MACHINE =======
enum SortState { IDLE_RUN, DIVERT_START, DIVERT_WAIT, DIVERT_RESET, DIVERT_PAUSE};
SortState state = IDLE_RUN;

unsigned long tMillis = 0;
unsigned long phaseStart = 0;

// Tunables (ms)
const unsigned settleBeforeDivert = 150;   // pause conveyor -> allow brick to settle
const unsigned divertSwingTime    = 1000;   // hold diverter in position
const unsigned resetSwingTime     = 8000;   // bring diverter back to neutral
const unsigned visionTimeout      = 5000;  // if vision silent, keep conveyor running

// Non-blocking 4-second pause after each sort
const unsigned long pauseDuration = 1000;  // 1 seconds

unsigned long lastVisionAt = 0;

// ======= HELPERS =======
/*void setColorLED(bool r, bool g, bool b) {
  digitalWrite(RED_LED_PIN,   r ? HIGH : LOW);
  digitalWrite(GREEN_LED_PIN, g ? HIGH : LOW);
  digitalWrite(BLUE_LED_PIN,  b ? HIGH : LOW);
}*/

void receiveColorCommand() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      String line = incomingColor;
      incomingColor = "";   // clear buffer
      line.trim();          // remove spaces and \r

      // ---- SPEED COMMAND: "SPD0:<number>" ----
      if (line.startsWith("SPD0:")) {
        String valStr = line.substring(5);  // part after "SPD:"
        unsigned long newUsCruise = valStr.toInt();

        // Optional safety limits (tune for your rig)
        if (newUsCruise >= 500 && newUsCruise <= 20000) {
          motors[0].usCruise = (unsigned)newUsCruise;

          // If the conveyor is already cruising, update currentUs too
          if (!motors[0].ramping && !motors[0].holding && motors[0].enabled) {
            motors[0].currentUs = (unsigned)newUsCruise;
          }

          Serial.print(F("New usCruise: "));
          Serial.println(motors[0].usCruise);
        } else {
          Serial.print(F("SPD out of range: "));
          Serial.println(newUsCruise);
        }

        // Do NOT start a divert cycle for SPD command
        return;
      }

      // ---- SPEED COMMAND: "SPD1:<number>" ----
      if (line.startsWith("SPD0:")) {
        String valStr = line.substring(5);  // part after "SPD:"
        unsigned long newUsCruise = valStr.toInt();

        // Optional safety limits (tune for your rig)
        if (newUsCruise >= 500 && newUsCruise <= 20000) {
          motors[1].usCruise = (unsigned)newUsCruise;

          // If the conveyor is already cruising, update currentUs too
          if (!motors[1].ramping && !motors[1].holding && motors[1].enabled) {
            motors[1].currentUs = (unsigned)newUsCruise;
          }

          Serial.print(F("New usCruise: "));
          Serial.println(motors[1].usCruise);
        } else {
          Serial.print(F("SPD out of range: "));
          Serial.println(newUsCruise);
        }

        // Do NOT start a divert cycle for SPD command
        return;
      }

      // ---- COLOR COMMAND (RED/GREEN/BLUE) ----
      Color = line;     // e.g. "Red", "Green", "Blue"
      newData = true;   // flag a color event
      return;           // process one command per loop()
    } else if (c != '\r') {
      incomingColor += c;  // build token
    }
  }
}


//===========================================================
// SETUP
//===========================================================
void setup() {
  Serial.begin(9600); // must match PC sender baud


/*
  // LEDs
  pinMode(RED_LED_PIN,   OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN,  OUTPUT);
  setColorLED(false, false, false);*/

  // Servos
  ARM1.attach(SERVO1_PIN);
  ARM2.attach(SERVO2_PIN);
  ARM1.write(SERVO1_NEUTRAL);
  ARM2.write(SERVO2_NEUTRAL);

  // Build motors
  for (size_t i = 0; i < N_MOTORS; ++i) {
    motors[i].dirPin    = MOTOR_CFGS[i].dirPin;
    motors[i].stepPin   = MOTOR_CFGS[i].stepPin;
    motors[i].enPin     = MOTOR_CFGS[i].enPin;
    motors[i].usStart   = MOTOR_CFGS[i].usStart;
    motors[i].usCruise  = MOTOR_CFGS[i].usCruise;
    motors[i].rampSteps = MOTOR_CFGS[i].rampSteps;
    motors[i].begin(MOTOR_CFGS[i].dirHigh);
    motors[i].startWithRamp();
  }

  // Start conveyor (M0) only
 // motors[0].startWithRamp();

  Serial.println("Arduino sorter ready. Send: RED|GREEN|BLUE + \\n");
}

//===========================================================
// LOOP
//===========================================================
void loop() {
  // Keep steppers alive
  for (size_t i = 0; i < N_MOTORS; ++i) motors[i].service();

  // Serial input
  receiveColorCommand();
  tMillis = millis();

  // Watchdog: if vision silent too long, ensure conveyor runs
  if ((tMillis - lastVisionAt) > visionTimeout && state == IDLE_RUN) {
    if (!motors[0].enabled) {
      motors[0].resume(true);
      Serial.println(F("Watchdog: conveyor resumed."));
    }
  }

  // On new color event, start divert sequence
  if (newData && state == IDLE_RUN) {
    newData = false;
    lastVisionAt = tMillis;

    // Pause conveyor, hold torque
    motors[0].stop(true);
    phaseStart = tMillis;
    state = DIVERT_START;

    Serial.print(F("Event: "));
    Serial.println(Color);
  } else if (newData) {
    // If an event arrives mid-cycle, stash/override for next cycle:
    // We'll just update Color so the next IDLE_RUN handles it.
    newData = false;
    Serial.println(F("Info: color received during cycle; queued."));
  }

  // ---- State machine ----
  switch (state) {
    case IDLE_RUN:
      // Conveyor runs; do nothing special here
      break;

    case DIVERT_START:
      if (tMillis - phaseStart >= settleBeforeDivert) {
        // Position diverter & LEDs per color
        if (Color.equalsIgnoreCase("Red")) {
          //setColorLED(true, false, false);
          ARM1.write(SERVO1_RED_POS);
          ARM2.write(SERVO2_RED_POS);
        } else if (Color.equalsIgnoreCase("Green")) {
          //setColorLED(false, true, false);
          ARM1.write(SERVO1_GREEN_POS);
          ARM2.write(SERVO2_GREEN_POS);
        } else if (Color.equalsIgnoreCase("Blue")) {
          //setColorLED(false, false, true);
          ARM1.write(SERVO1_BLUE_POS);
          ARM2.write(SERVO2_BLUE_POS);
        } else {
          // Unknown -> just resume conveyor and exit
          motors[0].resume(true);
          //setColorLED(false, false, false);
          state = IDLE_RUN;
          break;
        }
        phaseStart = tMillis;
        state = DIVERT_WAIT;
      }
      break;

    case DIVERT_WAIT:
      if (tMillis - phaseStart >= divertSwingTime) {
        // Resume conveyor to carry brick into bin
        motors[0].resume(true);
        phaseStart = tMillis;
        state = DIVERT_RESET;
      }
      break;

    case DIVERT_RESET:
      if (tMillis - phaseStart >= resetSwingTime) {
        // Return diverter to neutral
        ARM1.write(SERVO1_NEUTRAL);
        ARM2.write(SERVO2_NEUTRAL);
        //setColorLED(false, false, false);

        // === NON-BLOCKING 4-SECOND PAUSE ===
        // Stop conveyor (hold torque), start pause timer.
        motors[0].stop(true);
        phaseStart = tMillis;              // reuse as pause start
        state = DIVERT_PAUSE;
        Serial.println(F("Non-blocking pause: 4s..."));
      }
      break;

    case DIVERT_PAUSE:
      if (tMillis - phaseStart >= pauseDuration) {
        // Pause elapsed: resume conveyor and go idle
        motors[0].resume(true);
        Serial.println(F("Pause done. Conveyor resumed."));
        state = IDLE_RUN;
      }
      break;
  }
}