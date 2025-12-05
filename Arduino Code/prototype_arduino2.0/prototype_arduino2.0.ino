
//===========================================================
// Multi-Stepper Conveyor Controller (A4988/DRV8825)
//===========================================================
// Full-step recommended: set MS1/MS2/MS3 LOW on each driver.

//==============================
//COMMANDS/FUNCTIONS
//------------------
// Stop options:
//------------------
  // - hold=true  : stop stepping but keep coils energized (motor locks)
  // - hold=false : disable driver (motor free-spins)
//------------------
// Resume options:
//------------------
  // - withRAMP=true  : ramps up
  // - withRAMP=false : Straight to Full power

//=================================



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
  {4,    7,    0,    7000,    1500,     300,       false},  // M0
  {6,    5,    1,    6000,    2000,     250,       true }  // M1
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



// --- Arduino Color Command Receiver ---
// Works with Python sending: "Red\n", "Blue\n", or "Green\n"
// Only reacts when the detected color changes (prevents spam)

String incomingColor = "";   // buffer for received color
String lastColor = "";       // remember last processed color
String Color = "";
bool newData = false;        // flag for new serial data


//====================================
//
// Serial Connection Python to Arduino
//
//====================================


#include <Servo.h>

Servo ARM1, ARM2;

// Assign output pins (LEDs or motor control pins)
const int redPin = 8;
const int greenPin = 9;
const int bluePin = 10;

void setup() {
  Serial.begin(9600); // must match Python baud rate
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Start with all LEDs off
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);

//=================
//--Servo Arms-----
//=================

  ARM1.attach(9);
  ARM1.write(90);  //set servo to mid-point
  ARM2.attach(10);
  ARM2.write(90);  // set servo to mid-point


  // Build motor instances from config
  for (size_t i = 0; i < N_MOTORS; ++i) {
    motors[i].dirPin    = MOTOR_CFGS[i].dirPin;
    motors[i].stepPin   = MOTOR_CFGS[i].stepPin;
    motors[i].enPin     = MOTOR_CFGS[i].enPin;
    motors[i].usStart   = MOTOR_CFGS[i].usStart;
    motors[i].usCruise  = MOTOR_CFGS[i].usCruise;
    motors[i].rampSteps = MOTOR_CFGS[i].rampSteps;

    motors[i].begin(MOTOR_CFGS[i].dirHigh);
  }

    motors[0].startWithRamp();   // start conveyor belt

  Serial.println("Arduino ready to receive color commands...");
}

void loop() {
    // Service all motors (non-blocking). Call as often as possible.
  for (size_t i = 0; i < N_MOTORS; ++i) {
    motors[i].service();
  }


  receiveColorCommand(); // check for new serial data

  if(newData == false){

    motors[1].stop(true);             // stop M0, hold torque
    Serial.println("M0 paused (holding).");
  }

  if (newData) {
    motors[0].stop(true);             // stop M0, hold torque
    Serial.println("M0 paused (holding).");
    newData = false;       // reset flag after reading
    Color.trim();  // remove newline or spaces

    // Process only if the color has changed
    //if (Color != lastColor) {
      //lastColor = Color;  // update last known color
      //Serial.println("LastColor:" + lastColor);

/*
      if (Color.equalsIgnoreCase("Red")) {

        setColor(true, false, false);
        Serial.println("✅ New color detected: RED");
        ARM1.write(40);
        ARM2.write(180);

        delay(2000);
        motors[1].resume(true);           // smooth ramp resume
        Serial.println("M1 resumed with ramp.");

        motors[0].resume(false);          // instant resume at cruise
        Serial.println("M0 resumed instantly.");

        delay(10000);
        Serial.println("Red Brick is in Bin");

      } 
      else if (Color.equalsIgnoreCase("Green")) {
        motors[0].resume(false);          // instant resume at cruise
        Serial.println("M0 resumed instantly.");
        delay(2000);
        motors[0].stop(true);             // stop M0, hold torque
        Serial.println("M0 paused (holding).");

        setColor(false, true, false);
        Serial.println("✅ New color detected: GREEN");
        ARM1.write(120);
        ARM2.write(180);

        delay(2000);
        motors[1].resume(true);           // smooth ramp resume
        Serial.println("M1 resumed with ramp.");
        delay(10000);
        Serial.println("Green Brick is in Bin");

      } 
      else if (Color.equalsIgnoreCase("Blue")) {
        motors[0].resume(false);          // instant resume at cruise
        Serial.println("M0 resumed instantly.");
        delay(2000);
        motors[0].stop(true);             // stop M0, hold torque
        Serial.println("M0 paused (holding).");

        setColor(false, false, true);
        Serial.println("✅ New color detected: BLUE");
        ARM2.write(40);  
        ARM1.write(180);

        delay(2000);
        motors[1].resume(true);           // smooth ramp resume
        Serial.println("M1 resumed with ramp.");
        delay(10000);
        Serial.println("Blue Brick is in Bin");
      } 
      else {
        Serial.println("⚠️ Unknown command: " + Color);
      }*/
    }

}

// --- Function: Read color command from Serial ---
void receiveColorCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {          // End of command
      newData = true;
      //Serial.println(newData);
      Color = incomingColor;
      Serial.println("Present" + Color);

      incomingColor = "\0";
      Serial.println(incomingColor);
      return;


    } else {
      incomingColor += c;     // Build the string one char at a time
      //Serial.println(incomingColor);

    }
  }

}

// --- Helper: Turn LEDs (or motors) on/off based on color ---
void setColor(bool redState, bool greenState, bool blueState) {
  digitalWrite(redPin, redState);
  digitalWrite(greenPin, greenState);
  digitalWrite(bluePin, blueState);
}
