// --- Arduino Color Command Receiver ---
// Works with Python sending: "Red\n", "Blue\n", or "Green\n"
// Only reacts when the detected color changes (prevents spam)

String incomingColor = "";   // buffer for received color
String lastColor = "";       // remember last processed color
String Color = "";
bool newData = false;        // flag for new serial data

#include <Servo.h>

Servo ARM;

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

  ARM.attach(10);
  ARM.write(90);  // set servo to mid-point

  Serial.println("Arduino ready to receive color commands...");
}

void loop() {
  receiveColorCommand(); // check for new serial data

  if (newData) {
    newData = false;       // reset flag after reading
    Color.trim();  // remove newline or spaces

    // Process only if the color has changed
    if (Color != lastColor) {
      lastColor = Color;  // update last known color
      Serial.println("LastColor:" + lastColor);


      if (Color.equalsIgnoreCase("Red")) {
        setColor(true, false, false);
        Serial.println("✅ New color detected: RED");
        ARM.write(50);  // set servo to mid-point
      } 
      else if (Color.equalsIgnoreCase("Green")) {
        setColor(false, true, false);
        Serial.println("✅ New color detected: GREEN");
        ARM.write(80);  // set servo to mid-point

      } 
      else if (Color.equalsIgnoreCase("Blue")) {
        setColor(false, false, true);
        Serial.println("✅ New color detected: BLUE");
        ARM.write(20);  // set servo to mid-point
      } 
      else {
        Serial.println("⚠️ Unknown command: " + Color);
      }
    }
  }
}

// --- Function: Read color command from Serial ---
void receiveColorCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {          // End of command
      newData = true;
      Serial.println(newData);
      Color = incomingColor;
      Serial.println("Present" + Color);

      incomingColor = "\0";
      Serial.println(incomingColor);
      return;


    } else {
      incomingColor += c;     // Build the string one char at a time
      Serial.println(incomingColor);

    }
  }

}

// --- Helper: Turn LEDs (or motors) on/off based on color ---
void setColor(bool redState, bool greenState, bool blueState) {
  digitalWrite(redPin, redState);
  digitalWrite(greenPin, greenState);
  digitalWrite(bluePin, blueState);
}
