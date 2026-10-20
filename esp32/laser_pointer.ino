#include <ESP32Servo.h>

// Create servo objects
Servo panServo;
Servo tiltServo;

// ESP32 GPIO pins
const int PAN_PIN = 18;
const int TILT_PIN = 19;

// Default/reset position
const int START_PAN = 92;
const int START_TILT = 60;

// Servo angle limits
const int PAN_MIN = 0;
const int PAN_MAX = 180;

const int TILT_MIN = 0;
const int TILT_MAX = 180;

void setup() {
  // Start serial communication with Python
  Serial.begin(115200);

  // Configure servo PWM frequency
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);

  // Attach servos using pulse-width limits
  panServo.attach(PAN_PIN, 500, 2500);
  tiltServo.attach(TILT_PIN, 500, 2500);

  // Move to starting position
  panServo.write(START_PAN);
  tiltServo.write(START_TILT);

  delay(1000);

  // Notify Python that the ESP32 is ready
  Serial.println("READY");
  Serial.println("START PAN=92 TILT=60");
}

void loop() {
  // Wait until a serial command is received
  if (!Serial.available()) {
    return;
  }

  // Read one command until newline
  String command = Serial.readStringUntil('\n');
  command.trim();

  // Movement command format: M,PAN,TILT
  if (command.startsWith("M,")) {
    int comma1 = command.indexOf(',');
    int comma2 = command.indexOf(',', comma1 + 1);

    if (comma2 > 0) {
      int panAngle = command.substring(comma1 + 1, comma2).toInt();
      int tiltAngle = command.substring(comma2 + 1).toInt();

      // Keep angles within permitted limits
      panAngle = constrain(panAngle, PAN_MIN, PAN_MAX);
      tiltAngle = constrain(tiltAngle, TILT_MIN, TILT_MAX);

      // Move both servos
      panServo.write(panAngle);
      tiltServo.write(tiltAngle);

      // Send confirmation to Python
      Serial.print("MOVED PAN=");
      Serial.print(panAngle);
      Serial.print(" TILT=");
      Serial.println(tiltAngle);
    }
  }
  // Reset command
  else if (command == "RESET") {
    panServo.write(START_PAN);
    tiltServo.write(START_TILT);

    Serial.println("RESET PAN=92 TILT=60");
  }
}
