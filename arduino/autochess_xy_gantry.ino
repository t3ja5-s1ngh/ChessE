/*
  AutoChess - simple XY gantry controller for Arduino Uno + 2 x A4988.

  Laptop sends a line such as:
      PLAY 4 1 4 3
  This means: move from e2 (file 4, rank 1) to e4 (file 4, rank 3).

  Board coordinate convention:
      a1 = (0, 0), h8 = (7, 7)
      X increases from file a toward h.
      Y increases from rank 1 toward 8.

  IMPORTANT:
  - Place the probe at the CENTER of a1 before resetting/powering this sketch.
  - Calibrate STEPS_PER_SQUARE_X and STEPS_PER_SQUARE_Y for your mechanics.
  - Magnet output drives only a suitable MOSFET/driver input, NEVER the coil directly.
  - This is a basic prototype controller, not a safety-rated motion controller.
*/

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <stdio.h>

// A4988 STEP/DIR pin assignments (Arduino Uno).
const uint8_t X_STEP_PIN = 2;
const uint8_t X_DIR_PIN  = 5;
const uint8_t Y_STEP_PIN = 3;
const uint8_t Y_DIR_PIN  = 6;
const uint8_t ENABLE_PIN = 8;   // A4988 ENABLE is active LOW.

// Electromagnet MOSFET-driver input. HIGH = magnet ON in this example.
const uint8_t MAGNET_PIN = 11;

// -------------------- CALIBRATION --------------------
// PLACEHOLDER VALUES ONLY. Measure these for your actual belt/lead-screw,
// pulley, motor step angle and A4988 microstep setting.
const long STEPS_PER_SQUARE_X = 800;
const long STEPS_PER_SQUARE_Y = 800;

// Adjust these if a positive file/rank movement runs in the wrong direction.
const bool X_DIR_INVERTED = false;
const bool Y_DIR_INVERTED = false;

// Start with a conservative speed. This version uses coordinated constant-
// speed movement through MultiStepper; it does not implement acceleration.
const float MAX_SPEED_STEPS_PER_SECOND = 500.0;

const unsigned long MAGNET_SETTLE_MS = 250;
const unsigned long RELEASE_SETTLE_MS = 150;
// -----------------------------------------------------

AccelStepper xMotor(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper yMotor(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
MultiStepper xyMotors;

char inputLine[64];
uint8_t inputLength = 0;
bool droppingLongLine = false;

void moveToSquare(int fileIndex, int rankIndex) {
  long targetPositions[2];
  targetPositions[0] = (long)fileIndex * STEPS_PER_SQUARE_X;
  targetPositions[1] = (long)rankIndex * STEPS_PER_SQUARE_Y;

  xyMotors.moveTo(targetPositions);
  xyMotors.runSpeedToPosition();
}

void processCommand(char *line) {
  int sourceFile, sourceRank, destinationFile, destinationRank;

  if (sscanf(line, "PLAY %d %d %d %d",
             &sourceFile, &sourceRank,
             &destinationFile, &destinationRank) != 4) {
    Serial.println("ERR expected: PLAY sourceFile sourceRank destFile destRank");
    return;
  }

  if (sourceFile < 0 || sourceFile > 7 || sourceRank < 0 || sourceRank > 7 ||
      destinationFile < 0 || destinationFile > 7 || destinationRank < 0 || destinationRank > 7) {
    Serial.println("ERR square coordinates must be between 0 and 7");
    return;
  }

  if (sourceFile == destinationFile && sourceRank == destinationRank) {
    Serial.println("ERR source and destination are the same square");
    return;
  }

  // Always ensure the magnet is off while travelling to the source square.
  digitalWrite(MAGNET_PIN, LOW);
  Serial.println("MOVING_TO_SOURCE");
  moveToSquare(sourceFile, sourceRank);

  Serial.println("MAGNET_ON");
  digitalWrite(MAGNET_PIN, HIGH);
  delay(MAGNET_SETTLE_MS);

  Serial.println("MOVING_TO_DESTINATION");
  moveToSquare(destinationFile, destinationRank);

  Serial.println("MAGNET_OFF");
  digitalWrite(MAGNET_PIN, LOW);
  delay(RELEASE_SETTLE_MS);

  Serial.println("OK");
}

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable both A4988 drivers.

  pinMode(MAGNET_PIN, OUTPUT);
  digitalWrite(MAGNET_PIN, LOW); // Magnet starts OFF.

  xMotor.setPinsInverted(X_DIR_INVERTED, false, false);
  yMotor.setPinsInverted(Y_DIR_INVERTED, false, false);
  xMotor.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND);
  yMotor.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND);
  xMotor.setMinPulseWidth(2);
  yMotor.setMinPulseWidth(2);

  xyMotors.addStepper(xMotor);
  xyMotors.addStepper(yMotor);

  Serial.begin(115200);
  Serial.println("READY - AutoChess XY controller");
}

void loop() {
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      if (!droppingLongLine) {
        inputLine[inputLength] = '\0';
        if (inputLength > 0) {
          processCommand(inputLine);
        }
      } else {
        Serial.println("ERR command line too long");
      }
      inputLength = 0;
      droppingLongLine = false;
      continue;
    }

    if (!droppingLongLine) {
      if (inputLength < sizeof(inputLine) - 1) {
        inputLine[inputLength++] = incoming;
      } else {
        droppingLongLine = true;
      }
    }
  }
}
