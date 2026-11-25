// ---------- AXIS TEST ----------
// Type a command in the Serial Monitor (115200 baud, "Newline" ending):
//   x500   -> move X axis 500 steps forward
//   x-500  -> move X axis 500 steps backward
//   y300   -> move Y axis 300 steps forward
//   y-300  -> move Y axis 300 steps backward
//   s      -> stop (does nothing, just reminds you of the commands)

// ---------------- PINS ----------------
const int X_DIR = 2, X_STEP = 3;
const int Y_DIR = 8, Y_STEP = 9;

// ---------------- DIRECTION ----------------
// Flip these to true if an axis moves the wrong way
const bool X_DIR_INVERT = false;
const bool Y_DIR_INVERT = true;

// ---------------- SPEED ----------------
// Smaller delay = faster movement. Try 1000 to start, go lower to speed up.
const int STEP_DELAY_US = 1000;

void moveAxis(int dirPin, int stepPin, long steps, bool invert) {
  if (steps == 0) return;
  digitalWrite(dirPin, ((steps > 0) != invert) ? HIGH : LOW);
  delayMicroseconds(50);
  steps = labs(steps);
  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(X_DIR, OUTPUT);  pinMode(X_STEP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);  pinMode(Y_STEP, OUTPUT);
  Serial.println(F("Axis test ready. Commands: x500, x-500, y300, y-300"));
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() < 2) return;

  char axis = cmd[0];
  long steps = cmd.substring(1).toInt();

  if (axis == 'x') {
    Serial.print(F("Moving X by ")); Serial.println(steps);
    moveAxis(X_DIR, X_STEP, steps, X_DIR_INVERT);
  } else if (axis == 'y') {
    Serial.print(F("Moving Y by ")); Serial.println(steps);
    moveAxis(Y_DIR, Y_STEP, steps, Y_DIR_INVERT);
  } else {
    Serial.println(F("Unknown command. Use x<steps> or y<steps>"));
    return;
  }
  Serial.println(F("Done."));
}