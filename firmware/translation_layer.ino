#include <Servo.h>

// ---------------- PINS ----------------
const int X_DIR = 2, X_STEP = 3;
const int Y_DIR = 8, Y_STEP = 9;
const int SERVO_PIN = 7;

// ------------- GRID -------------
// Columns (moving right from home): y=0, h=1, g=2, f=3, e=4, d=5, c=6, b=7, a=8, x=9
// Ranks (moving up from home):      9=0 (home), 8, 7 ... 0
// Chessboard = columns h..a (1..8) x ranks 1..8
// Parking    = column y, column x, rank 0, rank 9
// Home       = y9 (column 0, rank 9) = origin (0,0)

// ------------- CALIBRATION -------------
const float STEPS_PER_CELL_X = 93.46;      // one column step, 3.4 cm
const float STEPS_PER_CELL_Y = 168.26;     // one rank step,   3.4 cm

const bool X_DIR_INVERT = false;           // +X = moving right from home
const bool Y_DIR_INVERT = false;           // +Y = moving up from home (rank 9 -> 0)

// ------------- SPEED -------------
const int CRUISE_DELAY_US = 1000;          // lower = faster
const int START_DELAY_US  = 2000;          // gentle start/stop speed
const int RAMP_STEPS      = 150;           // 0 = constant speed, no ramp

// ------------- SERVO -------------
const int SERVO_DOWN_ANGLE = 180;          // rack down (magnet clear of piece)
const int SERVO_UP_ANGLE   = 80;           // rack up (magnet gripping piece)
const int SWEEP_DELAY_MS   = 15;           // per degree. Higher = slower
const int SERVO_SETTLE_MS  = 300;          // pause after the servo stops

// ------------- PAUSE AT THE END OF A PATH -------------
const int END_PAUSE_MS = 2000;             // wait after rack goes down, before returning

// ------------- STATE -------------
Servo probe;
int servoPos = SERVO_DOWN_ANGLE;
long curX = 0, curY = 0;                   // steps from home y9 (y9 = 0,0)

const int MAX_CELLS = 40;
int cellCol[MAX_CELLS];                    // 0-9 (y=0 ... x=9)
int cellRank[MAX_CELLS];                   // 0-9
int cellActive[MAX_CELLS];                 // 1 = rack up after this cell, 0 = down

// ---------- servo ----------
void servoSweepTo(int target) {
  if (servoPos == target) return;
  while (servoPos != target) {
    servoPos += (target > servoPos) ? 1 : -1;
    probe.write(servoPos);
    delay(SWEEP_DELAY_MS);
  }
  delay(SERVO_SETTLE_MS);
}

void setRack(int active) {
  servoSweepTo(active ? SERVO_UP_ANGLE : SERVO_DOWN_ANGLE);
}

// ---------- motion ----------
int stepDelayFor(long i, long total) {
  long ramp = RAMP_STEPS;
  if (ramp > total / 2) ramp = total / 2;
  long pos = i;
  if (total - 1 - i < pos) pos = total - 1 - i;
  if (ramp == 0 || pos >= ramp) return CRUISE_DELAY_US;
  return START_DELAY_US - (long)(START_DELAY_US - CRUISE_DELAY_US) * pos / ramp;
}

void moveAxis(int dirPin, int stepPin, long steps, bool invert) {
  if (steps == 0) return;
  bool positive = steps > 0;
  digitalWrite(dirPin, (positive != invert) ? HIGH : LOW);
  delayMicroseconds(50);
  steps = labs(steps);
  for (long i = 0; i < steps; i++) {
    int d = stepDelayFor(i, steps);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(d);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(d);
  }
}

// Absolute move in steps from home y9. X first, then Y.
void goToSteps(long tx, long ty) {
  moveAxis(X_DIR, X_STEP, tx - curX, X_DIR_INVERT);
  delay(100);
  moveAxis(Y_DIR, Y_STEP, ty - curY, Y_DIR_INVERT);
  curX = tx;
  curY = ty;
}

void goToCell(int col, int rank) {
  long tx = lround(col * STEPS_PER_CELL_X);        // y=0 ... x=9
  long ty = lround((9 - rank) * STEPS_PER_CELL_Y); // 9=0 (home) ... 0=9
  goToSteps(tx, ty);
}

// ---------- input helpers ----------
String readLine() {
  while (true) {
    while (!Serial.available()) {}
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() > 0) return s;
  }
}

// Valid file letters: a-h, x, y. Valid ranks: 0-9.
bool validFile(char f) {
  return (f >= 'a' && f <= 'h') || f == 'x' || f == 'y';
}

// file letter -> column index: y=0, h=1, g=2 ... a=8, x=9
int fileToCol(char f) {
  if (f == 'y') return 0;
  if (f == 'x') return 9;
  return 'h' - f + 1;
}

// column index -> file letter
char colToFile(int c) {
  if (c == 0) return 'y';
  if (c == 9) return 'x';
  return (char)('h' - c + 1);
}

// withFlag = false: tokens like "g6" or "x3"      (start cell)
// withFlag = true : tokens like "g51" or "x30"     (cell + rack state 0/1)
// Returns number of cells, or -1 if invalid
int parseCells(String s, bool withFlag) {
  s.toLowerCase();
  const char *seps = " ,;{}[]()\t\r";
  for (unsigned i = 0; i < s.length(); i++) {
    if (strchr(seps, s[i])) s.setCharAt(i, ' ');
  }
  int expectedLen = withFlag ? 3 : 2;
  int count = 0, i = 0, len = s.length();
  while (i < len) {
    while (i < len && s[i] == ' ') i++;
    if (i >= len) break;
    int j = i;
    while (j < len && s[j] != ' ') j++;
    String t = s.substring(i, j);
    i = j;

    bool ok = (t.length() == expectedLen) &&
              validFile(t[0]) && t[1] >= '0' && t[1] <= '9';
    if (ok && withFlag) ok = (t[2] == '0' || t[2] == '1');
    if (!ok) {
      Serial.print(F("Bad entry: ")); Serial.println(t);
      if (withFlag) Serial.println(F("Each entry must be cell + 0/1, e.g. g51 or x30"));
      return -1;
    }
    if (count >= MAX_CELLS) { Serial.println(F("Too many cells")); return -1; }
    cellCol[count]    = fileToCol(t[0]);
    cellRank[count]   = t[1] - '0';
    cellActive[count] = withFlag ? (t[2] - '0') : 1;
    count++;
  }
  return count;
}

// Every leg of the path must run along a single column or rank.
bool pathIsStraightLegs(int n) {
  for (int k = 1; k < n; k++) {
    if (cellCol[k] != cellCol[k-1] && cellRank[k] != cellRank[k-1]) {
      Serial.print(F("Leg ")); Serial.print(k);
      Serial.println(F(" is diagonal, not allowed"));
      return false;
    }
  }
  return true;
}

int sgn(int v) { return (v > 0) - (v < 0); }

void printCell(int col, int rank) {
  Serial.print(colToFile(col));
  Serial.print(rank);
}

void setup() {
  Serial.begin(115200);
  pinMode(X_DIR, OUTPUT);  pinMode(X_STEP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);  pinMode(Y_STEP, OUTPUT);
  probe.attach(SERVO_PIN);
  probe.write(SERVO_DOWN_ANGLE);           // start with the rack down
  delay(500);
  Serial.println(F("Hub must be at home y9 when the board resets."));
}

void loop() {
  // ---------- STEP 1: start cell (where the piece is) ----------
  int sc, sr;
  while (true) {
    Serial.println(F("\nEnter START cell (where the piece is), e.g. g6 or x3:"));
    int n = parseCells(readLine(), false);
    if (n == 1) { sc = cellCol[0]; sr = cellRank[0]; break; }
    Serial.println(F("Please enter exactly one cell."));
  }

  Serial.print(F("Moving from y9 to "));
  printCell(sc, sr);
  Serial.println();
  goToCell(sc, sr);
  Serial.println(F("Arrived at start. Raising rack."));
  setRack(1);                              // attach the piece

  // ---------- STEP 2: path array (start cell may be left out) ----------
  int n;
  while (true) {
    Serial.println(F("Enter PATH as cell+0/1, e.g. {g51, g41, x40, y90}:"));
    n = parseCells(readLine(), true);
    if (n <= 0) { Serial.println(F("Try again.")); continue; }

    // If the first cell is not the start cell, add it in front (rack up)
    if (cellCol[0] != sc || cellRank[0] != sr) {
      if (n >= MAX_CELLS) { Serial.println(F("Too many cells")); continue; }
      for (int k = n; k > 0; k--) {
        cellCol[k]    = cellCol[k-1];
        cellRank[k]   = cellRank[k-1];
        cellActive[k] = cellActive[k-1];
      }
      cellCol[0]    = sc;
      cellRank[0]   = sr;
      cellActive[0] = 1;
      n++;
    }

    if (n < 2) { Serial.println(F("Path needs at least one cell besides the start.")); continue; }
    if (!pathIsStraightLegs(n)) continue;
    break;
  }

  // Rack state for the first leg (start cell's flag; 1 if it was left out)
  setRack(cellActive[0]);

  // Follow the path. A cell in the middle of a straight run is passed
  // through without stopping, unless the rack state changes there.
  for (int k = 1; k < n; k++) {
    int dx = cellCol[k] - cellCol[k-1];
    int dy = cellRank[k] - cellRank[k-1];
    bool last = (k == n - 1);

    if (dx == 0 && dy == 0) {              // repeated cell: only update rack
      if (!last) setRack(cellActive[k]);
      continue;
    }
    if (!last) {
      int nx = cellCol[k+1] - cellCol[k];
      int ny = cellRank[k+1] - cellRank[k];
      bool sameDir = (sgn(nx) == sgn(dx) && sgn(ny) == sgn(dy));
      bool sameRack = (cellActive[k] == cellActive[k-1]);
      if (sameDir && sameRack) continue;   // no need to stop here
    }

    Serial.print(F("-> "));
    printCell(cellCol[k], cellRank[k]);
    Serial.print(F("  rack after: "));
    Serial.println(last ? 0 : cellActive[k]);
    goToCell(cellCol[k], cellRank[k]);
    if (!last) setRack(cellActive[k]);     // state held until the next cell
  }

  // ---------- STEP 3: rack down, pause, then home ----------
  Serial.println(F("Path complete. Lowering rack."));
  setRack(0);
  delay(END_PAUSE_MS);
  Serial.println(F("Returning to home y9"));
  goToSteps(0, 0);
  Serial.println(F("Done, at y9."));
}