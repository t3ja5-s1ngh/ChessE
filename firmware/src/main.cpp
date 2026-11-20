#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

// ============================================================
// ChessElab Motion Controller
// XY Gantry + Servo-Actuated Magnetic Probe
// ============================================================

// -------------------- XY STEPPER PINS ------------------------

#define X_STEP_PIN 3
#define X_DIR_PIN  2

#define Y_STEP_PIN 9
#define Y_DIR_PIN  8

// -------------------- SERVO PIN ------------------------------

#define PROBE_SERVO_PIN 7

// -------------------- STEPPER CONFIGURATION ------------------

#define MOTOR_INTERFACE_TYPE AccelStepper::DRIVER

AccelStepper motorX(MOTOR_INTERFACE_TYPE, X_STEP_PIN, X_DIR_PIN);
AccelStepper motorY(MOTOR_INTERFACE_TYPE, Y_STEP_PIN, Y_DIR_PIN);

// -------------------- SERVO CONFIGURATION -------------------

Servo probeServo;

// IMPORTANT: Calibrate these angles for your mechanism.
const int PROBE_UP_ANGLE =20;
const int PROBE_DOWN_ANGLE = 100;

// Servo settling time
const unsigned long SERVO_SETTLE_TIME = 500;

// -------------------- CALIBRATION ----------------------------

const float X_STEPS_PER_MM = 0.36567164179;
const float Y_STEPS_PER_MM = 0.20310880829;

const float MAX_SPEED_X = 2000.0;
const float MAX_SPEED_Y = 2000.0;

const float ACCEL_X = 1000.0;
const float ACCEL_Y = 1000.0;

// -------------------- SYSTEM STATE ---------------------------

float currentX = 0.0;
float currentY = 0.0;

float targetX = 0.0;
float targetY = 0.0;

bool motionActive = false;
bool probeIsDown = false;

// ============================================================
// MOTOR SETUP
// ============================================================

void configureMotors()
{
    motorX.setMaxSpeed(MAX_SPEED_X);
    motorX.setAcceleration(ACCEL_X);

    motorY.setMaxSpeed(MAX_SPEED_Y);
    motorY.setAcceleration(ACCEL_Y);

    motorX.setCurrentPosition(0);
    motorY.setCurrentPosition(0);
}

// ============================================================
// PROBE SETUP
// ============================================================

void configureProbe()
{
    probeServo.attach(PROBE_SERVO_PIN);

    // Start with probe raised
    probeServo.write(PROBE_UP_ANGLE);

    delay(SERVO_SETTLE_TIME);

    probeIsDown = false;
}

// ============================================================
// COORDINATE CONVERSION
// ============================================================

long mmToStepsX(float mm)
{
    return (long)(mm * X_STEPS_PER_MM);
}

long mmToStepsY(float mm)
{
    return (long)(mm * Y_STEPS_PER_MM);
}

float stepsToMmX(long steps)
{
    return (float)steps / X_STEPS_PER_MM;
}

float stepsToMmY(long steps)
{
    return (float)steps / Y_STEPS_PER_MM;
}

// ============================================================
// XY MOVEMENT
// ============================================================

void moveXY(float x, float y)
{
    targetX = x;
    targetY = y;

    motorX.moveTo(mmToStepsX(x));
    motorY.moveTo(mmToStepsY(y));

    motionActive = true;

    Serial.print("MOVING X:");
    Serial.print(x);

    Serial.print(" Y:");
    Serial.println(y);
}

// ============================================================
// PROBE CONTROL
// ============================================================

void probeDown()
{
    Serial.println("INFO: Lowering probe");

    probeServo.write(PROBE_DOWN_ANGLE);

    delay(SERVO_SETTLE_TIME);

    probeIsDown = true;

    Serial.println("INFO: Probe lowered");
}

void probeUp()
{
    Serial.println("INFO: Raising probe");

    probeServo.write(PROBE_UP_ANGLE);

    delay(SERVO_SETTLE_TIME);

    probeIsDown = false;

    Serial.println("INFO: Probe raised");
}

// ============================================================
// HOMING
// ============================================================

void homeAxes()
{
    Serial.println("ERROR: Homing not implemented yet");

    // Later:
    // 1. Move X toward its limit switch
    // 2. Detect switch
    // 3. Set X = 0
    // 4. Move Y toward its limit switch
    // 5. Detect switch
    // 6. Set Y = 0
}

// ============================================================
// POSITION REPORT
// ============================================================

void reportPosition()
{
    Serial.print("X:");
    Serial.print(stepsToMmX(motorX.currentPosition()), 2);

    Serial.print(" Y:");
    Serial.print(stepsToMmY(motorY.currentPosition()), 2);

    Serial.print(" MOVING:");
    Serial.print(motionActive ? "YES" : "NO");

    Serial.print(" PROBE:");
    Serial.println(probeIsDown ? "DOWN" : "UP");
}

// ============================================================
// EMERGENCY STOP
// ============================================================

void emergencyStop()
{
    motorX.stop();
    motorY.stop();

    motionActive = false;

    probeUp();

    Serial.println("ERROR: EMERGENCY STOP");
}

// ============================================================
// COMMAND PARSER
// ============================================================

bool extractValue(String command, char axis, float &value)
{
    int index = command.indexOf(axis);

    if (index == -1)
        return false;

    int start = index + 1;
    int end = start;

    while (end < command.length())
    {
        char c = command.charAt(end);

        if (c == ' ' || c == '\t')
            break;

        end++;
    }

    String valueString = command.substring(start, end);

    value = valueString.toFloat();

    return true;
}

// ============================================================
// COMMAND EXECUTION
// ============================================================

void processCommand(String command)
{
    command.trim();

    if (command.length() == 0)
        return;

    Serial.print("CMD: ");
    Serial.println(command);

    // Emergency stop can be processed at any time
    if (command == "M112")
    {
        emergencyStop();
        Serial.println("OK");
        return;
    }

    // Reject other commands while the XY gantry is moving
    if (motionActive)
    {
        Serial.println("ERROR:BUSY");
        return;
    }

    // --------------------------------------------------------
    // G0 - Move XY
    // Example: G0 X100 Y150
    // --------------------------------------------------------

    if (command.startsWith("G0"))
    {
        float x = currentX;
        float y = currentY;

        extractValue(command, 'X', x);
        extractValue(command, 'Y', y);

        moveXY(x, y);
    }

    // --------------------------------------------------------
    // G28 - Home
    // --------------------------------------------------------

    else if (command == "G28")
    {
        homeAxes();
    }

    // --------------------------------------------------------
    // M10 - Probe DOWN
    // --------------------------------------------------------

    else if (command == "M10")
    {
        probeDown();

        Serial.println("OK");
    }

    // --------------------------------------------------------
    // M11 - Probe UP
    // --------------------------------------------------------

    else if (command == "M11")
    {
        probeUp();

        Serial.println("OK");
    }

    // --------------------------------------------------------
    // M114 - Position report
    // --------------------------------------------------------

    else if (command == "M114")
    {
        reportPosition();

        Serial.println("OK");
    }

    // --------------------------------------------------------
    // M400 - Motion status
    // --------------------------------------------------------

    else if (command == "M400")
    {
        if (motionActive)
            Serial.println("ERROR:BUSY");
        else
            Serial.println("OK");
    }

    // --------------------------------------------------------
    // Unknown command
    // --------------------------------------------------------

    else
    {
        Serial.println("ERROR:UNKNOWN_COMMAND");
    }
}

// ============================================================
// SERIAL INPUT
// ============================================================

void readSerial()
{
    if (Serial.available() <= 0)
        return;

    String command = Serial.readStringUntil('\n');

    processCommand(command);
}

// ============================================================
// MOTION UPDATE
// ============================================================

void updateMotion()
{
    if (!motionActive)
        return;

    motorX.run();
    motorY.run();

    if (motorX.distanceToGo() == 0 &&
        motorY.distanceToGo() == 0)
    {
        motionActive = false;

        currentX = stepsToMmX(motorX.currentPosition());
        currentY = stepsToMmY(motorY.currentPosition());

        Serial.println("DONE");
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    configureMotors();
    configureProbe();

    delay(500);

    Serial.println();
    Serial.println("==================================");
    Serial.println(" ChessElab Motion Controller");
    Serial.println(" XY: AccelStepper");
    Serial.println(" Probe: Servo");
    Serial.println("==================================");

    Serial.println("G0 X... Y... - XY movement");
    Serial.println("G28           - Home");
    Serial.println("M10           - Probe down");
    Serial.println("M11           - Probe up");
    Serial.println("M114          - Position report");
    Serial.println("M112          - Emergency stop");

    Serial.println("READY");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    readSerial();

    updateMotion();
}
