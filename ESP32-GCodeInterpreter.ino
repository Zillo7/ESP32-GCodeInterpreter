#include <ESP32Servo.h>
#include <string>
#include <cctype>
#include <iostream>
#include <sstream>
#include <vector>


///Machine name - for instrument ID///
std::string MachineName = "Harmonica";

char AxesNames[6] = { 'X', 'Y', 'Z', 'A', 'B', 'C' };

///Stepper Motors///

int stepPins[6] = { 13, -1, -1, -1, -1, -1 };  // X..C
int dirPins[6] = { 12, -1, -1, -1, -1, -1 };
int homingPins[6] = { 27, -1, -1, -1, -1, -1 };
int eStopPins[6] = { 14, -1, -1, -1, -1, -1 };
int enablePins[6] = { 14, 14, 14, 14, 14, 14 };
int speeds[6] = { 3200, 600, 600, 600, 600, 600 };
int homeDir[6] = { 0, 1, 1, 1, 1, 1 };
int homeSpeed[6] = { 100, 50, 50, 50, 50, 50 };
int distToMove[6] = { 1000, 2000, 2000, 2000, 2000, 2000 };
int homePullOffSteps[6] = { 200, 200, 200, 200, 200, 200 };
int homeSlowSpeed[6] = { 25, 25, 25, 25, 25, 25 };

///End Stepper Motors///

///Servos///

Servo servos[6];
int servoPins[6] = { -1, -1, -1, -1, -1, -1 };
int servoPositions[6] = { 0, 0, 0, 0, 0, 0 };

///End Servos///

///Solenoids///

int solenoidPins[10] = { 25, 26, -1, -1, -1, -1, -1, -1, -1, -1 };
bool solenoidsOn[10] = { false, false, false, false, false, false, false, false, false, false };

///End Solenoids///

//pin types
int pinModes[99];


/// LEDC-based stepper control (no acceleration) ///
#include <Arduino.h>

// One LEDC channel per motor (0..5)
const int ledcResolution = 8;
// Per-motor pins (kept from original)
// Per-motor speeds in steps/sec
// Motion state
struct MotorState {
  long currentPos = 0;      // steps
  long targetPos = 0;       // steps
  long stepsRemaining = 0;  // steps to go
  bool moving = false;
  bool homing = false;
  int homingPhase = 0;  // 0 none, 1 pull-off, 2 approach
  unsigned long lastMicros = 0;
  double freq = 0.0;   // Hz
  double accum = 0.0;  // fractional step accumulator
  bool dir = true;     // HIGH/LOW on dir pin
  char axisName = '?';
};

MotorState motors[6];

int axisIndexFromChar(char axis) {
  axis = toupper(static_cast<unsigned char>(axis));
  for (int i = 0; i < 6; ++i) {
    if (AxesNames[i] == axis) {
      return i;
    }
  }
  return -1;
}

void ledcStartIfValid(int idx, double freqHz) {
  if (stepPins[idx] > -1) {
    ledcWriteTone(stepPins[idx], (uint32_t)freqHz);
  }
}

void ledcStop(int idx) {
  if (stepPins[idx] > -1) {
    ledcWriteTone(stepPins[idx], 0);
  }
}


void setDir(int idx, bool d) {
  if (dirPins[idx] > -1) digitalWrite(dirPins[idx], d ? HIGH : LOW);
  motors[idx].dir = d;
}

void enableDriver(int idx, bool enable) {
  if (enablePins[idx] > -1) {
    // Original used setEnablePin(..., false) meaning enable is active-LOW
    digitalWrite(enablePins[idx], enable ? LOW : HIGH);
  }
}

void startMoveTo(int idx, long target, int speedSPS) {
  motors[idx].targetPos = target;
  long delta = target - motors[idx].currentPos;
  if (delta == 0) {
    ledcStop(idx);
    motors[idx].moving = false;
    motors[idx].stepsRemaining = 0;
    return;
  }
  bool d = (delta > 0);
  setDir(idx, d);
  motors[idx].stepsRemaining = delta > 0 ? delta : -delta;
  motors[idx].moving = true;
  motors[idx].homing = false;
  motors[idx].homingPhase = 0;
  motors[idx].freq = speedSPS;
  motors[idx].accum = 0.0;
  motors[idx].lastMicros = micros();
  motors[idx].axisName = AxesNames[idx];
  ledcStartIfValid(idx, motors[idx].freq);
}

void startMoveRelative(int idx, long delta, int speedSPS) {
  startMoveTo(idx, motors[idx].currentPos + delta, speedSPS);
}

void startHomingMove(int idx, bool towardHome, long steps, int speed, int phase) {
  if (stepPins[idx] == -1 || steps <= 0 || speed <= 0) {
    return;
  }
  enableDriver(idx, true);
  setDir(idx, towardHome);
  motors[idx].homing = true;
  motors[idx].homingPhase = phase;
  motors[idx].moving = true;
  motors[idx].stepsRemaining = steps;
  motors[idx].freq = speed;
  motors[idx].accum = 0.0;
  motors[idx].lastMicros = micros();
  motors[idx].dir = towardHome;
  motors[idx].targetPos = motors[idx].currentPos + (towardHome ? steps : -steps);
  motors[idx].axisName = AxesNames[idx];
  ledcStartIfValid(idx, motors[idx].freq);
}

bool homingSwitchActive(int idx) {
  if (homingPins[idx] == -1) {
    return false;
  }
  return digitalRead(homingPins[idx]) == HIGH;
}

void handleHomingSwitchTriggered(int idx, int prevPhase);

void beginSlowApproach(int idx) {
  long approachSteps = distToMove[idx];
  if (approachSteps <= 0) {
    approachSteps = 1;
  }
  int speed = homeSlowSpeed[idx] > 0 ? homeSlowSpeed[idx] : 1;
  bool towardHome = homeDir[idx] > 0;

  if (homingSwitchActive(idx)) {
    stopMotorIdx(idx);
    std::string axis(1, motors[idx].axisName);
    WriteLine("Homing failed for axis " + axis + " (switch remained active)");
    return;
  }

  startHomingMove(idx, towardHome, approachSteps, speed, 2);

  if (homingSwitchActive(idx)) {
    // If the switch is already active, immediately treat as a hit.
    int prevPhase = 2;
    stopMotorIdx(idx);
    handleHomingSwitchTriggered(idx, prevPhase);
  }
}

void handleHomingSwitchTriggered(int idx, int prevPhase) {
  bool towardHome = homeDir[idx] > 0;
  bool awayDir = !towardHome;
  if (prevPhase == 0) {
    long pullOff = homePullOffSteps[idx];
    int speed = homeSpeed[idx] > 0 ? homeSpeed[idx] : 1;
    if (pullOff > 0) {
      startHomingMove(idx, awayDir, pullOff, speed, 1);
    } else {
      beginSlowApproach(idx);
    }
    return;
  }

  if (prevPhase == 2) {
    motors[idx].currentPos = 0;
    motors[idx].targetPos = 0;
    motors[idx].homingPhase = 0;
    motors[idx].homing = false;
    std::string axis(1, motors[idx].axisName);
    WriteLine("Homing complete for axis " + axis);
    return;
  }
}

void handleHomingMoveComplete(int idx) {
  if (!motors[idx].homing) {
    stopMotorIdx(idx);
    return;
  }

  int phase = motors[idx].homingPhase;
  if (phase == 1) {
    beginSlowApproach(idx);
    return;
  }

  stopMotorIdx(idx);
  std::string axis(1, motors[idx].axisName);
  WriteLine("Homing failed for axis " + axis + " (switch not detected)");
}

void startHomingSequence(int idx) {
  if (stepPins[idx] == -1) {
    return;
  }
  motors[idx].axisName = AxesNames[idx];
  bool towardHome = homeDir[idx] > 0;
  bool switchActive = homingSwitchActive(idx);

  if (!switchActive) {
    long seekSteps = distToMove[idx];
    if (seekSteps <= 0) {
      seekSteps = 1;
    }
    int fastSpeed = homeSpeed[idx] > 0 ? homeSpeed[idx] : 1;
    startHomingMove(idx, towardHome, seekSteps, fastSpeed, 0);
    return;
  }

  long pullOff = homePullOffSteps[idx];
  int fastSpeed = homeSpeed[idx] > 0 ? homeSpeed[idx] : 1;
  if (pullOff > 0) {
    startHomingMove(idx, !towardHome, pullOff, fastSpeed, 1);
  } else {
    beginSlowApproach(idx);
  }
}

void stopMotorIdx(int idx) {
  motors[idx].moving = false;
  motors[idx].homing = false;
  motors[idx].homingPhase = 0;
  motors[idx].stepsRemaining = 0;
  ledcStop(idx);
}

void serviceMotors() {
  unsigned long now = micros();
  for (int i = 0; i < 6; i++) {
    if (!motors[i].moving) continue;
    if (motors[i].homing && homingPins[i] > -1) {
      int phase = motors[i].homingPhase;
      if ((phase == 0 || phase == 2) && homingSwitchActive(i)) {
        int prevPhase = phase;
        stopMotorIdx(i);
        handleHomingSwitchTriggered(i, prevPhase);
        continue;
      }
    }
    unsigned long dt = now - motors[i].lastMicros;
    motors[i].lastMicros = now;
    double stepsExact = motors[i].freq * (double)dt / 1000000.0 + motors[i].accum;
    long whole = (long)stepsExact;
    motors[i].accum = stepsExact - (double)whole;
    if (whole <= 0) continue;
    if (whole >= motors[i].stepsRemaining) {
      // Finish move
      long stepsDone = motors[i].stepsRemaining;
      motors[i].currentPos += (motors[i].dir ? stepsDone : -stepsDone);
      motors[i].stepsRemaining = 0;
      if (motors[i].homing) {
        handleHomingMoveComplete(i);
      } else {
        stopMotorIdx(i);
      }
    } else {
      motors[i].stepsRemaining -= whole;
      motors[i].currentPos += (motors[i].dir ? whole : -whole);
    }
  }
}

///End Stepper Motors


// ===== G-code input buffering =====
#include <deque>

static const size_t GCODE_MAX_LINE = 256;  // max chars per line (adjust as needed)
static const size_t GCODE_MAX_QUEUE = 32;  // max queued lines

std::string rxBuffer;                // collects characters until newline
std::deque<std::string> gcodeQueue;  // FIFO of complete lines
bool lineOverflow = false;           // flag to drop overly long line safely

inline void enqueueLine(const std::string& s) {
  if (!s.empty()) {
    if (gcodeQueue.size() < GCODE_MAX_QUEUE) {
      gcodeQueue.push_back(s);
    } else {
      // queue full: drop oldest to avoid crash
      gcodeQueue.pop_front();
      gcodeQueue.push_back(s);
    }
  }
}

// Read all available serial bytes and split into lines by '\n'
void readSerialLines() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;  // ignore CR, handle LF only

    if (c == '\n') {
      if (!lineOverflow) {
        // trim whitespace
        size_t start = rxBuffer.find_first_not_of(" \t");
        size_t end = rxBuffer.find_last_not_of(" \t");
        std::string line = (start == std::string::npos) ? "" : rxBuffer.substr(start, end - start + 1);
        enqueueLine(line);
      }
      // reset for next line
      rxBuffer.clear();
      lineOverflow = false;
    } else {
      if (!lineOverflow) {
        if (rxBuffer.size() < GCODE_MAX_LINE - 1) {
          rxBuffer.push_back(c);
        } else {
          // line too long: mark overflow and ignore until newline
          lineOverflow = true;
        }
      }
    }
  }
}

bool allMotorsIdle() {
  for (int i = 0; i < 6; ++i) {
    if (motors[i].moving) return false;
  }
  return true;
}

// Process exactly one queued line if motors are idle (or process regardless if you prefer streaming).
void serviceGcodeQueue() {
  if (!gcodeQueue.empty()) {
    std::string line = gcodeQueue.front();
    gcodeQueue.pop_front();
    if (!line.empty()) {
      ProcessInput(line);
    }
  }
}
// ===== end G-code input buffering =====


std::string inputString = "";
std::vector<std::string> split_string(const std::string& str) {
  std::vector<std::string> words;
  std::stringstream ss(str);
  std::string word;
  while (ss >> word) {
    words.push_back(word);
  }
  return words;
}


void setup() {

  // Configure direction and enable pins
  for (int i = 0; i < 6; i++) {
    if (dirPins[i] > -1) {
      pinMode(dirPins[i], OUTPUT);
      pinModes[i] = 2;
    }
    if (enablePins[i] > -1) {
      pinMode(enablePins[i], OUTPUT);
      pinModes[i] = 2;
      digitalWrite(enablePins[i], HIGH);
    }  // disabled
    if (homingPins[i] > -1) {
      pinMode(homingPins[i], INPUT);
      pinModes[i] = 1;
    }
  }
  // LEDC attach step pins
  for (int i = 0; i < 6; i++) {
    if (stepPins[i] > -1) {
      ledcAttach(stepPins[i], 1000, ledcResolution);
      ledcWriteTone(stepPins[i], 0);
    }
  }


  ///End stepper motor set up///


  ///Set up servo motors///

  for (int i = 0; i < (sizeof(servoPins) / sizeof(servoPins[0])); i++) {
    if (servoPins[i] > -1) { servos[i].attach(servoPins[i]); }
  }
  ///End servo set up///

  ///Set up solenoids///

  for (int i = 0; i < (sizeof(solenoidPins) / sizeof(solenoidPins[0])); i++) {
    if (solenoidPins[i] > -1) {
      pinMode(solenoidPins[i], OUTPUT);
      pinModes[solenoidPins[i]] = 2;
    }
  }

  ///End solenoid set up///


  Serial.begin(115200);
}


void loop() {
  // Read incoming serial and queue complete G-code lines
  readSerialLines();
  // Process one queued G-code line when motors are idle
  serviceGcodeQueue();
  serviceMotors();
}


void WriteLine(std::string text) {
  Serial.println(text.c_str());
  Serial.flush();
}
void WriteLine(int i) {
  Serial.println(i);
  Serial.flush();
}

void SetIOPin(int pinNumber, bool high) {
  if (high) {
    digitalWrite(pinNumber, HIGH);
  } else {
    digitalWrite(pinNumber, LOW);
  }
}


void ProcessInput(std::string input) {
  std::vector<std::string> args = split_string(input);

  if (args.size() == 0) { return; }



  if (args[0] == "G0" || args[0] == "G1")  /////move/////
  {
    // Absolute moves to target positions in steps; uses per-axis Speed_* as constant step rate
    for (int x = 1; x < args.size(); x++) {
      int i = x - 1;
      if (args[x][0] == AxesNames[i] && stepPins[i] > -1) {
        long tgt = std::stol(args[x].substr(1));
        startMoveTo(i, tgt, speeds[i]);
      }
    }
    WriteLine(input + " ok");
  }

  if (args[0] == "G2")  /////arc clockwise/////
  {


    WriteLine(input + " ok");
  }
  if (args[0] == "G3")  /////arc counter-clockwise/////
  {


    WriteLine(input + " ok");
  }
  if (args[0] == "G4")  /////dwell/////
  {


    WriteLine(input + " ok");
  }

  if (args[0] == "G28")  /////home/////
  {
    bool any = false;
    if (args.size() > 1) {
      for (size_t x = 1; x < args.size(); x++) {
        if (args[x].empty()) continue;
        int idx = axisIndexFromChar(args[x][0]);
        if (idx > -1) {
          if (motors[idx].moving) {
            stopMotorIdx(idx);
          }
          startHomingSequence(idx);
          any = true;
        }
      }
    }

    if (!any) {
      for (int i = 0; i < 6; ++i) {
        if (stepPins[i] == -1) continue;
        if (motors[i].moving) {
          stopMotorIdx(i);
        }
        startHomingSequence(i);
      }
    }

    WriteLine(input + " ok");
  }


  if (args[0] == "G92")  /////set position/////
  {
    for (int x = 1; x < args.size(); x++) {
      int i = x - 1;
      if (args[x][0] == AxesNames[i]) { motors[i].currentPos = std::stol(args[x].substr(1)); }
    }
    WriteLine(input + " ok");
  }


  if (args[0] == "M17")  /////enable motors/////
  {
    if (args.size() > 1) {
      for (int x = 1; x < args.size(); x++) {
        int i = x - 1;
        if (args[x][0] == AxesNames[i]) { enableDriver(i, true); }
      }
    } else {
      for (int x = 0; x < (sizeof(AxesNames) / sizeof(AxesNames[0])); x++) {
        enableDriver(x, true);
      }
    }
    WriteLine(input + " ok");
  }


  if (args[0] == "M18")  /////disable motors/////
  {
    if (args.size() > 1) {
      for (int x = 1; x < args.size(); x++) {
        int i = x - 1;
        if (args[x][0] == AxesNames[i]) { enableDriver(i, false); }
      }
    } else {
      for (int x = 0; x < (sizeof(AxesNames) / sizeof(AxesNames[0])); x++) { enableDriver(x, false); }
    }
    WriteLine(input + " ok");
  }

  if (args[0] == "M42")  /////set IO pin/////
  {
    int pinNumber = 99, pinState = 99, pinType = 99;

    //read gcode line parameters
    for (int x = 1; x < args.size(); x++) {
      if (args[x][0] == 'P') {
        pinNumber = std::stoi(args[x].substr(1, args[x].length() - 1));
      } else if (args[x][0] == 'S') {
        pinState = std::stoi(args[x].substr(1, args[x].length() - 1));
      } else if (args[x][0] == 'T') {
        pinType = std::stoi(args[x].substr(1, args[x].length() - 1));
      }
    }

    if (pinNumber < 99) {
      if (pinType < 99) {
        //change pin type to input or output
        if (pinType == 1) {
          pinMode(pinNumber, OUTPUT);
          pinModes[pinNumber] = 2;
        } else {
          pinMode(pinNumber, INPUT);
          pinModes[pinNumber] = 1;
        }
      }
      if (pinState < 99) {
        for (int x = 0; x < (sizeof(solenoidPins) / sizeof(solenoidPins[0])); x++) {
          if (solenoidPins[x] == pinNumber) {
            SetIOPin(solenoidPins[x], pinState == 1);
            solenoidsOn[x] = pinState == 1;
            break;
          }
        }
      }
    }

    WriteLine(input + " ok");
  }

  if (args[0] == "M43")  /////read IO pin(s)/////
  {
    int pinNumber = 99;

    //read gcode line parameters
    for (int x = 1; x < args.size(); x++) {
      if (args[x][0] == 'P') { pinNumber = std::stoi(args[x].substr(1, args[x].length() - 1)); }
    }

    if (pinNumber < 99) {
      WriteLine(digitalRead(pinNumber));
    } else {
      //Write all pin states
      std::string line = "";
      for (int x = 1; x < pinNumber; x++) {
        {
          if (pinModes[x] == 2) { line = line + std::to_string(x) + ":" + std::to_string(digitalRead(x)) + " "; }
        }
      }
      WriteLine(line);
    }
    WriteLine(input + " ok");
  }

  if (args[0] == "M84")  /////Stop idle hold/////
  {

    WriteLine(input + " ok");
  }

  if (args[0] == "M112")  /////Full Stop (EStop)/////
  {

    WriteLine(input + " ok");
  }

  if (args[0] == "M114")  /////Get current position/////
  {
    std::string line = "";
    for (int i = 0; i < (sizeof(AxesNames) / sizeof(AxesNames[0])); i++) {
      if (stepPins[i] > -1) { line = line + AxesNames[i] + ":" + std::to_string(motors[i].currentPos); }
    }
    WriteLine(line);
    WriteLine(input + " ok");
  }

  if (args[0] == "M115")  /////Get machine name/////
  {
    WriteLine("MACHINE_TYPE:" + MachineName);
    WriteLine(input + " ok");
  }

  if (args[0] == "M119")  /////Endstop status/////
  {
    // Report homing/endstop inputs (HIGH=triggered; adjust if your switches are active-LOW)
    std::string line = "";

    for (int i = 0; i < (sizeof(AxesNames) / sizeof(AxesNames[0])); i++) {
      if (homingPins[i] > -1) { line = line + AxesNames[i] + ":" + std::to_string(digitalRead(homingPins[i]) == HIGH); }
    }
    WriteLine(line);
    WriteLine(input + " ok");
  }

  if (args[0] == "M201")  /////Set acceleration (ignored in LEDC constant-speed mode)/////
  {
    // Parse but ignore; report ok so G-code senders don't error out
    // Example accepted forms: "M201 X1000 Y2000" etc.
    WriteLine(input + " ok");
  }

  if (args[0] == "M203")  /////Set feedrate/////
  {
    for (int x = 1; x < args.size(); x++) {
      int i = x - 1;
      if (args[x][0] == AxesNames[i]) {
        speeds[i] = std::stoi(args[x].substr(1));
        speeds[i] = speeds[i];
      }
    }
    WriteLine(input + " ok");
  }

  if (args[0] == "M279")  /////Get servo position/////
  {
    std::string line = "";

    for (int i = 0; i < (sizeof(servoPins) / sizeof(servoPins[0])); i++) {
      if (servoPins[i] > -1) { line = line + AxesNames[i] + ":" + std::to_string(servos[i].read()) + " "; }
    }
    WriteLine(line);
    WriteLine(input + " ok");
  }

  if (args[0] == "M280")  /////Set servo position/////
  {
    int pinNumber = 99, angle = 0;
    bool angleSet = false;

    //read gcode line parameters
    for (int x = 1; x < args.size(); x++) {
      if (args[x][0] == 'P') {
        pinNumber = std::stoi(args[x].substr(1, args[x].length() - 1));
      } else if (args[x][0] == 'S') {
        angle = std::stoi(args[x].substr(1, args[x].length() - 1));
        angleSet = true;
      }
    }

    if (angleSet) {
      for (int i = 0; i < (sizeof(servoPins) / sizeof(servoPins[0])); i++) {
        if (pinNumber == servoPins[i]) {
          servos[i].write(angle);
          break;
        }
      }
    }

    WriteLine(input + " ok");
  }
}
