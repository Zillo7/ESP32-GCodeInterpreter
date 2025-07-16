#include <ESP32Servo.h>
#include <ESP_FlexyStepper.h>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>


///Machine name - for instrument ID///
std::string MachineName = "Harmonica";


///Stepper Motors///

ESP_FlexyStepper stepper_A, stepper_B, stepper_C, stepper_D, stepper_E, stepper_F;

int StepPin_A = 13, MotorDirPin_A = 12, HomingPin_A = 27, EStopPin_A = 14, EnablePin_A = 14, HomeMoveDir_A = 1,
    StepPin_B = -1, MotorDirPin_B = -1, HomingPin_B = -1, EStopPin_B = -1, EnablePin_B = 14, HomeMoveDir_B = 1,
    StepPin_C = -1, MotorDirPin_C = -1, HomingPin_C = -1, EStopPin_C = -1, EnablePin_C = 14, HomeMoveDir_C = 1,
    StepPin_D = -1, MotorDirPin_D = -1, HomingPin_D = -1, EStopPin_D = -1, EnablePin_D = 14, HomeMoveDir_D = 1,
    StepPin_E = -1, MotorDirPin_E = -1, HomingPin_E = -1, EStopPin_E = -1, EnablePin_E = 14, HomeMoveDir_E = 1,
    StepPin_F = -1, MotorDirPin_F = -1, HomingPin_F = -1, EStopPin_F = -1, EnablePin_F = 14, HomeMoveDir_F = 1;

    //harmonica pinout
    //StepPin_A = 13, MotorDirPin_A = 12, HomingPin_A = 27, EStopPin_A = 14, EnablePin_A = 14, HomeMoveDir_A = 1,
    //StepPin_B = -1, MotorDirPin_B = -1, HomingPin_B = -1, EStopPin_B = -1, EnablePin_B = 14, HomeMoveDir_B = 1,

    //int solenoid_A_Pin = 25, solenoid_B_Pin = 26,

    //testing platform pinout
    //int StepPin_A = 32, MotorDirPin_A = 18, HomingPin_A = 27, EStopPin_A = 16, EnablePin_A = 14, HomeMoveDir_A = 1,
    //StepPin_B = 33, MotorDirPin_B = 19, HomingPin_B = 26, EStopPin_B = 16, EnablePin_B = 14, HomeMoveDir_B = 1,

    //int solenoid_A_Pin = 25, solenoid_B_Pin = 26,

int DistToMove_A = 2000, Speed_A = 3200, Acell_A = 10000, Decel_A = 10000,
    DistToMove_B = 2000, Speed_B = 600, Acell_B = 500, Decel_B = 500,
    DistToMove_C = 2000, Speed_C = 600, Acell_C = 500, Decel_C = 500,
    DistToMove_D = 2000, Speed_D = 600, Acell_D = 500, Decel_D = 500,
    DistToMove_E = 2000, Speed_E = 600, Acell_E = 500, Decel_E = 500,
    DistToMove_F = 2000, Speed_F = 600, Acell_F = 500, Decel_F = 500;

int steps_A = 0;

///End Stepper Motors///


///Servos///

Servo servo_A, servo_B, servo_C, servo_D, servo_E, servo_F;
int servo_A_Pin = -1, servo_B_Pin = -1, servo_C_Pin = -1, servo_D_Pin = -1, servo_E_Pin = -1, servo_F_Pin = -1;
int servo_A_Pos = 0, servo_B_Pos = 0, servo_C_Pos = 0, servo_D_Pos = 0, servo_E_Pos = 0, servo_F_Pos = 0;

///End Servos///

///Solenoids///

int solenoid_A_Pin = 25, solenoid_B_Pin = 26, solenoid_C_Pin = -1, solenoid_D_Pin = -1, solenoid_E_Pin = -1, solenoid_F_Pin = -1, solenoid_G_Pin = -1, solenoid_H_Pin = -1, solenoid_I_Pin = -1, solenoid_J_Pin = -1;
bool solenoid_A_High = false, solenoid_B_High = false, solenoid_C_High = false, solenoid_D_High = false, solenoid_E_High = false, solenoid_F_High = false, solenoid_G_High = false, solenoid_H_High = false, solenoid_I_High = false, solenoid_J_High = false;

///End Solenoids///


std::string inputString = "";
bool inputComplete = false;


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
  Serial.begin(115200);
  inputString.reserve(200);


///Set up stepper motors///
  if (StepPin_A > -1)
  {
    // connect and configure the stepper motor to its IO pins
    stepper_A.connectToPins(StepPin_A, MotorDirPin_A);
    stepper_A.setEnablePin(EnablePin_A, false);
    stepper_A.setSpeedInStepsPerSecond(Speed_A);
    stepper_A.setAccelerationInStepsPerSecondPerSecond(Acell_A);
    stepper_A.setDecelerationInStepsPerSecondPerSecond(Decel_A);
    stepper_A.startAsService();
    WriteLine("Started Stepper_A");
  }
  if (StepPin_B > -1)
  {
    stepper_B.connectToPins(StepPin_B, MotorDirPin_B);
    stepper_A.setEnablePin(EnablePin_B, false);
    stepper_B.setSpeedInStepsPerSecond(Speed_B);
    stepper_B.setAccelerationInStepsPerSecondPerSecond(Acell_B);
    stepper_B.setDecelerationInStepsPerSecondPerSecond(Decel_B);
    stepper_B.startAsService(0);
    WriteLine("Started Stepper_B");
  }
  if (StepPin_C > -1)
  {
    stepper_C.connectToPins(StepPin_C, MotorDirPin_C);
    stepper_A.setEnablePin(EnablePin_C, false);
    stepper_C.setSpeedInStepsPerSecond(Speed_C);
    stepper_C.setAccelerationInStepsPerSecondPerSecond(Acell_C);
    stepper_C.setDecelerationInStepsPerSecondPerSecond(Decel_C);
    stepper_C.startAsService(0);
    WriteLine("Started Stepper_C");
  }
  if (StepPin_D > -1)
  {
    stepper_D.connectToPins(StepPin_D, MotorDirPin_D);
    stepper_A.setEnablePin(EnablePin_D, false);
    stepper_D.setSpeedInStepsPerSecond(Speed_D);
    stepper_D.setAccelerationInStepsPerSecondPerSecond(Acell_D);
    stepper_D.setDecelerationInStepsPerSecondPerSecond(Decel_D);
    stepper_D.startAsService(0);
    WriteLine("Started Stepper_D");
  }
  if (StepPin_E > -1)
  {
    stepper_E.connectToPins(StepPin_E, MotorDirPin_E);
    stepper_A.setEnablePin(EnablePin_E, false);
    stepper_E.setSpeedInStepsPerSecond(Speed_E);
    stepper_E.setAccelerationInStepsPerSecondPerSecond(Acell_E);
    stepper_E.setDecelerationInStepsPerSecondPerSecond(Decel_E);
    stepper_E.startAsService(0);
    WriteLine("Started Stepper_E");
  }
  if (StepPin_F > -1)
  {
    // connect and configure the stepper motor to its IO pins
    stepper_F.connectToPins(StepPin_F, MotorDirPin_F);
    stepper_A.setEnablePin(EnablePin_F, false);
    stepper_F.setSpeedInStepsPerSecond(Speed_F);
    stepper_F.setAccelerationInStepsPerSecondPerSecond(Acell_F);
    stepper_F.setDecelerationInStepsPerSecondPerSecond(Decel_F);
    stepper_F.startAsService(0);
    WriteLine("Started Stepper_F");
  }
///End stepper motor set up///
  

  //stepper_A.registerTargetPositionReachedCallback(targetPositionReachedCallback);
  //you can also register for other events using (these accept currently only functions without any parameters):
  // stepper_A.registerEmergencyStopReleasedCallback(...);
  // stepper_A.registerEmergencyStopTriggeredCallback(...);
  // stepper_A.registerHomeReachedCallback(...);
  // stepper_A.registerLimitReachedCallback(...);

  


  ///Set up servo motors///
  if (servo_A_Pin >= 0) { servo_A.attach(servo_A_Pin); }
  if (servo_B_Pin >= 0) { servo_A.attach(servo_B_Pin); }
  if (servo_C_Pin >= 0) { servo_A.attach(servo_C_Pin); }
  if (servo_D_Pin >= 0) { servo_A.attach(servo_D_Pin); }
  if (servo_E_Pin >= 0) { servo_A.attach(servo_E_Pin); }
  if (servo_F_Pin >= 0) { servo_A.attach(servo_F_Pin); }
  ///End servo set up///

  ///Set up solenoids///

  if (solenoid_A_Pin >= 0) { pinMode(solenoid_A_Pin, OUTPUT); }
  if (solenoid_B_Pin >= 0) { pinMode(solenoid_B_Pin, OUTPUT); }
  if (solenoid_C_Pin >= 0) { pinMode(solenoid_C_Pin, OUTPUT); }
  if (solenoid_D_Pin >= 0) { pinMode(solenoid_D_Pin, OUTPUT); }
  if (solenoid_E_Pin >= 0) { pinMode(solenoid_E_Pin, OUTPUT); }
  if (solenoid_F_Pin >= 0) { pinMode(solenoid_F_Pin, OUTPUT); }
  if (solenoid_G_Pin >= 0) { pinMode(solenoid_G_Pin, OUTPUT); }
  if (solenoid_H_Pin >= 0) { pinMode(solenoid_H_Pin, OUTPUT); }
  if (solenoid_I_Pin >= 0) { pinMode(solenoid_I_Pin, OUTPUT); }
  if (solenoid_J_Pin >= 0) { pinMode(solenoid_J_Pin, OUTPUT); }

  ///End solenoid set up///
}


void loop() {
  if (inputComplete) {
    //do a thing
    //WriteLine(inputString);
    inputString = "";
    inputComplete = false;
  }

  if(steps_A != 0) { stepper_A.moveToPositionInSteps(steps_A); steps_A = 0; }
}


void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') { inputComplete = true; }
    if (inputComplete) { ProcessInput(inputString); }
  }
}


void ProcessInput(std::string input) {
  std::vector<std::string> args = split_string(input);

  if (args.size() == 0) { return; }
  

  if (args[0] == "G0" || args[0] == "G1")  /////move/////
  {
    for (int x = 1; x < args.size(); x++) {
           if (args[x][0] == 'X') { stepper_A.setTargetPositionInSteps(std::stoi(args[x].substr(1, args[x].length() - 1))); }// = (std::stoi(args[x].substr(1, args[x].length() - 1))); }
      else if (args[x][0] == 'Y') { stepper_B.setTargetPositionInSteps(std::stoi(args[x].substr(1, args[x].length() - 1))); }
      else if (args[x][0] == 'Z') { stepper_C.setTargetPositionInSteps(std::stoi(args[x].substr(1, args[x].length() - 1))); }
      else if (args[x][0] == 'A') { stepper_D.setTargetPositionInSteps(std::stoi(args[x].substr(1, args[x].length() - 1))); }
      else if (args[x][0] == 'B') { stepper_E.setTargetPositionInSteps(std::stoi(args[x].substr(1, args[x].length() - 1))); }
      else if (args[x][0] == 'C') { stepper_F.setTargetPositionInSteps(std::stoi(args[x].substr(1, args[x].length() - 1))); }
    }

    
    WriteLine("G0/1 ok");

  }

  if (args[0] == "G2")  /////arc clockwise/////
  {


    WriteLine("G2 ok");
  } 
  if (args[0] == "G3")  /////arc counter-clockwise/////
  {


    WriteLine("G3 ok");

  } 
  if (args[0] == "G4")  /////dwell/////
  {


    WriteLine("G4 ok");

  } 
  if (args[0] == "G28")  /////home/////
  {
    if(args.size() > 1)
    {
      for (int x = 1; x < args.size(); x++) {
      if (args[x][0] == 'X') { stepper_A.moveToHomeInSteps(HomeMoveDir_A, 50, DistToMove_A, HomingPin_A); }
      //else if (args[x][0] = 'Y')
      //{  }
    }
    }
    else
    {
        stepper_A.moveToHomeInSteps(HomeMoveDir_A, 50, DistToMove_A, HomingPin_A);
    }

    WriteLine("G28 ok");

  } 
  if (args[0] == "G92")  /////set position/////
  {
    for (int x = 1; x < args.size(); x++) {
      if (args[x][0] == 'X') { stepper_A.setCurrentPositionInSteps(std::stoi(args[x].substr(1, args[x].length() - 1))); }
      //else if (args[x][0] = 'Y')
      //{  }
    }
    WriteLine("G92 ok");

  } 
  if (args[0] == "M17")  /////enable motors/////
  {
    if(args.size() > 1)
    {
      for (int x = 1; x < args.size(); x++) {
      if (args[x][0] == 'X') { stepper_A.enableDriver(); }
      //else if (args[x][0] = 'Y')
      //{  }
    }
    }
    else
    {
        stepper_A.enableDriver();
    }

    WriteLine("M17 ok");

  } 
  if (args[0] == "M18")  /////disable motors/////
  {
    if(args.size() > 1)
    {
      for (int x = 1; x < args.size(); x++) {
      if (args[x][0] == 'X') { stepper_A.disableDriver(); }
      //else if (args[x][0] = 'Y')
      //{  }
    }
    }
    else
    {
        stepper_A.disableDriver();
    }

    WriteLine("M18 ok");

  } 
  if (args[0] == "M42")  /////set IO pin/////
  {
	  int pinNumber = 99, pinState = 99, pinType = 99;
	  
	 //read gcode line parameters
	for (int x = 1; x < args.size(); x++) {
      if (args[x][0] == 'P') { pinNumber = std::stoi(args[x].substr(1, args[x].length() - 1)); }
      else if (args[x][0] == 'S') { pinState = std::stoi(args[x].substr(1, args[x].length() - 1)); }
      else if (args[x][0] == 'T') { pinType = std::stoi(args[x].substr(1, args[x].length() - 1)); }
    }
	
	if (pinNumber < 99)
	{
		if(pinType < 99)
		{
			//change pin type to input or output
			if (pinType == 1) { pinMode(pinNumber, OUTPUT); }
			else { pinMode(pinNumber, INPUT); }
		}
		if(pinState < 99)
		{
			if (pinNumber == solenoid_A_Pin) { SetIOPin(solenoid_A_Pin, pinState == 1); solenoid_A_High = pinState == 1; }
      else if (pinNumber == solenoid_B_Pin) { SetIOPin(solenoid_B_Pin, pinState == 1); solenoid_B_High = pinState == 1; }
		}
	}

    WriteLine("M42 ok");

  } 
  if (args[0] == "M43")  /////read IO pin(s)/////
  {
	int pinNumber = 99;
	  
	 //read gcode line parameters
	for (int x = 1; x < args.size(); x++) {
      if (args[x][0] == 'P') { pinNumber = std::stoi(args[x].substr(1, args[x].length() - 1)); }
    }
	
	if (pinNumber < 99)
	{ WriteLine(digitalRead(pinNumber)); }
    else 
	{
		//read all pins TODO
		
	}
  } 
  if (args[0] == "M84")  /////Stop idle hold/////
  {
    stepper_A.disableDriver();

    WriteLine("M84 ok");

  } 
  if (args[0] == "M112")  /////Full Stop (EStop)/////
  {
    stepper_A.emergencyStop(false);

    WriteLine("M112 ok");

  } 
  if (args[0] == "M114")  /////Get current position/////
  {
    if (args[0] == "M114")  /////Get current position/////
{
  char line[128];
  snprintf(line, sizeof(line), "X:%ld", stepper_A.getCurrentPositionInSteps());

  if (StepPin_B > -1) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), " Y:%ld", stepper_B.getCurrentPositionInSteps());
    strncat(line, buffer, sizeof(line) - strlen(line) - 1);
  }
  if (StepPin_C > -1) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), " Z:%ld", stepper_C.getCurrentPositionInSteps());
    strncat(line, buffer, sizeof(line) - strlen(line) - 1);
  }
  if (StepPin_D > -1) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), " A:%ld", stepper_D.getCurrentPositionInSteps());
    strncat(line, buffer, sizeof(line) - strlen(line) - 1);
  }
  if (StepPin_E > -1) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), " B:%ld", stepper_E.getCurrentPositionInSteps());
    strncat(line, buffer, sizeof(line) - strlen(line) - 1);
  }
  if (StepPin_F > -1) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), " C:%ld", stepper_F.getCurrentPositionInSteps());
    strncat(line, buffer, sizeof(line) - strlen(line) - 1);
  }

  WriteLine(line);
  WriteLine("M114 ok");
}

    //std::string line = "X:" + std::to_string(stepper_A.getCurrentPositionInSteps());
    //if (StepPin_B > -1) { line = line + " Y:" + std::to_string(stepper_B.getCurrentPositionInSteps()); }
    //if (StepPin_C > -1) { line = line + " Z:" + std::to_string(stepper_C.getCurrentPositionInSteps()); }
    //if (StepPin_D > -1) { line = line + " A:" + std::to_string(stepper_D.getCurrentPositionInSteps()); }
    //if (StepPin_E > -1) { line = line + " B:" + std::to_string(stepper_E.getCurrentPositionInSteps()); }
    //if (StepPin_F > -1) { line = line + " C:" + std::to_string(stepper_F.getCurrentPositionInSteps()); }
    //WriteLine(line);

    //WriteLine("M114 ok");

  } 
  if (args[0] == "M115")  /////Get machine name/////
  {
    WriteLine("MACHINE_TYPE:" + MachineName);
  }
  if (args[0] == "M119")  /////Get endstop status/////
  {
    std::string line = "";
    if (HomingPin_A > -1) { line = line + "X:" + std::to_string(stepper_A.getCurrentPositionInMillimeters()); }
    if (HomingPin_B > -1) { line = line + ", Y:" + std::to_string(stepper_B.getCurrentPositionInMillimeters()); }
    if (HomingPin_C > -1) { line = line + ", Z:" + std::to_string(stepper_C.getCurrentPositionInMillimeters()); }
    if (HomingPin_D > -1) { line = line + ", A:" + std::to_string(stepper_D.getCurrentPositionInMillimeters()); }
    if (HomingPin_E > -1) { line = line + ", B:" + std::to_string(stepper_E.getCurrentPositionInMillimeters()); }
    if (HomingPin_F > -1) { line = line + ", C:" + std::to_string(stepper_F.getCurrentPositionInMillimeters()); }
    WriteLine(line);

    WriteLine("M119 ok");
  }
  if (args[0] == "M201")  /////Set acceleration/////
  {
    for (int x = 1; x < args.size(); x++) {
      if (args[x][0] == 'X') { Acell_A = Decel_A = std::stoi(args[x].substr(1, args[x].length() - 1));
      
      stepper_A.setAccelerationInStepsPerSecondPerSecond(Acell_A);
      stepper_A.setDecelerationInStepsPerSecondPerSecond(Decel_A);
      }
      //else if (args[x][0] = 'Y')
      //{  }
    }

    WriteLine("M201 ok");

  }
  if (args[0] == "M203")  /////Set feedrate/////
  {
    for (int x = 1; x < args.size(); x++) {
      if (args[x][0] == 'X') { stepper_A.setSpeedInStepsPerSecond(std::stoi(args[x].substr(1, args[x].length() - 1))); }
      //else if (args[x][0] = 'Y')
      //{  }
    }

    WriteLine("M203 ok");

  }
  if (args[0] == "M280")  /////Set servo position/////
  {
		  int pinNumber = 99, angle = 0; bool angleSet = false;
	  
	 //read gcode line parameters
	for (int x = 1; x < args.size(); x++) {
      if (args[x][0] == 'P') { pinNumber = std::stoi(args[x].substr(1, args[x].length() - 1)); }
      else if (args[x][0] == 'S') { angle = std::stoi(args[x].substr(1, args[x].length() - 1)); angleSet = true; }
    }

    if (angleSet)
    {
      if (pinNumber == servo_A_Pin) { servo_A.write(angle); }
      else if (pinNumber == servo_B_Pin) { servo_B.write(angle); }
      else if (pinNumber == servo_C_Pin) { servo_C.write(angle); }
      else if (pinNumber == servo_D_Pin) { servo_D.write(angle); }
      else if (pinNumber == servo_E_Pin) { servo_E.write(angle); }
      else if (pinNumber == servo_F_Pin) { servo_F.write(angle); }
    }
	
    WriteLine("M280 ok");

  }
  

}

void WriteLine(std::string text) {
  Serial.println(text.c_str());
  Serial.flush();
}
void WriteLine(int i) {
  Serial.println(i); Serial.flush();
}

void SetIOPin(int pinNumber, bool high)
{ if (high) { digitalWrite(pinNumber, HIGH); } else { digitalWrite(pinNumber, LOW); } }


