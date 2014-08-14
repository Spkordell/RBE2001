/*
Purpose: This code was written to control Team #6's robot in the RBE 2001 final project entitled "The robotic Replacement of Spent Nuclear Fuel Rods."
Pin Configurations: See define statemens below.
Author: Steven Kordell, Jiaqi Ren, Cody Wall
Date: 10/9/2012
*/

#include <TimerThree.h>
#include <TimerOne.h>
#include <Servo.h>
#include <LiquidCrystal.h>

////////////////////BEGIN PIN DEFINITISONS////////////////////
//Input pins
#define lineSensorPin0 A0 //front left line sensor
#define lineSensorPin1 A1 //rear left line sensor
#define lineSensorPin2 A2 //rear right line sensor
#define lineSensorPin3 A3 //front right line sensor
#define pauseButtonPin 22 
#define armUpStopPin 24
#define armDownStopPin 25
#define frontBumperPin 26
//Output pins
#define leftMotorPin 10
#define rightMotorPin 9
#define armLiftMotorPin 8
#define clawRetractMotorPin 7
#define clawPin 6
#define radiationLedPin 13 //active low
////////////////////END PIN DEFINITISONS////////////////////

//Constants for various speeds
#define defaultForwardVelocity 21
#define slowForwardVelocity 17
byte forwardVelocity=defaultForwardVelocity; //value between 0 and 90
#define turningVelocity 25 //value between 0 and 90
#define armLiftSpeed 30 //value between 0 and 90

//Constants for line sensing
#define blackValue 750
#define whiteValue 28
#define lowSensorMap 0  //value on white
#define highSensorMap 1  //value on black

//Bluetooth Signals
#define btStart 0x5F
//Message Types
#define msgStoreAvailability 0x01
#define msgSupplyAvailability 0x02
#define msgRadiationAlert 0x03
#define msgStopMovement 0x04
#define msgResumeMovement 0x05
#define msgRobotStatus 0x06
#define msgHeartbeat 0x07
//addresses
#define reactorCtrlAddress 0x00
#define broadcastAddress 0x00
#define teamAddress 0x02
//radiation types
#define dataSpent 0x2C
#define dataNew 0xFF
//Robot status types
#define rsStopped 0x01
#define rsMovingTeleoperation 0x02
#define rsMovingAutonomous 0x03
#define rsNoRodInGripper 0x01
#define rsRodInGripper 0x02
#define rsGripAttemptInProgress 0x01
#define rsDrivingToReactor 0x02
#define rsDrivingToStorage 0x03
#define rsDrivingToSupply 0x04
#define rsIdle 0x05

//Position,heading, and task state definitions
enum positionStates {reactorA,reactorB,newRod1,newRod2,newRod3,newRod4,spentRod1,spentRod2,spentRod3,spentRod4,turnpikeA,turnpikeB,turnpikeC,turnpikeD,turnpikeE,intersectionA,intersectionB,intersectionC,intersectionD,legNewRod1,legNewRod2,legNewRod3,legNewRod4,legSpentRod1,legSpentRod2,legSpentRod3,legSpentRod4};
enum headingStates {north,south,east,west};
enum reactorTubeStates {tubeA,tubeB};
enum taskStates {navigateToReactorPickup,removeSpentReactorRod,findEmptyStorage,navigateToStorage,storeSpentRod,findSupplyRod,getNewRod,navigateToReactorDropoff,insertNewReactorRod,changeTube};
enum rodType {spentRod,newRod,noRod};

//Position, heading, and task state variables
positionStates currentPosition = legSpentRod1;
headingStates currentHeading = east;
taskStates currentTask  = navigateToReactorPickup;
reactorTubeStates currentTube = tubeA;
positionStates targetPosition = reactorA;
rodType radiationType = noRod;

//Variables to store sensor values
int lineSensorFL;
int lineSensorRL;
int lineSensorRR;
int lineSensorFR;

//Actuation objects
Servo leftMotor;
Servo rightMotor;
Servo armLiftMotor;
Servo clawRetractMotor;
Servo claw;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);   // Initialize the library with the numbers of the interface pins

//bluetooth variables
byte btlength;
byte btMsgType;
byte btSrcAddress;
byte btDestAddress;
byte btData;
byte btChecksum;
byte supplyRodStatus;
byte spentRodStatus;

//Miscellanious variables
boolean trigger;
boolean alertLedState;
boolean stopMotionFlag=false;
int errorCode;
byte heartbeatCounter;
byte radiationCounter;

void setup() {
  lcd.begin(16, 2);
  //attach actuation pins to servo objects
  leftMotor.attach(leftMotorPin);
  rightMotor.attach(rightMotorPin);
  armLiftMotor.attach(armLiftMotorPin,1000,2000);
  clawRetractMotor.attach(clawRetractMotorPin);
  claw.attach(clawPin);
  //PinMode settings
  pinMode(pauseButtonPin,INPUT);
  pinMode(armUpStopPin,INPUT);
  pinMode(armDownStopPin,INPUT);
  pinMode(frontBumperPin,INPUT);
  pinMode(radiationLedPin,OUTPUT);
  digitalWrite(radiationLedPin,HIGH);
  //Enable Pullup resistors as necessary
  digitalWrite(armUpStopPin,HIGH);
  digitalWrite(armDownStopPin,HIGH);
  digitalWrite(frontBumperPin,HIGH);  
  digitalWrite(pauseButtonPin,HIGH);
  //Initialize Serial communication for bluetooth  
  Serial1.begin(115200);
  //Serial.begin(9600);
  //initialize timers
  Timer1.initialize(500000);
  Timer1.attachInterrupt(updateAlertLed); //update the state of the radiation alert LED and transmit a heartbeat if necessary
  updateDisplay();
  transmitStatus(rsIdle);
  while (digitalRead(pauseButtonPin)); {}
  delay(500);
  retractClaw();
  openClaw();
  lowerArm();
  transmitStatus(rsNoRodInGripper);
  navigateToField();
}

void loop() {
  navigate();
  if (!digitalRead(pauseButtonPin)) {        //see if pause/play button has been pressed
    stopAll();
    transmitStatus(rsStopped);
    delay(1000);
    while (digitalRead(pauseButtonPin)); {}
    delay(500);
    transmitStatus(rsMovingAutonomous);
  }
}

void navigateToField() {
  transmitStatus(rsMovingAutonomous);
  leftMotor.write(90);
  rightMotor.write(180); 
  delay(450);  
  while(lineSensorRR==0) {
    getBackLine();
    leftMotor.write(65);
    rightMotor.write(120);  
  }
  turnLeft90();
  transmitStatus(rsDrivingToReactor);
}

void raiseArm() {
 while (digitalRead(armUpStopPin)) {
    armLiftMotor.write(90+armLiftSpeed);
  }
  armLiftMotor.write(90);
  //Serial.println(digitalRead(armUpStopPin));
}

void lowerArm() {
   while (digitalRead(armDownStopPin)) {
    armLiftMotor.write(90-armLiftSpeed);
  }
  armLiftMotor.write(90);
}

void retractClaw() {
  clawRetractMotor.write(180);
}

void extendClaw() {
  clawRetractMotor.write(0);
}

void openClaw() {
  claw.write(0);
}

void closeClaw() {
  claw.write(180);
}

void navigate() {
  getFullbtPacket();
  while (stopMotionFlag) {
    stopAll();
    getFullbtPacket();
  }
  getFrontLine();
  if (lineSensorFL==0 && lineSensorFR==0) {
    leftMotor.write(90-forwardVelocity);
    rightMotor.write(90+forwardVelocity+5);
  }
  if (trigger) {
      getBackLine();
      if (lineSensorRL==1 && lineSensorRR==1) {
        trigger=false;
        checkIntersection();
        //turnLeft90();
      }
    }
  if (digitalRead(frontBumperPin)==0) {  
    checkIntersection();
    forwardVelocity=defaultForwardVelocity;
  }
  if (lineSensorFL==1 && lineSensorFR==1) {
     if (((currentHeading==west) && (currentPosition==legSpentRod1 || currentPosition==legSpentRod2 || currentPosition==legSpentRod3 || currentPosition==legSpentRod4)) || ((currentHeading==east) && (currentPosition==legNewRod4 || currentPosition==legNewRod3 || currentPosition==legNewRod2 || currentPosition==legNewRod1)) || ((currentHeading==north) && (currentPosition==turnpikeE)) || ((currentHeading==south) && (currentPosition==turnpikeA))) {
        //while (1) {stopAll();}
        //checkIntersection();
        forwardVelocity=slowForwardVelocity;
      } else {
        trigger=true;
        //leftMotor.write(90-forwardVelocity); //tag - moved
        //rightMotor.write(90+forwardVelocity); //tag -moved
      }
      leftMotor.write(90-forwardVelocity); //tag//added
      rightMotor.write(90+forwardVelocity+5); //tag//added
  }
  if (lineSensorFL==1 && lineSensorFR==0) {
    leftMotor.write(90+forwardVelocity);
    rightMotor.write(90+forwardVelocity);
  }
  if (lineSensorFL==0 && lineSensorFR==1) {
    leftMotor.write(90-forwardVelocity);
    rightMotor.write(90-forwardVelocity);
  }
}

void turn180() {
  if (currentPosition==turnpikeA || currentPosition==turnpikeE) {    
    turnRight90(); 
  } else {
    turnRight90();
    turnRight90();  
  }
}

void turnLeft90(){
  lineSensorFL=0;
  lineSensorFR=0;  
  while (lineSensorFL==0) {
    leftMotor.write(90+turningVelocity+5);
    rightMotor.write(90+turningVelocity+5);
    getFrontLine();
  }
}

void turnRight90() { 
  lineSensorFL=0;
  lineSensorFR=0;  
  while (lineSensorFR==0) {
    leftMotor.write(90-turningVelocity);
    rightMotor.write(90-turningVelocity);
    getFrontLine();
  }
}

void reverse() {
   while (lineSensorFR==0) {
     getFrontLine();
     leftMotor.write(90+forwardVelocity);
     rightMotor.write(90-forwardVelocity);
   }
   while (lineSensorFR==1) {
     getFrontLine();
     leftMotor.write(90+forwardVelocity);
     rightMotor.write(90-forwardVelocity);
   }   
}

void stopAll() {
  leftMotor.write(90);
  rightMotor.write(90); 
  armLiftMotor.write(90);
}

void getBackLine() {
  lineSensorRL=map(analogRead(lineSensorPin1),whiteValue,blackValue,lowSensorMap,highSensorMap);
  lineSensorRR=map(analogRead(lineSensorPin2),whiteValue,blackValue,lowSensorMap,highSensorMap);
}

void getFrontLine() {
  lineSensorFL=map(analogRead(lineSensorPin0),whiteValue,blackValue,lowSensorMap,highSensorMap);
  lineSensorFR=map(analogRead(lineSensorPin3),whiteValue,blackValue,lowSensorMap,highSensorMap);
}

void goNorth() {
  switch (currentHeading) {
    case north:
      navigate();
      break;
    case south:
      turn180();
      break;
    case east:
      turnLeft90();
      break;
    case west:
      turnRight90();
      break;
    default:
      noCaseFound(1);
      break;
  }
 currentHeading=north; 
}
void goSouth() {
  switch (currentHeading) {
    case north:
      turn180();
      break;
    case south:
      navigate();
      break;
    case east:
      turnRight90();
      break;
    case west:
      turnLeft90();
      break;
    default:
      noCaseFound(2);
      break;
  }  
  currentHeading=south;
}
void goEast() {
  switch (currentHeading) {
    case north:
      turnRight90();
      break;
    case south:
      turnLeft90();
      break;
    case east:
      navigate();
      break;
    case west:
      turn180();
      break;
    default:
      noCaseFound(3);
      break;
  }  
  currentHeading=east;
}
void goWest() {
  switch (currentHeading) {
    case north:
      turnLeft90();
      break;
    case south:
      turnRight90();
      break;
    case east:
      turn180();
      break;
    case west:
      navigate();
      break;
    default:
      noCaseFound(4);
      break;
  }  
  currentHeading=west;
}

void decideDirection() {
  updateDisplay();
  switch (currentPosition){
    case intersectionA:
      switch (targetPosition) {
        case newRod1:
          goEast();
          currentPosition=legNewRod1;
          break;
        case spentRod4:
          goWest();
          currentPosition=legSpentRod4;    
          break;
        case reactorA:
          goSouth();
          currentPosition=turnpikeA;         
          break;
        default:
          goNorth();
          currentPosition=turnpikeB; 
          break;
      }
      break;
    case intersectionB:
      switch (targetPosition) {
        case newRod2:
          goEast();
          currentPosition=legNewRod2;
          break;
        case spentRod3:
          goWest();
          currentPosition=legSpentRod3;
          break;
        case newRod1:
          goSouth();
          currentPosition=turnpikeB;      
          break;
        case spentRod4:
          goSouth();
          currentPosition=turnpikeB;      
          break;
        case reactorA:
          goSouth();
          currentPosition=turnpikeB;      
          break;                   
        default:
          goNorth();
          currentPosition=turnpikeC;
          break;
      }
      break;
    case intersectionC:  //currently on
      switch (targetPosition) {
        case newRod3:
          goEast();
          currentPosition=legNewRod3;
          break;
        case spentRod2:
          goWest();
          currentPosition=legSpentRod2;
          break;
        case spentRod1:
          goNorth();
          currentPosition=turnpikeD;
          break;
        case newRod4:
          goNorth();
          currentPosition=turnpikeD;
          break;
        case reactorB:
          goNorth();
          currentPosition=turnpikeD;
          break;          
        default:
          goSouth();
          currentPosition=turnpikeC;
          break;
      }      
      break;      
    case intersectionD:
      switch (targetPosition) {
        case newRod4:
          goEast();
          currentPosition=legNewRod4;
          break;
        case spentRod1:
          goWest();
          currentPosition=legSpentRod1;
          break;
        case reactorB:
          goNorth();
          currentPosition=turnpikeE;
          break;
        default:
          goSouth();
          currentPosition=turnpikeD;
          break;
      }
      break;
    default:
      noCaseFound(5);
      break;
  }
}

void checkIntersection() {
  getFullbtPacket();
  updateDisplay();
  switch (currentHeading) {
    case north:
      switch (currentPosition) {
        case turnpikeA:
          currentPosition=intersectionA;
          break;
        case turnpikeB:
          currentPosition=intersectionB;
          break;
        case turnpikeC:
          currentPosition=intersectionC;
          break;
        case turnpikeD:
          currentPosition=intersectionD;
          break;
        case turnpikeE:
          currentPosition=reactorB;
          break; 
        default:
          noCaseFound(6);
          break;      
      }
      break;
    case south:
      switch (currentPosition) {
        case turnpikeA:
          currentPosition=reactorA;
          break;
        case turnpikeB:
          currentPosition=intersectionA;
          break;
        case turnpikeC:
          currentPosition=intersectionB;
          break;
        case turnpikeD:
          currentPosition=intersectionC;
          break;
        case turnpikeE:
          currentPosition=intersectionD;
          break; 
        default:
          noCaseFound(7);
          break;
      }  
      break;
    case east:
      switch (currentPosition) {
        case legSpentRod1:
          currentPosition=intersectionD;
          break;
        case legSpentRod2:
          currentPosition=intersectionC;
          break;
        case legSpentRod3:
          currentPosition=intersectionB;
          break;
        case legSpentRod4:
          currentPosition=intersectionA;
          break;   
        case legNewRod1:
           currentPosition=newRod1;
            break;    
        case legNewRod2:
           currentPosition=newRod2;
            break;    
        case legNewRod3:
           currentPosition=newRod3;
            break;    
        case legNewRod4:
           currentPosition=newRod4;
           break;
        default:
          noCaseFound(8);
          break;      
      }
      break;
    case west:
      switch (currentPosition) {
        case legNewRod1:
          currentPosition=intersectionA;
          break;
        case legNewRod2:
          currentPosition=intersectionB;
          break;
        case legNewRod3:
          currentPosition=intersectionC;
          break;
        case legNewRod4:
          currentPosition=intersectionD;
          break;
        case legSpentRod1:
          currentPosition=spentRod1;
          break;
        case legSpentRod2:
          currentPosition=spentRod2;
          break;      
        case legSpentRod3:
          currentPosition=spentRod3;
          break;
        case legSpentRod4:
          currentPosition=spentRod4;
          break; 
        default:
          noCaseFound(9);
          break;              
      }
      break;
    default:
      noCaseFound(10);
      break;
  }
  if (currentPosition==intersectionA || currentPosition==intersectionB || currentPosition==intersectionC || currentPosition==intersectionD) {
    decideDirection();
  } else {
    performTask();
  }
  updateDisplay();
}

void noCaseFound(int error) {
  Serial.println("Error: no case found");
  Serial.println(currentPosition);
  Serial.println(currentHeading);
  Serial.println(currentTask);
  errorCode=error;
  updateDisplay();
  lcd.print("!");
  while (1) {
    stopAll();
    transmitStatus(rsStopped);
  }
}

void performTaskSpentRod() {
  insertRodInStorage();
  reverse();
  turn180();
  currentTask=findSupplyRod;
  currentHeading=east;
}

void performTaskNewRod() {
  removeRodFromStorage();
  reverse();
  turn180();
  stopAll();
  lowerArm();
  currentTask=navigateToReactorDropoff;
  currentHeading=west;
}

void performTaskReactor() {
  switch (currentTask) {
    case navigateToReactorPickup:
      currentTask=removeSpentReactorRod;
      break;
    case navigateToReactorDropoff:
      currentTask=insertNewReactorRod;
      break;
    default:
      noCaseFound(13);
      break;
  }
  updateDisplay();
  switch (currentTask) {
    case removeSpentReactorRod:
      removeRodFromReactor();
      currentTask=findEmptyStorage;
      break;
    case insertNewReactorRod:
      insertRodInReactor();
      currentTask=changeTube;
      break;
    default: 
      noCaseFound(12);
      break;
  }
  reverse();
  turn180();
  stopAll();
  switch (currentTask) {
    case findEmptyStorage: raiseArm(); break;
    case changeTube: lowerArm(); break;
  }
}

void performTask() { //if code got here, bot just arived at target position
 updateDisplay();
 switch (currentPosition) {
    case reactorA:
      performTaskReactor();
      currentPosition=turnpikeA;
      currentHeading=north;
      break;
    case reactorB:
      performTaskReactor();
      currentPosition=turnpikeE;
      currentHeading=south;
      break;
    case spentRod1:
      performTaskSpentRod();
      currentPosition=legSpentRod1;
      break;
    case spentRod2:
      performTaskSpentRod();
       currentPosition=legSpentRod2;
      break;
    case spentRod3:
      performTaskSpentRod();
      currentPosition=legSpentRod3;
      break;
    case spentRod4:
      performTaskSpentRod();
       currentPosition=legSpentRod4;
      break;      
    case newRod1:
      performTaskNewRod();
      currentPosition=legNewRod1;
      break;
    case newRod2:
      performTaskNewRod();
      currentPosition=legNewRod2;
      break;
    case newRod3:
      performTaskNewRod();
      currentPosition=legNewRod3;
      break;
    case newRod4:
      performTaskNewRod();
      currentPosition=legNewRod4;
      break;     
  }
  
  setNewTargetPosition(); 
  /*
  Serial.println("Done");
  Serial.println(currentPosition);
  Serial.println(currentHeading);
  Serial.println(currentTask);
  /*
  while(1) {
    stopAll();
  }
  */
  updateDisplay();
}

void setNewTargetPosition() {
  getFullbtPacket();
  updateDisplay();
  switch (currentTask) {
    case findEmptyStorage:
      getStorageAvailability();
      currentTask=navigateToStorage;
      transmitStatus(rsDrivingToStorage);
      break;
    case changeTube:
      if (currentTube==tubeA) {
        currentTube=tubeB;
        targetPosition=reactorB;
      } else {
        currentTube=tubeA;
        targetPosition=reactorA;
      }
      currentTask=navigateToReactorPickup;
      transmitStatus(rsDrivingToReactor);
      break;
    case navigateToReactorDropoff:
       if (currentTube==tubeA) {
        targetPosition=reactorA;
      } else {
        targetPosition=reactorB;
      }  
      break;
      transmitStatus(rsDrivingToReactor);
    case findSupplyRod:  
      getSupplyAvailability();
      currentTask=getNewRod;
      break;
      transmitStatus(rsDrivingToSupply);
    default:
      noCaseFound(11);  //problem here
      break;
  }
}

void removeRodFromReactor() {
  radiationType=spentRod;
  transmitStatus(rsIdle);
  openClaw();
  delay(500);
  transmitStatus(rsGripAttemptInProgress);
  extendClaw();
  delay(700);
  closeClaw();
  transmitStatus(rsRodInGripper);
  delay(500);
  retractClaw();
  delay(1200);
  raiseArm();
  transmitStatus(rsMovingAutonomous);
}
void insertRodInReactor() {
  transmitStatus(rsIdle);
  extendClaw();
  delay(700);
  openClaw();
  transmitStatus(rsNoRodInGripper);
  delay(500);
  radiationType=noRod;
  transmitStatus(rsMovingAutonomous);
}

void removeRodFromStorage() {
  int i;
  radiationType=newRod;
  transmitStatus(rsIdle);
  openClaw();
  delay(400);
  transmitStatus(rsGripAttemptInProgress);
  extendClaw();
  delay(700);
  closeClaw();
  delay(400);
  clawRetractMotor.write(55);
  delay(600);
  openClaw();
  delay(400); 
  for (i=55;i>0;i--) {
    clawRetractMotor.write(i);  
    delay(10);
  }
  /*
  clawRetractMotor.write(30);  
  delay(300); //250
  extendClaw(); 
  delay(400); //250  */
  closeClaw();
  transmitStatus(rsRodInGripper);
  delay(400);
  retractClaw();
  delay(600);
  transmitStatus(rsMovingAutonomous);
}

void insertRodInStorage() {
  transmitStatus(rsIdle);
  extendClaw();
  delay(700);
  openClaw();
  transmitStatus(rsNoRodInGripper);
  delay(500);
  retractClaw();
  radiationType=noRod;
  transmitStatus(rsMovingAutonomous);  
}

void getStorageAvailability() {
  if (currentTube==tubeA) {
    if (!bitRead(spentRodStatus,3)) {
      targetPosition=spentRod4;
    } else if (!bitRead(spentRodStatus,2)) {
       targetPosition=spentRod3;
    } else if (!bitRead(spentRodStatus,1)) {
       targetPosition=spentRod2;
    } else if (!bitRead(spentRodStatus,0)) {
       targetPosition=spentRod1;
    } else {
      targetPosition=spentRod4;
    }
  }
  if (currentTube==tubeB) {
    if (!bitRead(spentRodStatus,0)) {
      targetPosition=spentRod1;
    } else if (!bitRead(spentRodStatus,1)) {
       targetPosition=spentRod2;
    } else if (!bitRead(spentRodStatus,2)) {
       targetPosition=spentRod3;
    } else if (!bitRead(spentRodStatus,3)) {
       targetPosition=spentRod4;
    } else {
      targetPosition=spentRod1;
    }
  }
}

void getSupplyAvailability() {
  if (currentTube==tubeA) {
    if (bitRead(supplyRodStatus,0)) {
      targetPosition=newRod1;
    } else if (bitRead(supplyRodStatus,1)) {
       targetPosition=newRod2;
    } else if (bitRead(supplyRodStatus,2)) {
       targetPosition=newRod3;
    } else if (bitRead(supplyRodStatus,3)) {
       targetPosition=newRod4;
    } else {
      targetPosition=newRod1;
    }
  }  
  if (currentTube==tubeB) {
    if (bitRead(supplyRodStatus,3)) {
      targetPosition=newRod4;
    } else if (bitRead(supplyRodStatus,2)) {
       targetPosition=newRod3;
    } else if (bitRead(supplyRodStatus,1)) {
       targetPosition=newRod2;
    } else if (bitRead(supplyRodStatus,0)) {
       targetPosition=newRod1;
    } else {
      targetPosition=newRod4;
    }
  }
}

void getFullbtPacket() {  
  if(getBTData()==btStart) {
    btlength=getBTData();    
    btMsgType=getBTData();
    btSrcAddress=getBTData();
    btDestAddress=getBTData();
    btData=getBTData();
    btChecksum=getBTData();
    if (255-(btMsgType+btSrcAddress+btDestAddress+btData+btlength)==btChecksum) { //check for correct check sum
      if (btDestAddress==broadcastAddress || btDestAddress==teamAddress) { //check for correct address
        switch (btMsgType) {
          case msgStoreAvailability: spentRodStatus=btData; break;
          case msgSupplyAvailability: supplyRodStatus=btData; break; 
          case msgStopMovement: stopMotionFlag=true; transmitStatus(rsStopped); break;
          case msgResumeMovement: transmitStatus(rsMovingAutonomous); stopMotionFlag=false; break;
        }
      }
    }
  }
}

byte getBTData() {
  if(Serial1.available()>0) {
    return Serial1.read();
  }
}

void sendBTData(byte data) {
  Serial1.write(data);
}

void transmitHeartbeat() {
  sendBTData((byte)btStart);
  sendBTData((byte)0x06);
  sendBTData((byte)msgHeartbeat);
  sendBTData((byte)teamAddress);
  sendBTData((byte)reactorCtrlAddress);
  sendBTData((byte)0x00);
  sendBTData((byte)(255 -(0x06+msgHeartbeat+teamAddress+reactorCtrlAddress+0x00)));
}

void transmitRadiationAlert() {
  byte Rdata;
  byte Rchecksum;
  if (radiationType=newRod) {
    Rdata=dataNew;
  } else {
    Rdata=dataNew;
  }
  Rchecksum= 255 -(0x06+msgRadiationAlert+teamAddress+reactorCtrlAddress+Rdata);
  sendBTData((byte)btStart);
  sendBTData((byte)0x06);
  sendBTData((byte)msgRadiationAlert);
  sendBTData((byte)teamAddress);
  sendBTData((byte)reactorCtrlAddress);  
  sendBTData((byte)Rdata);
  sendBTData((byte)Rchecksum);
}

void transmitStatus(byte rsData) {
  byte rsChecksum;
  rsChecksum= 255 -(0x06+msgRobotStatus+teamAddress+reactorCtrlAddress+rsData);
  sendBTData((byte)btStart);
  sendBTData((byte)0x06);
  sendBTData((byte)msgRobotStatus);
  sendBTData((byte)teamAddress);
  sendBTData((byte)reactorCtrlAddress);  
  sendBTData((byte)rsData);
  sendBTData((byte)rsChecksum);  
}

void updateAlertLed() {
  if (radiationType!=noRod) {
    alertLedState=!alertLedState;
      if (radiationCounter>4) {
        radiationCounter=0;
        transmitRadiationAlert();
      } else {
        radiationCounter++;
      }  
    } else {
    alertLedState=true;
  }
  digitalWrite(radiationLedPin,alertLedState);
  if (heartbeatCounter>2) {
    heartbeatCounter=0;
    transmitHeartbeat();
  } else {
    heartbeatCounter++;
  }
}


void updateDisplay() {
   lcd.clear();
   switch (currentPosition) {
    case reactorA: lcd.print("reactorA"); break;
    case reactorB: lcd.print("reactorB"); break;
    case newRod1: lcd.print("newRod1"); break;
    case newRod2: lcd.print("newRod2"); break;
    case newRod3: lcd.print("newRod3"); break;
    case newRod4: lcd.print("newRod4"); break;
    case spentRod1: lcd.print("spentRod1"); break;
    case spentRod2: lcd.print("spentRod2"); break;
    case spentRod3: lcd.print("spentRod3"); break;
    case spentRod4: lcd.print("spentRod4"); break;
    case turnpikeA: lcd.print("turnpikeA"); break;
    case turnpikeB: lcd.print("turnpikeB"); break;
    case turnpikeC: lcd.print("turnpikeC"); break;
    case turnpikeD: lcd.print("turnpikeD"); break;
    case turnpikeE: lcd.print("turnpikeE"); break;
    case intersectionA: lcd.print("intersectionA"); break;
    case intersectionB: lcd.print("intersectionB"); break;
    case intersectionC: lcd.print("intersectionC"); break;
    case intersectionD: lcd.print("intersectionD"); break;
    case legNewRod1: lcd.print("legNewRod1"); break;
    case legNewRod2: lcd.print("legNewRod2"); break;
    case legNewRod3: lcd.print("legNewRod3"); break;
    case legNewRod4: lcd.print("legNewRod4"); break;
    case legSpentRod1: lcd.print("legSpentRod1"); break;
    case legSpentRod2: lcd.print("legSpentRod2"); break;
    case legSpentRod3: lcd.print("legSpentRod3"); break;
    case legSpentRod4: lcd.print("legSpentRod4"); break;
    default: lcd.print("noCase"); break;
  }
  lcd.print("-");
  switch (currentHeading) {
    case north: lcd.print("N"); break;
    case south: lcd.print("S"); break;
    case east: lcd.print("E"); break;
    case west: lcd.print("W"); break;
    default: lcd.print("noCase"); break;
  }
  lcd.setCursor(0,1);
  switch (currentTask) {
    case navigateToReactorPickup: lcd.print("navToPick"); break;
    case removeSpentReactorRod: lcd.print("removeSpent"); break;
    case findEmptyStorage: lcd.print("findStore"); break;
    case navigateToStorage: lcd.print("navToStore"); break;
    case storeSpentRod: lcd.print("storeSpent"); break;
    case findSupplyRod: lcd.print("findSupply"); break;
    case getNewRod: lcd.print("getNewRod"); break;
    case navigateToReactorDropoff: lcd.print("navToDrop"); break;
    case insertNewReactorRod: lcd.print("insertNew"); break;
    case changeTube: lcd.print("changeTube"); break;
  }
  lcd.print("-");
  switch (currentTube) {
    case tubeA: lcd.print("A"); break;
    case tubeB: lcd.print("B"); break;
  }
  lcd.print("-");
  lcd.print(errorCode);
}
