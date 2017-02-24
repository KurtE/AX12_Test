
//#define INCLUDE_TIMED_TESTS
//====================================================================================================
// Kurts Test program to try out different ways to manipulate the AX12 servos on the PhantomX
// This is a test, only a test...  
//====================================================================================================
//============================================================================
// Global Include files
//=============================================================================
//#define USE_BIOLOID_SERIAL
// Default to BioloidSerial on Teensy
#if defined(KINETISK)
#define USE_BIOLOID_SERIAL
#endif
#ifdef USE_BIOLOID_SERIAL
#include <ax12Serial.h>
#include <BioloidSerial.h>
#else
#include <ax12.h>
#include <BioloidController.h>
#endif
//=============================================================================
// Options...
//=============================================================================

// Uncomment the next line if building for a Quad instead of a Hexapod.
//#define QUAD_MODE
//#define TURRET
//#define DEBUG_IO_PINS

//V0.2
//#define SERVO_DIRECTION_PIN -1
//#define SERVO_POWER_ENABLE_PIN 2


// V0.3
//#define SERVO_DIRECTION_PIN 2
//#define SERVO_POWER_ENABLE_PIN 3

#define AX_BUS_UART Serial1
//#define VOLTAGE_ANALOG_PIN 0
#define SOUND_PIN 1
#define SERVO1_SPECIAL  19     // We wish to reserve servo 1 so we can see servo reset

//=============================================================================
// Define differnt robots..
//=============================================================================

// Constants
/* Servo IDs */
#define     RF_COXA       2
#define     RF_FEMUR      4
#define     RF_TIBIA      6

#define     RM_COXA      14
#define     RM_FEMUR     16
#define     RM_TIBIA     18

#define     RR_COXA       8
#define     RR_FEMUR     10
#define     RR_TIBIA     12

#ifdef SERVO1_SPECIAL
#define     LF_COXA       19
#else
#define     LF_COXA       1
#endif
#define     LF_FEMUR      3
#define     LF_TIBIA      5

#define     LM_COXA      13
#define     LM_FEMUR     15
#define     LM_TIBIA     17

#define     LR_COXA       7
#define     LR_FEMUR      9
#define     LR_TIBIA     11

#ifdef TURRET
#define     TURRET_ROT    20
#define     TURRET_TILT   21
#endif

static const byte pgm_axdIDs[] PROGMEM = {
  LF_COXA, LF_FEMUR, LF_TIBIA,    
#ifndef QUAD_MODE
  LM_COXA, LM_FEMUR, LM_TIBIA,    
#endif  
  LR_COXA, LR_FEMUR, LR_TIBIA,
  RF_COXA, RF_FEMUR, RF_TIBIA, 
#ifndef QUAD_MODE
  RM_COXA, RM_FEMUR, RM_TIBIA,    
#endif
  RR_COXA, RR_FEMUR, RR_TIBIA
#ifdef TURRET
  , TURRET_ROT, TURRET_TILT
#endif
};    

#define NUM_SERVOS ((int)(sizeof(pgm_axdIDs)/sizeof(pgm_axdIDs[0])))
const char* IKPinsNames[] = {
  "LFC","LFF","LFT",
#ifndef QUAD_MODE
  "LMC","LMF","LMT",
#endif  
  "LRC","LRF","LRT",
  "RFC","RFF","RFT",
#ifndef QUAD_MODE
  "RMC","RMF","RMT",
#endif  
  "RRC","RRF","RRT",
#ifdef TURRET
  "T-ROT", "T-TILT"
#endif
};
//=============================================================================
// Globals
//=============================================================================
// Global objects
/* IK Engine */
#ifdef USE_BIOLOID_SERIAL
BioloidControllerEx bioloid = BioloidControllerEx(); 
#else
BioloidController bioloid = BioloidController(1000000);  // may use or not... may go direct to AX12// other globals.
#endif
word           g_wVoltage;
char           g_aszCmdLine[80];
uint8_t        g_iszCmdLine;
boolean        g_fTrackServos = false;

// Values to use for servo position...
byte          g_bServoID;
word          g_wServoGoalPos;
word          g_wServoGoalSpeed;

#ifndef SERVO_DIRECTION_PIN
#define SERVO_DIRECTION_PIN -1
#endif

//====================================================================================================
// Setup 
//====================================================================================================
void setup() {
#ifdef USE_BIOLOID_SERIAL
  while (!Serial && (millis() < 3000)) ;  // Give time for Teensy and other USB arduinos to create serial port
#endif
  Serial.begin(38400);  // start off the serial port.  

  delay(250);
#ifdef SOUND_PIN
  pinMode(SOUND_PIN, OUTPUT);
  digitalWrite(SOUND_PIN, LOW);
#endif
  
#ifdef DEBUG_IO_PINS
  pinMode(4, OUTPUT);
#endif
  delay(250);

#ifdef USE_BIOLOID_SERIAL
  bioloid.begin(1000000, &AX_BUS_UART, SERVO_DIRECTION_PIN);
#endif
  bioloid.poseSize = NUM_SERVOS;

#ifdef SERVO_POWER_ENABLE_PIN
  pinMode(SERVO_POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(SERVO_POWER_ENABLE_PIN, HIGH);
#endif  
  delay(1000);
  Serial.print("System Voltage in 10ths: ");
  Serial.println(g_wVoltage = ax12GetRegister(LF_COXA, AX_PRESENT_VOLTAGE, 1), DEC);

#ifdef DEBUG_IO_PINS
  pinMode(A1, OUTPUT);
  digitalWrite(A1, HIGH);

  // This probably should be simply the SERVO_POWER_ENABLE_PIN
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
#endif
  
}


//====================================================================================================
// Loop
//====================================================================================================
void loop() {
  // Output a prompt
  word wNewVoltage = ax12GetRegister(LF_COXA, AX_PRESENT_VOLTAGE, 1);
  if (wNewVoltage != g_wVoltage) {
    g_wVoltage = wNewVoltage;
    Serial.print("System Voltage in 10ths: ");
    Serial.println(g_wVoltage, DEC);
  }

  // lets toss any charcters that are in the input queue
  while(Serial.read() != -1) 
    ;

  Serial.println("0 - All Servos off");
  Serial.println("1 - All Servos center");
  Serial.println("2 - Set Servo position [<Servo>] <Position> [<Speed>]");
  Serial.println("3 - Set Servo Angle");
  Serial.println("4 - Get Servo Positions");
  Serial.println("5 - Find All Servos");
  Serial.println("6 - Set Servo return delay time");
  Serial.println("8 - Set ID: <old> <new>");
  Serial.println("9 - Print Servo Values");
  Serial.println("t - Toggle track Servos");
  Serial.println("h - hold [<sn>]");
  Serial.println("f - free [<sn>]"); 
  Serial.print(":");
  Serial.flush();  // make sure the complete set of prompts has been output...  
  // Get a command
  if (GetCommandLine()) {
    Serial.println("");
    Serial.print("Cmd: ");
    Serial.println(g_aszCmdLine);
    g_iszCmdLine = 1;  // skip over first byte...
    switch (g_aszCmdLine[0]) {
    case '0':
      AllServosOff();
      break;
    case '1':
      AllServosCenter();
      break;
    case '2':
      SetServoPosition();  
      break;
    case '3':
      break;
    case '4':
      GetServoPositions();
      break;
    case '5':
      FindServos();
      break;
    case '6':
      SetServoReturnDelayTime();
      break;  
    case '8':
      SetServoID();
      break;
    case '9':
      PrintServoValues();
      break;
    case 'f':
    case 'F':
      HoldOrFreeServos(0);
      break;
    case 'h':
    case 'H':
      HoldOrFreeServos(1);
      break;
#if INCLUDE_TIMED_TESTS
    case 'm':
      TimedMove2();
      break;
    case 'n':
      TimedMove3();
      break;
#endif

    case 't':
    case 'T':
      g_fTrackServos = !g_fTrackServos;
      if (g_fTrackServos) {
        Serial.println("Tracking On");
        TrackServos(true);  // call to initialize all of the positions.
      }
      else
        Serial.println("Tracking Off");
      TrackPrintMinsMaxs();
      break;
    }
  }
}

// Helper function to read in a command line
uint8_t GetCommandLine(void) {
  int ch;
  uint8_t ich = 0;
  g_iszCmdLine = 0;

  for(;;) {
    // throw away any thing less than CR character...
    ch = Serial.read();
    if ((ch >= 10) && (ch <=15)) {
      g_aszCmdLine[ich] = 0;
      return ich;
    }    
    if (ch != -1) 
      g_aszCmdLine[ich++] = ch;

    if (g_fTrackServos)
      TrackServos(false);
  }
}

//
boolean FGetNextCmdNum(word *pw ) {
  // Skip all leading num number characters...
  while ((g_aszCmdLine[g_iszCmdLine] < '0') || (g_aszCmdLine[g_iszCmdLine] > '9')) {
    if (g_aszCmdLine[g_iszCmdLine] == 0)
      return false;  // end of the line...
    g_iszCmdLine++;  
  }
  *pw = 0;
  while ((g_aszCmdLine[g_iszCmdLine] >= '0') && (g_aszCmdLine[g_iszCmdLine] <= '9')) {
    *pw = *pw * 10 + (g_aszCmdLine[g_iszCmdLine] - '0');
    g_iszCmdLine++;
  }
  return true;
}

//=======================================================================================
void AllServosOff(void) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    ax12SetRegister(pgm_read_byte(&pgm_axdIDs[i]), AX_TORQUE_ENABLE, 0x0);
    ax12ReadPacket(6);  // git the response...
  }
}
//=======================================================================================
void AllServosCenter(void) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    // See if this turns the motor off and I can turn it back on...
    ax12SetRegister(pgm_read_byte(&pgm_axdIDs[i]), AX_TORQUE_ENABLE, 0x1);
    ax12ReadPacket(6);  // git the response...
    ax12SetRegister2(pgm_read_byte(&pgm_axdIDs[i]), AX_GOAL_POSITION_L, 0x1ff);
    ax12ReadPacket(6);  // git the response...
  }
}
//=======================================================================================
void HoldOrFreeServos(byte fHold) {
  word iServo;

  if (!FGetNextCmdNum(&iServo)) {
    // All servos...
    for (int i = 0; i < NUM_SERVOS; i++) {
      ax12SetRegister(pgm_read_byte(&pgm_axdIDs[i]), AX_TORQUE_ENABLE, fHold);
      ax12ReadPacket(6);  // git the response...
    }
  } 
  else {
    ax12SetRegister(iServo, AX_TORQUE_ENABLE, fHold);
    ax12ReadPacket(6);  // git the response...
  }
}

//=======================================================================================

//=======================================================================================
void SetServoPosition(void) {
  word w1;
  word w2;

  if (!FGetNextCmdNum(&w1))
    return;    // no parameters so bail.

  Serial.println("Set Servo Position"); 
  if (FGetNextCmdNum(&w2)) {  // We have at least 2 parameters
    g_bServoID = w1;    // So first is which servo
    g_wServoGoalPos = w2;
    if (FGetNextCmdNum(&w2)) {  // We have at least 3 parameters
      g_wServoGoalSpeed = w2;  
      ax12SetRegister2(g_bServoID, AX_GOAL_SPEED_L, g_wServoGoalSpeed);
      ax12ReadPacket(6);  // git the response...
      Serial.print("Goal Speed: ");
      Serial.print(g_wServoGoalSpeed, DEC);
    }
  } 
  else 
    g_wServoGoalPos = w1;  // Only 1 paramter so assume it is the new position

  // Now lets try moving that servo there   
  ax12SetRegister2(g_bServoID, AX_GOAL_POSITION_L, g_wServoGoalPos);
  ax12ReadPacket(6);  // git the response...
  Serial.print(" ID: ");
  Serial.print(g_bServoID, DEC);
  Serial.print(" ");
  Serial.println(g_wServoGoalPos, DEC);
}  

//=======================================================================================
void SetServoReturnDelayTime(void) {
  word w1;
  word w2;

  if (!FGetNextCmdNum(&w1))
    return;    // no parameters so bail.

  if (!FGetNextCmdNum(&w2))
    w2 = 0;   // we will default to 0 (our desired)

  Serial.print("Set Servo ID: ");
  Serial.print(w1, DEC);
  Serial.print(" return delay time: ");
  Serial.println(w2, DEC);

  // Now lets try moving that servo there   
  ax12SetRegister(w1, AX_RETURN_DELAY_TIME, w2);
  ax12ReadPacket(6);  // get the response...
}  



//=======================================================================================
void SetServoID(void) {
  word w1;
  word w2;

  if (!FGetNextCmdNum(&w1))
    return;    // no parameters so bail.

  if (!FGetNextCmdNum(&w2))
    return;    // no parameters so bail.

  Serial.print("Set Servo ID From: ");
  Serial.print(w1, DEC);
  Serial.print(" To: ");
  Serial.println(w2, DEC);

  // Now lets try moving that servo there   
  ax12SetRegister(w1, AX_ID, w2);
  ax12ReadPacket(6);  // git the response...
}  


void WaitForMoveToComplete(word wID) {
  do {
    //    delay(1);
  } 
  while (ax12GetRegister(wID, AX_MOVING, 1));
}


//=======================================================================================
void GetServoPositions(void) {

  unsigned long ulBefore;
  unsigned long ulDelta;
  bioloid.readPose();
  int w;
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print((byte)pgm_read_byte(&pgm_axdIDs[i]), DEC);
    Serial.print(":");
    ulBefore = micros();
    w = ax12GetRegister(pgm_read_byte(&pgm_axdIDs[i]), AX_PRESENT_POSITION_L, 2 );
    ulDelta = micros() - ulBefore;
    Serial.print(w, DEC);
    Serial.print(" ");
    Serial.print(ulDelta, DEC);
    Serial.print(" ");
    Serial.println(ax12GetRegister(pgm_read_byte(&pgm_axdIDs[i]), AX_RETURN_DELAY_TIME, 1), DEC);

    if (w == 0xffff) {
      Serial.print("   Retry: ");
      w = ax12GetRegister(pgm_read_byte(&pgm_axdIDs[i]), AX_PRESENT_POSITION_L, 2 );
      Serial.println(w, DEC);
    }    
    delay (100);
  }
}


//=======================================================================================
void FindServos(void) {

  bioloid.readPose();
  int w;
  Serial.println("Begin: ");
  for (int i = 0; i < 254; i++) {
    w = ax12GetRegister(i, AX_PRESENT_POSITION_L, 2 );
    if (w != (int)-1) {
      Serial.print(i, DEC);
      Serial.print(" - ");
      Serial.println(w, DEC);
    }    
    delay (100);
  }
  Serial.println("Done");
}
//=======================================================================================
int g_asPositionsPrev[NUM_SERVOS];
int g_asMins[NUM_SERVOS];
int g_asMaxs[NUM_SERVOS];

void TrackServos(boolean fInit) {

  bioloid.readPose();
  int w;
  bool fSomethingChanged = false;
  for (int i = 0; i < NUM_SERVOS; i++) {
    w = ax12GetRegister(pgm_read_byte(&pgm_axdIDs[i]), AX_PRESENT_POSITION_L, 2 );
    if (fInit) {
      g_asMins[i] = w;
      g_asMaxs[i] = w;
    }
    if (w != g_asPositionsPrev[i]) {
      if (!fInit) {
        // only print if we moved more than some delta...
        if (abs(w-g_asPositionsPrev[i]) > 3) {
          Serial.print(IKPinsNames[i]);
          Serial.print("(");
          Serial.print((byte)pgm_read_byte(&pgm_axdIDs[i]), DEC);
          Serial.print("):");
          Serial.print(w, DEC);
          Serial.print("(");
          Serial.print((((long)(w-512))*375L)/128L, DEC);
          Serial.print(") ");
          fSomethingChanged = true;
        }
      }
      g_asPositionsPrev[i] = w;
      if (g_asMins[i] > w)
        g_asMins[i] = w;

      if (g_asMaxs[i] < w)
        g_asMaxs[i] = w;
    }  
  }
  if (fSomethingChanged)
    Serial.println();
}

void TrackPrintMinsMaxs(void) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print((byte)pgm_read_byte(&pgm_axdIDs[i]), DEC);
    Serial.print(":");
    Serial.print(g_asMins[i], DEC);
    Serial.print("(");
    Serial.print((((long)(g_asMins[i]-512))*375L)/128L, DEC);
    Serial.print(") ");

    Serial.print(g_asMaxs[i], DEC);
    Serial.print("(");
    Serial.print((((long)(g_asMaxs[i]-512))*375L)/128L, DEC);
    Serial.println(")");
  }
}


//=======================================================================================
void PrintServoValues(void) {

  word wID;
  word w;
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  if (!FGetNextCmdNum(&wID))
    return;
  for (int i = 0; i < 50; i++) {
    Serial.print(i, DEC);
    Serial.print(":");
#ifdef DEBUG_IO_PINS
    digitalWrite(A2, HIGH);
#endif
    w = ax12GetRegister(wID, i, 1 );
#ifdef DEBUG_IO_PINS
    digitalWrite(A2, LOW);
    if (w == (word)-1)
      digitalWrite(A3, !digitalRead(A3));
#endif
    Serial.print(w, HEX);
    Serial.print(" ");
    if ((i%10) == 9)
      Serial.println("");
    Serial.flush();  // try to avoid any interrupts while processing.
    delay(5);
  }    
}
//=======================================================================================


//=======================================================================================
boolean GetMultax12Registers(int id, int regstart, int length, uint8_t *pab){  
  uint8_t *pbT;  
  setTX(id);
  // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
  int checksum = ~((id + 6 + regstart + length)%256);
  ax12writeB(0xFF);
  ax12writeB(0xFF);
  ax12writeB(id);
  ax12writeB(4);    // length
  ax12writeB(AX_READ_DATA);
  ax12writeB(regstart);
  ax12writeB(length);
  ax12writeB(checksum);  
  setRX(id);    
  // Should verify size of data actually read...
  if(ax12ReadPacket(length + 6) > 0){
    pbT = &ax_rx_buffer[5];
    while (length--)
      *pab++ = *pbT++;    // copy the data
    return true;   
  }
  return false;
}













