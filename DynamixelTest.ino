
//#define INCLUDE_TIMED_TESTS
//====================================================================================================
// Kurt's Dynamixel Servo test program - This version is setup to be more generic
// in that it scans for all servos and now tries to look for both Protocol 1 and Protocol 2 servos
// This is a test, only a test...
//====================================================================================================
//============================================================================
// Global Include files
//=============================================================================

#include <dxlSerial.h>
#include "RobotsInfo.h"  // get the stuff out of the main sources...
//=============================================================================
// Options...
//=============================================================================

// Uncomment the next line if building for a Quad instead of a Hexapod.
//#define QUAD_MODE
//#define TURRET
//#define DEBUG_IO_PINS

#define DXL_BAUD 1000000 // default for AX 
//#define DXL_BAUD 57600 // default for XL servos
// V0.3 T36
#if defined(KINETISK)
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define SERVO_DIRECTION_PIN 28
#define SERVO_POWER_ENABLE_PIN 29
#define SERVO_TX_PIN 26
#define SERVO_RX_PIN  27
#else
// V0.3 T32
//V0.2
#define SERVO_DIRECTION_PIN -1
#define SERVO_POWER_ENABLE_PIN 2

//#define SERVO_DIRECTION_PIN 2
//#define SERVO_POWER_ENABLE_PIN 3
#endif
#endif

#define AX_BUS_UART Serial1
//#define VOLTAGE_ANALOG_PIN 0
//=============================================================================
// Globals
//=============================================================================
// Global objects
/* IK Engine */
word           g_wVoltage = 0;
uint8_t        g_servo_index_voltage = 0;
char           g_aszCmdLine[80];
uint8_t        g_iszCmdLine;
boolean        g_fTrackServos = false;

enum           {SERVO_NOT_FOUND = 0, SERVO_PROTOCOL1 = 1, SERVO_PROTOCOL2};
uint8_t        g_servo_protocol[255] = {SERVO_NOT_FOUND};  // What type of servos do we have????
uint8_t        g_count_servos_found = 0;

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
  while (!Serial && (millis() < 3000)) ;  // Give time for Teensy and other USB arduinos to create serial port
  Serial.begin(38400);  // start off the serial port.

  delay(250);

#ifdef SERVO_TX_PIN
  AX_BUS_UART.setTX(SERVO_TX_PIN);
#endif
#ifdef SERVO_RX_PIN
  AX_BUS_UART.setRX(SERVO_RX_PIN);
#endif

  Serial.print("Servo Direction pin: ");
  Serial.println(SERVO_DIRECTION_PIN);

  dxlInit(DXL_BAUD, &AX_BUS_UART, SERVO_DIRECTION_PIN);

#ifdef SERVO_POWER_ENABLE_PIN
  Serial.print("Servo Enable Pin: ");
  Serial.println(SERVO_POWER_ENABLE_PIN, DEC);
  pinMode(SERVO_POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(SERVO_POWER_ENABLE_PIN, HIGH);
#endif
  delay(1000);
  // Lets start of trying to locate all servos.
  FindServos();

  PrintServoVoltage();

}


//====================================================================================================
// Loop
//====================================================================================================
void loop() {
  // Output a prompt
  PrintServoVoltage();

  // lets toss any charcters that are in the input queue
  while (Serial.read() != -1)
    ;

  Serial.println("0 - All Servos off");
  Serial.println("1 - All Servos center");
  Serial.println("2 - Set Servo position [<Servo>] <Position> [<Speed>]");
//  Serial.println("3 - Set Servo Angle");
  Serial.println("4 - Get Servo Positions");
  Serial.println("5 - Find All Servos");
  Serial.println("6 - Set Servo return delay time");
  Serial.println("8 - Set ID: <old> <new>");
  Serial.println("9 - Print Servo Values");
  Serial.println("b - Baud <new baud>");
  Serial.println("t - Toggle track Servos");
  Serial.println("h - hold [<sn>]");
  Serial.println("f - free [<sn>]");
  Serial.println("S - Syncwrite all servos center");
  Serial.println("w - write <servo> <reg> <val> (<val2>...)\n\r");
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
  //    case '3':
  //      break;
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
      case 'b':
      case 'B':
        SetBaudRate();
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
      case 's':
      case 'S':
        SyncwriteCenterServos();
        break;
        
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
      case 'w':
      case 'W':
        WriteServoValues();
        break;
    }
  }
}

//====================================================================================================
void PrintServoVoltage() {
  // Lets try reading in the current voltage for the next servo we found... 
  if (g_count_servos_found == 0) return; // no servos found
  g_servo_index_voltage++;    // will wrap around...
  uint8_t sanity_test_count = 0;
  while (!g_servo_protocol[g_servo_index_voltage]) {
    g_servo_index_voltage++;
    sanity_test_count--;
    if (sanity_test_count == 0) return;
  } 
  word wNewVoltage;
  if (g_servo_protocol[g_servo_index_voltage] == SERVO_PROTOCOL1) {
    wNewVoltage = ax12GetRegister(g_servo_index_voltage, AX_PRESENT_VOLTAGE, 1);
  } else {
    wNewVoltage = dxlP2GetRegisters(g_servo_index_voltage, DXL_X_PRESENT_INPUT_VOLTAGE, 2);

  }
  if (wNewVoltage != g_wVoltage) {
    g_wVoltage = wNewVoltage;
    Serial.print("Servo: ");
    Serial.print(g_servo_index_voltage, DEC);
    Serial.print(" Voltage in 10ths: ");
    Serial.println(g_wVoltage, DEC);
  }
}


//====================================================================================================
// Helper function to read in a command line
uint8_t GetCommandLine(void) {
  int ch;
  uint8_t ich = 0;
  g_iszCmdLine = 0;

  for (;;) {
    // throw away any thing less than CR character...
    ch = Serial.read();
    if ((ch >= 10) && (ch <= 15)) {
      g_aszCmdLine[ich] = 0;
      return ich;
    }
    if (ch != -1)
      g_aszCmdLine[ich++] = ch;

    if (g_fTrackServos)
      TrackServos(false);
  }
}

//====================================================================================================

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

void FindServos(void) {

  g_count_servos_found = 0;
  int w;
  Serial.println("\nSearch for all servos");
  Serial.println("Begin Protocol 1: ");
  for (int i = 0; i < 254; i++) {
    w = ax12GetRegister(i, AX_PRESENT_POSITION_L, 2 );
    if (w != (int) - 1) {
      g_servo_protocol[i] = SERVO_PROTOCOL1;
      g_count_servos_found++;
      Serial.print(i, DEC);
      Serial.print(" - ");
      Serial.println(w, DEC);
    } else {
      g_servo_protocol[i] = SERVO_NOT_FOUND;
    }
    delay (5);
  }
  Serial.println("Done");
  Serial.println("Begin Protocol 2: ");
  for (int i = 0; i < 254; i++) {
    uint32_t model_firmware = dxlP2Ping(i);
    if (model_firmware) {
      if (g_servo_protocol[i] == SERVO_PROTOCOL1) {
        Serial.println("Found both Protocol 1 and Protocol 2 Servo with same ID");
      } else {
        g_count_servos_found++;
      }
      g_servo_protocol[i] = SERVO_PROTOCOL2;
      Serial.print(i, DEC);
      Serial.print(", Model:");
      Serial.print(model_firmware & 0xffff, HEX);
      Serial.print(", firmware: ");
      Serial.print(model_firmware >> 16, HEX);
      w = dxlP2GetRegisters(i, DXL_X_PRESENT_POSITION, 4 );
      Serial.print(" : ");
      Serial.println(w, DEC);
    }
    delay (5);
  }
  Serial.println("Done");
}

//=======================================================================================
void AllServosOff(void) {
  // Tell protocol 1 servos to turn off their Torque
  ax12SetRegister(0xfe, AX_TORQUE_ENABLE, 0x0);

  // Lets tell all Protocol 2 servos to turn their torque off as well
  dxlP2SetRegisters(0xfe, DXL_X_TORQUE_ENABLE, 0x0, 1);
}
//=======================================================================================
void AllServosCenter(void) {
  for (int i = 0; i < 255; i++) {
    if (g_servo_protocol[i] == SERVO_PROTOCOL1) {
      // See if this turns the motor off and I can turn it back on...
      ax12SetRegister(i, AX_TORQUE_ENABLE, 0x1);
      ax12ReadPacket(6);  // get the response...
      ax12SetRegister2(i, AX_GOAL_POSITION_L, 0x1ff);
      ax12ReadPacket(6);  // get the response...
    } else if (g_servo_protocol[i] == SERVO_PROTOCOL2) {
      dxlP2SetRegisters(i, DXL_X_TORQUE_ENABLE, 0x1);
      dxlP2ReadPacket();  // get the response...
      dxlP2SetRegisters(i, DXL_X_GOAL_POSITION, 2047, 4);
      dxlP2ReadPacket();  // get the response...
    }
  }
}

//=======================================================================================
uint8_t packet[256];

void SyncwriteCenterServos() {
  // First lets setup to output a protocol 1 version...
  uint8_t count_protocol1 = 0;
  uint8_t count_protocol2 = 0;
  uint8_t *pb = packet;
  
  for (int i = 0; i < 255; i++) {
    if (g_servo_protocol[i] == SERVO_PROTOCOL1) {
      count_protocol1++;
      *pb++ = i;  // output servo number
      *pb++ = 0xff;  // output the center position
      *pb++ = 0x01;
    } else if (g_servo_protocol[i] == SERVO_PROTOCOL2) {
      count_protocol2++;
    }
  }
  if (count_protocol1) {
    Serial.printf("SW Count protocol1 %d\n", count_protocol1);
    dxlP1SyncWrite(count_protocol1, AX_GOAL_POSITION_L, 2, packet);
  }

  if (count_protocol2) {
    Serial.printf("SW Count protocol2 %d\n", count_protocol2);
    pb = packet;
    for (int i = 0; i < 255; i++) {
      if (g_servo_protocol[i] == SERVO_PROTOCOL2) {
        *pb++ = i;  // output servo number
        *pb++ = 0xff;  // output the center position 
        *pb++ = 0x07;  //
        *pb++ = 0x00;  //
        *pb++ = 0x00;  //
      }
    }
    dxlP2SyncWrite(count_protocol2, DXL_X_GOAL_POSITION, 4, packet);
  } 

}



//=======================================================================================
void HoldOrFreeServos(byte fHold) {
  word iServo;
  if (!FGetNextCmdNum(&iServo)) {
    for (int i = 0; i < 255; i++) {
      if (g_servo_protocol[i] == SERVO_PROTOCOL1) {
        // See if this turns the motor off and I can turn it back on...
        ax12SetRegister(i, AX_TORQUE_ENABLE, fHold);
        ax12ReadPacket(6);  // get the response...
      } else if (g_servo_protocol[i] == SERVO_PROTOCOL2) {
        dxlP2SetRegisters(i, DXL_X_TORQUE_ENABLE, fHold);
        dxlP2ReadPacket();  // get the response...
      }
    }
  }
  else {
    if (g_servo_protocol[iServo] == SERVO_PROTOCOL1) {
      ax12SetRegister(iServo, AX_TORQUE_ENABLE, fHold);
      ax12ReadPacket(6);  // get the response...
    } else if (g_servo_protocol[iServo] == SERVO_PROTOCOL2) {
      dxlP2SetRegisters(iServo, DXL_X_TORQUE_ENABLE, fHold);
      dxlP2ReadPacket();  // get the response...
    }
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
      if (g_servo_protocol[g_bServoID] == SERVO_PROTOCOL1) {
        ax12SetRegister2(g_bServoID, AX_GOAL_SPEED_L, g_wServoGoalSpeed);
        ax12ReadPacket(6);  // get the response...
      }
      Serial.print("Goal Speed: ");
      Serial.print(g_wServoGoalSpeed, DEC);
    }
  }
  else
    g_wServoGoalPos = w1;  // Only 1 paramter so assume it is the new position

  // Now lets try moving that servo there
  if (g_servo_protocol[g_bServoID] == SERVO_PROTOCOL1) {
    ax12SetRegister2(g_bServoID, AX_GOAL_POSITION_L, g_wServoGoalPos);
    ax12ReadPacket(6);  // get the response...
  } else if (g_servo_protocol[g_bServoID] == SERVO_PROTOCOL2) {
    dxlP2SetRegisters(g_bServoID, DXL_X_TORQUE_ENABLE, 1);
    dxlP2ReadPacket();  // get the response...
    dxlP2SetRegisters(g_bServoID, DXL_X_GOAL_POSITION, g_wServoGoalPos, 4);
    dxlP2ReadPacket();  // get the response...
  }
  Serial.print(" ID: ");
  Serial.print(g_bServoID, DEC);
  Serial.print(" ");
  Serial.println(g_wServoGoalPos, DEC);
}

//=======================================================================================
bool IsValidServo(uint8_t servo_id) {
  if (g_servo_protocol[servo_id])
    return true;  // was found before. 

  // First lets try Protocol1 ping
  if (dxlP1Ping(servo_id)) {
    g_servo_protocol[servo_id] = SERVO_PROTOCOL1;
    g_count_servos_found++;
    return true; 
  } 
  if (dxlP2Ping(servo_id)) {
    g_servo_protocol[servo_id] = SERVO_PROTOCOL2;
    g_count_servos_found++;
    return true; 
  }
  return false;
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

  if (!IsValidServo(w1)) {
    Serial.print("Servo: ");
    Serial.print(w1, DEC);
    Serial.println("Was not found");
    return;
  }

  // Now lets try moving that servo there
  if (g_servo_protocol[w1] == SERVO_PROTOCOL1) {
    ax12SetRegister(w1, AX_RETURN_DELAY_TIME, w2);
    ax12ReadPacket(6);  // get the response...
  } else {
    dxlP2SetRegisters(w1, DXL_X_RETURN_DELAY_TIME, w2);
    dxlP2ReadPacket();
  }
}


//=======================================================================================
void SetServoID(void) {
  word wIDFrom;
  word wIDTo;

  if (!FGetNextCmdNum(&wIDFrom))
    return;    // no parameters so bail.

  if (!FGetNextCmdNum(&wIDTo))
    return;    // no parameters so bail.


  Serial.print("Set Servo ID From: ");
  Serial.print(wIDFrom, DEC);
  Serial.print(" To: ");
  Serial.println(wIDTo, DEC);

  if (!IsValidServo(wIDFrom)) {
    Serial.print("Servo: ");
    Serial.print(wIDFrom, DEC);
    Serial.println("Was not found");
    return;
  }

  // Now lets try moving that servo there
  if (g_servo_protocol[wIDFrom] == SERVO_PROTOCOL1) {
    ax12SetRegister(wIDFrom, AX_ID, wIDTo);
    ax12ReadPacket(6);  // get the response...
  } else {
    dxlP2SetRegisters(wIDFrom, DXL_X_ID, wIDTo);
    dxlP2ReadPacket();
  }
}


//=======================================================================================
void GetServoPositions(void) {

  unsigned long ulBefore;
  unsigned long ulDelta;
  if (!g_count_servos_found) {
    Serial.println("Previous Find Servos failed to locate any servos: so retry");
    FindServos();
    return;
  }

  int w;
  for (int i = 0; i < 255; i++) {
    if (g_servo_protocol[i]) {
      Serial.print(i, DEC);
      Serial.print(":");
      ulBefore = micros();
      if (g_servo_protocol[i] == SERVO_PROTOCOL1) {
        w = ax12GetRegister(i, AX_PRESENT_POSITION_L, 2 );
        ulDelta = micros() - ulBefore;
        Serial.print(w, DEC);
        Serial.print(" ");
        Serial.print(ulDelta, DEC);
        Serial.print(" ");
        Serial.println(ax12GetRegister(i, AX_RETURN_DELAY_TIME, 1), DEC);
        if (w == (int)0xffff) {
          Serial.print("   Retry: ");
          w = ax12GetRegister(i, AX_PRESENT_POSITION_L, 2 );
          Serial.println(w, DEC);
        }
      } else if (g_servo_protocol[i] == SERVO_PROTOCOL2) {
        w = dxlP2GetRegisters(i, DXL_X_PRESENT_POSITION, 4 );
        ulDelta = micros() - ulBefore;
        Serial.print(w, DEC);
        Serial.print(" ");
        Serial.print(ulDelta, DEC);
        Serial.print(" ");
        Serial.println(dxlP2GetRegisters(i, DXL_X_RETURN_DELAY_TIME, 1), DEC);
      }
    }
    delay(5);
  }
}


//=======================================================================================
int g_asPositionsPrev[NUM_SERVOS];
int g_asMins[NUM_SERVOS];
int g_asMaxs[NUM_SERVOS];

void TrackServos(boolean fInit) {

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
        if (abs(w - g_asPositionsPrev[i]) > 3) {
          Serial.print(IKPinsNames[i]);
          Serial.print("(");
          Serial.print((byte)pgm_read_byte(&pgm_axdIDs[i]), DEC);
          Serial.print("):");
          Serial.print(w, DEC);
          Serial.print("(");
          Serial.print((((long)(w - 512)) * 375L) / 128L, DEC);
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
    Serial.print((((long)(g_asMins[i] - 512)) * 375L) / 128L, DEC);
    Serial.print(") ");

    Serial.print(g_asMaxs[i], DEC);
    Serial.print("(");
    Serial.print((((long)(g_asMaxs[i] - 512)) * 375L) / 128L, DEC);
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

  if (g_servo_protocol[wID] == SERVO_PROTOCOL1) {
    Serial.println("Protocol 1");
    for (int i = 0; i < 50; i++) {
      Serial.print(i, DEC);
      Serial.print(":");
#ifdef DEBUG_IO_PINS
      digitalWrite(A2, HIGH);
#endif
      w = ax12GetRegister(wID, i, 1 );
#ifdef DEBUG_IO_PINS
      digitalWrite(A2, LOW);
      if (w == (word) - 1)
        digitalWrite(A3, !digitalRead(A3));
#endif
      Serial.print(w, HEX);
      Serial.print(" ");
      if ((i % 10) == 9)
        Serial.println("");
      Serial.flush();  // try to avoid any interrupts while processing.
      delay(5);
    }
  } else if (g_servo_protocol[wID] == SERVO_PROTOCOL2) {
    Serial.println("Protocol 2");
    for (int i = 0; i < 147; i++) {
      if ((i & 0xf) == 0) {
        Serial.print(i, HEX);
        Serial.print(":");
      }
      w = dxlP2GetRegisters(wID, i, 1 );
      Serial.print(w, HEX);
      Serial.print(" ");
      if ((i & 0xf) == 0xf)
        Serial.println();
      delay(1);
    }
    Serial.println();
  }
}
//=======================================================================================
void WriteServoValues() {
  word wID;
  word wReg;
  word wVal;

  if (!FGetNextCmdNum(&wID))
    return;    // no parameters so bail.

  if (!FGetNextCmdNum(&wReg))
    return;    // no parameters so bail.

  while (FGetNextCmdNum(&wVal)) {
    Serial.print("Write register ID: ");
    Serial.print(wID, DEC);
    Serial.print(" Reg: ");
    Serial.print(wReg, DEC);
    Serial.print(" Val: ");
    Serial.print(wVal, DEC);
    if (g_servo_protocol[wID] == SERVO_PROTOCOL1) {
      // See if this turns the motor off and I can turn it back on...
      ax12SetRegister(wID, wReg, wVal);
      if (ax12ReadPacket(6)) // get the response...
        Serial.println(" Success");
      else
        Serial.println(" Failed");  
      
    } else if (g_servo_protocol[wID] == SERVO_PROTOCOL2) {
      dxlP2SetRegisters(wID, wReg, wVal);
      int resp_packet_size = dxlP2ReadPacket();  // get the response...
      if (resp_packet_size > 0) {
        if (uint8_t last_error = dxlGetLastError()) {
          Serial.print(" Error: ");
          Serial.println(last_error, HEX);
        } else {
          Serial.println(" Success");
        }
      } else {
        Serial.println(" Failed");
      }
    }
    wReg++;   // get to the next reg
  }
}


//=======================================================================================
boolean GetMultax12Registers(int id, int regstart, int length, uint8_t *pab) {
  uint8_t *pbT;
  setTX(id);
  // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM
  int checksum = ~((id + 6 + regstart + length) % 256);
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
  if (ax12ReadPacket(length + 6) > 0) {
    pbT = &ax_rx_buffer[5];
    while (length--)
      *pab++ = *pbT++;    // copy the data
    return true;
  }
  return false;
}

void SetBaudRate()
{
  word wBaud;

  if (!FGetNextCmdNum(&wBaud))
    return;    // no parameters so bail.
  Serial.print("Setting Baud to: ");
  Serial.println(wBaud);
  dxlEnd(); // close out the current setup
  dxlInit(wBaud, &AX_BUS_UART, SERVO_DIRECTION_PIN);
  Serial.println("Doing new Servo Scan");
  FindServos();
}

