/* --------------------------------------------------------------------
This file is part of the master thesis "A Shape-Changing Display for 
Ambient Notifications".

This Sketch is used to control a LED stripe with 44 WS2812B RGB LEDs.
- An Arduino Nano is adressed by I2C https://www.arduino.cc/en/Reference/Wire
  see "I2C Adressing Schema"
- A Raspberry Pi 3 works as I2C Master

Included libraries:
- Arduino Wire
- Arduino Animation Library (modified)
- Adafruit NeoPixel (WS2812B)
- Debugging Tools

The Arduino ALA library is free software: you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

The Arduino ALA library is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with The Arduino ALA library.  If not, see <http://www.gnu.org/licenses/>.

--------------------------------------------------------------------
I2C Adressing Schema
--------------------------------------------------------------------

- Columns: A, B, C, ...
- Rows: 1, 2, 3, ...
- Bar adressing: A1, B2, C3, ...

I2C Master
- 0x0f

Bar/ I2C Receiver
- Motor Controller Adress: even numbers
- LED Controller Adress: odd numbers

- e.g. Row 1 = 0x1#: 
  - Bar A1: 0x10 (Motor), 0x11 (LED)
  - Bar B1: 0x12 (Motor), 0x13 (LED)
  - Bar C1: 0x14 (Motor), 0x15 (LED)
  
- e.g. Bar A1:

        [A   ][B   ][C   ][D   ][E   ][F   ][       ][RPI]
        0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
[  ] 00:          -- -- -- -- -- -- -- -- -- -- -- -- 0f 
[ 1] 10: 10 11 -- -- -- -- -- -- -- -- -- -- -- -- -- --
[ 2] 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
[ 3] 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
[ 4] 40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
[ 5] 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
[ 6] 60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
[  ] 70: -- -- -- -- -- -- -- --
-------------------------------------------------------------------- */

#include <Wire.h>               // I2C https://www.arduino.cc/en/Tutorial/MasterWriter
#include <Stepper-MotorControl.h>
#include <Debugger.h>           // Debugging Tool
#include <ctype.h>
#include <OneWire.h>

////////////////////////////////////////////////////////////////////////////////

#define MASTER_ADDRESS  0x0f    // Raspberry Pi, Decimal: 15
#define SLAVE_ADDRESS   0x10    // MotorController 0x10, Decimal: 16

#define BAUD_RATE       9600    // Serial Baud Rate
#define I2C_STATUS_PIN  2       // D2


#define JUMPER_PIN    A1

////////////////////////////////////////////////////////////////////////////////
// Parameter

MotorControl ctl;
Debugger  debug;                   // Debugger

boolean _serialActive   = false;  // MotorController recognized i2c-jumper
boolean _i2cActive      = false;

boolean _setupComplete  = false;
long    _ts             = 0;
unsigned long lastTimeStamp;

unsigned int _targetPosition = -1;
String _stepMode = "";
boolean _hasNewTargetPosition = false;

////////////////////////////////////////////////////////////////////////////////

OneWire  ds(7);  // 1-Wire Bus Pin

const int _maxTokenIDs = 20;

byte tokenIDs[_maxTokenIDs][8] = {
  // family    // serialnumber             // crc
  {1,          184,   1, 79, 14, 0, 0,     216},
  {1,          190,  18, 79, 14, 0, 0,      88},
  {1,          123, 206, 78, 14, 0, 0,     220},
  {1,          203,  24, 79, 14, 0, 0,     150},
  {1,          160, 141, 79, 14, 0, 0,     200},
  {1,           76, 208, 78, 14, 0, 0,     106},
  {1,          158, 235, 78, 14, 0, 0,      52},
  {1,          168,  92, 79, 14, 0, 0,     250},
  {1,          185, 102, 79, 14, 0, 0,     175},
  {1,          225,  74, 79, 14, 0, 0,     249}  
};
short _tokenID = 0;

////////////////////////////////////////////////////////////////////////////////

void setup() {
  
  // Start HW Debugging
  pinMode(13, OUTPUT); 
  
  // Start Serial/ Debugging
  debug.enable(BAUD_RATE);      // Baudrate 9600
  _serialActive = true;
  
  if ( isI2CDeactivated() ) {

    debug.println("//// ----- Serial Mode ----- ////");
       
  } else {

    // Start I2C
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent); // register reading event handler
    Wire.onRequest(sendTokenID);  // register handler for request events -> send current token id
    
    debug.println("//// ----- I2C Mode ----- ////");
    debug.println("MASTER ID: ", MASTER_ADDRESS);
    debug.println("OWN ID: ", SLAVE_ADDRESS); 
    debug.println("//// -------------------- ////");

    _i2cActive = true;

    lastTimeStamp = millis();

  }
}

////////////////////////////////////////////////////////////////////////////////

void loop() {  

  // Check Serial Port
  if ( ! _i2cActive ) { // i2c deactivated
    checkSerial();   
  } 

  // ! The I2C Token Request Interrupt will abort a smooth movement
  // if targetPosition not reached, do movement
  //if (_hasNewTargetPosition) {
  //  doMovement(_targetPosition, _stepMode);
  //} 
  
  // check every 500 ms token changes
  if (millis() - lastTimeStamp >= 500) {
    discoverTokenID();
    lastTimeStamp = millis();    
  }  
}

////////////////////////////////////////////////////////////////////////////////

// callback for sending data
void sendTokenID(){  
  Wire.write((uint8_t *)&_tokenID, sizeof(_tokenID));  
}


// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int h) {
  
  debug.println("//// -------------------- ////");
  debug.println("Receiving I2C Message:");
  
  digitalWrite(13, HIGH);

  // Get message
  String msg = "";
  while(1 <= Wire.available()) {  
    char c =  Wire.read(); 
    msg += String( c );     
  }

  debug.print(msg);

  parseMessage(msg);

  digitalWrite(13, LOW);
}


void checkSerial() {

  if ( Serial.available() ) {
    
    debug.println("//// -------------------- ////");
    debug.println("Receiving Serial Message:");   

    digitalWrite(13, HIGH);

    // Get message
    String msg =  Serial.readString();
    debug.print(msg);  
    parseMessage(msg);

    digitalWrite(13, LOW);
  }
}

void parseMessage(String msg) {

  debug.println("Parsing Message...");
  debug.println(msg);

  // Example
  // INIT:top/[STEPS FOR POSITION 100]
  // INIT:bottom/[STEPS FOR POSITION 0]
  // INIT:99/XX-XX-XX-XX-XX-XX-XX-XX [HEX]
  // INIT:4/01-cb-18-4f-0e-00-00-96
  // INIT:calibrate
  // MOVE:[TARGET POSITION 0-100]/[SPEED 'full', 'half', 'quarter', 'eigthth']
  // MOVE:10/eigthth
  // MOVE:10/quarter
  // UP:[STEPS, default 400]
  // DOWN:[STEPS, default 400]
  // TEST:checkup
  
  
  
  // 1) Seperate command and value
  char valueSeperator = ':';
  
  String temp = msg;
  int maxIndex = temp.length()-1;  
  String cmd = "";
  String value = "";
  
  for (int i = 0; i < maxIndex; i++) {
    if( temp.charAt(i) == valueSeperator ) {
      cmd = temp.substring(0, i);
      value = temp.substring(i+1, maxIndex+1);
      
      debug.println("Command: ", cmd);      
      debug.println("Value: ", value);
    }
  }

  // 2) Execute commands
  if (cmd.indexOf("MOVE") > -1) {
    
    debug.println("Move:");

    // Parse MOVE parameter
    const int paramNum = 2;
    String movement[paramNum];    
    
    int lastEntry = -2;    
    String tempValue = value;
        
    // Parse Value
    for (int param = 0; param < paramNum+1; param++) {

        if (tempValue.indexOf("/") > -1 ) {          
          
          movement[param] = tempValue.substring(0, tempValue.indexOf("/") );
          debug.println(String( param + 1) + ": \t", movement[param] );  
                    
          tempValue = tempValue.substring(tempValue.indexOf("/")+1, tempValue.length());
          lastEntry = param;
          
        } else if (param == lastEntry + 1) { 
          movement[param] = tempValue;   
          debug.println(String( param + 1) + ": \t", movement[param] );             
        }  
    }

    unsigned int targetPosition = movement[0].toInt();
    String stepMode = movement[1];

    debug.println("Move to: ", targetPosition);

    _targetPosition = targetPosition;
    _stepMode = stepMode;
    _hasNewTargetPosition = true; // activate movement
    
    doMovement(targetPosition, stepMode);

    
  } else if (cmd.indexOf("UP") > -1) {
        
    ctl.setStepMode(HALF_STEP);
    
    if ( isValidNumber(value) ) {  
      ctl.moveUp(value.toInt());
    } else {
      debug.print("[Default Value: 400 Steps]");
      ctl.moveUp(400);
    }
    debug.println("Current Position: ", ctl.getPosition() );
    
  } else if (cmd.indexOf("DOWN") > -1) {
    
    ctl.setStepMode(HALF_STEP);
    
    if ( isValidNumber(value) ) {  
      ctl.moveDown(value.toInt());
    } else {
      debug.print("[Default Value: 400 Steps]");
      ctl.moveDown(400);
    }
    debug.println("Current Position: ", ctl.getPosition() );
    
  } else if (cmd.indexOf("TEST") > -1) {

    if (value.indexOf("checkup") > -1) {

      debug.println("Check Up Steps: ", ctl.getCheckUpSteps());
  
      long checkUp = ctl.getCheckUpSteps();
  
      ctl.calibrate();
  
      for (int i = 0; i < (checkUp/ctl.getMaxPosition()) / 2; i++) {
        ctl.moveTo(0, HALF_STEP);
        debug.println("Steps:", ctl.getSteps() );
        delay(10);
        ctl.moveTo(100, HALF_STEP);
        debug.println("Steps:", ctl.getSteps() );
        delay(10);
      }
    }
  
  } else if (cmd.indexOf("INIT") > -1) {
    
    debug.println("Initialization:");

    if (value.indexOf("calibrate") > -1) {
      debug.println("Calibrate");
      ctl.calibrate();
      debug.print("... OK");
    } else {

      // Parse INIT parameter
      const int paramNum = 2;
      String init[paramNum];    
      
      int lastEntry = -2;    
      String tempValue = value;
  
      // Parse Value
      for (int param = 0; param < paramNum+1; param++) {
  
          if (tempValue.indexOf("/") > -1 ) {          
            
            init[param] = tempValue.substring(0, tempValue.indexOf("/") );
            debug.println(String( param + 1) + ": \t", init[param] );  
                      
            tempValue = tempValue.substring(tempValue.indexOf("/")+1, tempValue.length());
            lastEntry = param;
            
          } else if (param == lastEntry + 1) { 
            init[param] = tempValue;   
            debug.println(String( param + 1) + ": \t", init[param] );             
          }  
      }
      
      // top/ bottom
      if ( init[0].indexOf("top") > -1 || init[0].indexOf("bottom") > -1 ) {
  
        debug.println("TOP/ BOTTOM");
        
        String side = init[0];
        unsigned long steps;
    
        if (side.indexOf("top") > -1) {
          if ( isValidNumber( init[1] ) ) {
            ctl.setMaxPosition(init[1].toInt());
          }        
        } else if (side.indexOf("bottom") > -1) {
          if ( isValidNumber( init[1] ) ) {
            int bottom = init[1].toInt();
            if (bottom < 0 ) bottom = 0;
            ctl.setMinPosition(bottom);
          }        
        } 
  
      } else  if ( isValidNumber(init[0]) ) {
        
        unsigned int id = init[0].toInt() - 1; // arrays ...
  
        if (id > _maxTokenIDs) {
          debug.println("ERROR: Token ID must be <="+_maxTokenIDs, "Your ID: "+id);
          return;
        }
        
        String serialNumber = init[1];
    
        debug.println("Token ID:", id);
        debug.print("Serial: ", serialNumber);
  
        const int paramNum = 7;
        String serial[paramNum];    
        
        int lastEntry = -2;    
        String tempValue = serialNumber;
    
        // Parse Value
        for (int param = 0; param < paramNum+1; param++) {
    
            if (tempValue.indexOf("-") > -1 ) {          
              
              serial[param] = tempValue.substring(0, tempValue.indexOf("-") );
              debug.println(String( param + 1) + ": \t", serial[param] );             
                        
              tempValue = tempValue.substring(tempValue.indexOf("-")+1, tempValue.length() ) ;
              lastEntry = param;
              
            } else if (param == lastEntry + 1) { 
              serial[param] = tempValue;               
              debug.println(String( param + 1) + ": \t", serial[param] );             
            }  
  
            debug.print(":\t", hexToDec(serial[param]) ); 
            tokenIDs[id][param] = hexToDec(serial[param]);
            debug.print(":\t", tokenIDs[id][param] ); 
        }
  
        debug.println("-----------------------------------------");
        for(int i = 0; i < _maxTokenIDs; i++) {
          debug.println("");
          for(int j = 0; j < 8; j++) {
            debug.print(tokenIDs[i][j] ); 
          }
        }
      }

    }
       
  } else if (cmd.indexOf("TOKEN") > -1) {
    discoverTokenID();
  }
}


/* -------------------------------------------------------------------------------------
 * Movement
 */

void doMovement(unsigned int targetPosition, String stepMode) {

  if (targetPosition != -1 && stepMode != "") { // default initialization
  
    ctl.enable();
  
    if (stepMode == "full") {    
      ctl.moveTo( targetPosition, FULL_STEP);    
    } else if (stepMode == "half") {
      ctl.moveTo( targetPosition, HALF_STEP);
    } else if (stepMode == "quarter") {
      ctl.moveTo( targetPosition, QUARTER_STEP);
    } else if (stepMode == "eigthth") {
      ctl.moveTo( targetPosition, EIGTHTH_STEP);
    }
  
    ctl.disable();

    _hasNewTargetPosition = false;
  }
}

unsigned int scalePosition( int targetPosition ) {

  switch(ctl.getStepMode()) {
    case FULL_STEP: return map(targetPosition, 0, 100, ctl.getMinPosition()/8, ctl.getMaxPosition()/8 );
    case HALF_STEP: return map(targetPosition, 0, 100, ctl.getMinPosition()/4, ctl.getMaxPosition()/4 );
    case QUARTER_STEP: return map(targetPosition, 0, 100, ctl.getMinPosition()/2, ctl.getMaxPosition()/2 );
    case EIGTHTH_STEP: return map(targetPosition, 0, 100, ctl.getMinPosition(), ctl.getMaxPosition() );

    default: return map(targetPosition, 0, 100, ctl.getMinPosition(), ctl.getMaxPosition() );
  }
  
  
}

unsigned int scalePosition( int targetPosition, unsigned int stepMode ) {

  switch(stepMode) {
    case FULL_STEP: return map(targetPosition, 0, 100, ctl.getMinPosition()/8, ctl.getMaxPosition()/8 );
    case HALF_STEP: return map(targetPosition, 0, 100, ctl.getMinPosition()/4, ctl.getMaxPosition()/4 );
    case QUARTER_STEP: return map(targetPosition, 0, 100, ctl.getMinPosition()/2, ctl.getMaxPosition()/2 );
    case EIGTHTH_STEP: return map(targetPosition, 0, 100, ctl.getMinPosition(), ctl.getMaxPosition() );

    default: return map(targetPosition, 0, 100, ctl.getMinPosition(), ctl.getMaxPosition() );
  }
  
  
}

/* -------------------------------------------------------------------------------------
 * 
 */

// Unidirectional bitwise communication between 
// the MotorController (D2 OUT) and the LEDController (D2 IN) 
// 1 = Serial Communication
// 0 = I2C Communication
boolean isI2CDeactivated() {
  
  pinMode(I2C_STATUS_PIN, INPUT);  
   
  int value;
  value = digitalRead(I2C_STATUS_PIN);  

  _i2cActive = (value == 0);  
  
  return (value == 1);
}


boolean isValidNumber(String str){
   for(byte i=0;i<str.length();i++)
   {
      if( ! isDigit(str.charAt(i)) ) {        
        return false;
      }
   }
   return true;
} 


void discoverTokenID() {
  
  byte i, sn;
  byte addr[8];
  int serialNumber = 0;
  
  while (ds.search(addr)) {      
            
      boolean checkSN = false;
            
      // check all serialnumnbers
      for (sn=0; sn < _maxTokenIDs; sn++) {
        
        // check each address byte
        for (i=0; i < 8; i++) {
          
          checkSN = false;
          
          if (tokenIDs[sn][i] == addr[i]) {
            checkSN = true;
          } 
  
          if (! checkSN) {
            break;
          }
        }

        if (checkSN) {
            serialNumber = sn + 1;
            break;
        }
      }

      if ( OneWire::crc8( addr, 7) != addr[7]) {
        return;
      } 
      
      if (checkSN) {
        
        if (_tokenID != serialNumber) {
          debug.println("NEW ID: ",serialNumber);
          _tokenID = serialNumber;
          debug.print(" -> Token ID set for I2C request");
        }
        
      } else {
        debug.println("UNKNOWN ID");
        return;
      }     
  }

  ds.reset_search();

  // First time without any token
  // Reset _tokenID
  // Log changes
  if (serialNumber == 0 && _tokenID != 0) {
    debug.println("NO TOKEN FOUND");
    _tokenID = 0;
    debug.print(" -> No Token ID for I2C request");
  }
  
  return;
}


// Convert color HEX #FF to RGB 255
unsigned int hexToDec(String hexString) {
  
  unsigned int decValue = 0;
  int nextInt;
  
  for (int i = 0; i < hexString.length(); i++) {
    
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    
    decValue = (decValue * 16) + nextInt;
  }
  
  return decValue;
}

