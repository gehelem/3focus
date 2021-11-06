#define STEPONTIMEuS 5
#define ENABLETIMEuS 200
#define DEBOUNCEmS 250
#define ENCODERINC  10

#define BORIS

#ifdef BORIS
#define MyDirX      55
#define MyStepX     54
#define MyDirY      61
#define MyStepY     60
#define MyDirZ      48
#define MyStepZ     46
#define MyEnableX   38
#define MyEnableY   56
#define MyEnableZ   62
#define encoderPinS   3
#define encoderPinA    20
#define encoderPinB    21
#define ledXpin   22 //A8
#define ledYpin   24 //A9
#define ledZpin   26 //A10
#endif

#ifndef BORIS
#define MyDirX      5
#define MyStepX     2
#define MyDirY      6
#define MyStepY     3
#define MyDirZ      7
#define MyStepZ     4
#define MyEnable    8
#define encoderPinS 14
#define encoderPinA 18
#define encoderPinB 19
#define ledXpin   A8
#define ledYpin   A9
#define ledZpin   A10
#endif
 
#define MAXCOMMAND 8
char cmd[MAXCOMMAND];         // these are for handling and processing serial commands
char param[MAXCOMMAND];
char line[MAXCOMMAND];
int eoc = 0;    // end of command
int idx = 0;    // index into command string
 
int btnpos = 0;
long pos;
boolean isMoving = false;
volatile boolean forward = true;
volatile boolean enabled = false;
volatile unsigned long enabledcount = 65535*65535*65535;
volatile int actualposition = 16000;
volatile int targetposition = 16000;
//volatile int encoderposition = 16000;
volatile int serialposition = 16000;
 
boolean A_set = false;
boolean B_set = false;
 
 
void setup() {
 
  pinMode(MyDirX, OUTPUT);
  pinMode(MyStepX, OUTPUT);
  pinMode(MyDirY, OUTPUT);
  pinMode(MyStepY, OUTPUT);
  pinMode(MyDirZ, OUTPUT);
  pinMode(MyStepZ, OUTPUT);
  pinMode(ledXpin, OUTPUT);                                 
  pinMode(ledYpin, OUTPUT);
  pinMode(ledZpin, OUTPUT); 
#ifdef BORIS
  pinMode(MyEnableX, OUTPUT);
  pinMode(MyEnableY, OUTPUT);
  pinMode(MyEnableZ, OUTPUT);
#endif  
#ifndef BORIS
  pinMode(MyEnable, OUTPUT);
#endif  
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderPinS, INPUT_PULLUP); 
 
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);
 
  digitalWrite(MyDirX, LOW);
  digitalWrite(MyStepX, LOW);
  digitalWrite(MyDirY, LOW);
  digitalWrite(MyStepY, LOW);
  digitalWrite(MyDirZ, LOW);
  digitalWrite(MyStepZ, LOW);
#ifndef BORIS    
  digitalWrite(MyEnable, HIGH);
#endif
#ifdef BORIS        
  digitalWrite(MyEnableX, HIGH);
  digitalWrite(MyEnableY, HIGH);
  digitalWrite(MyEnableZ, HIGH);
#endif
 
  digitalWrite(ledXpin, HIGH);
  digitalWrite(ledYpin, HIGH);
  digitalWrite(ledZpin, HIGH);
 
  Serial.begin (9600);

  
 
}
 
void loop() {
  if (enabled) enabledcount--;
  if (enabledcount <100) {
#ifndef BORIS    
        digitalWrite(MyEnable, HIGH);
#endif
#ifdef BORIS        
        digitalWrite(MyEnableX, HIGH);
        digitalWrite(MyEnableY, HIGH);
        digitalWrite(MyEnableZ, HIGH);
#endif
        enabled=false;
        enabledcount=65535*65535*65535;
  }

  
  if (serialposition != actualposition) {
    dosteps();
    isMoving = true;
  } else isMoving = false; 
 
  if (digitalRead(encoderPinS) == LOW) {
    btnpos++;
    if (btnpos > 3) btnpos = 0;
    digitalWrite(ledXpin, LOW);
    digitalWrite(ledYpin, LOW);
    digitalWrite(ledZpin, LOW);
    if ((btnpos==0) || (btnpos == 1)) digitalWrite(ledXpin, HIGH);
    if ((btnpos==0) || (btnpos == 2)) digitalWrite(ledYpin, HIGH);
    if ((btnpos==0) || (btnpos == 3)) digitalWrite(ledZpin, HIGH);
    delay(DEBOUNCEmS);    
  }
 
  // process the command string when a hash arrives:
  if (eoc) {
    processCommand(line);
    memset(line, 0, MAXCOMMAND);
    eoc = false;
  }
 
}
 
void doEncoderA() {
//  delay(DEBOUNCEmS);
  if ( digitalRead(encoderPinA) != A_set ) {
      A_set = !A_set;
      //if ( A_set && !B_set ) encoderposition += ENCODERINC;
      if (!enabled) {
#ifndef BORIS    
          digitalWrite(MyEnable, LOW);
#endif
#ifdef BORIS        
          digitalWrite(MyEnableX, LOW);
          digitalWrite(MyEnableY, LOW);
          digitalWrite(MyEnableZ, LOW);
#endif        
          delayMicroseconds(ENABLETIMEuS);         
          enabled=true;
      }
      if (forward) {
          digitalWrite(MyDirX, LOW);
          digitalWrite(MyDirY, LOW);
          digitalWrite(MyDirZ, LOW);
          forward=false;
      }
      if ((btnpos==0) && A_set && !B_set ) { 
          digitalWrite(MyStepX, HIGH);
          digitalWrite(MyStepY, HIGH);
          digitalWrite(MyStepZ, HIGH);
          delayMicroseconds(STEPONTIMEuS);          
          digitalWrite(MyStepX, LOW);
          digitalWrite(MyStepY, LOW);
          digitalWrite(MyStepZ, LOW);
          delayMicroseconds(STEPONTIMEuS);
      }
      if ((btnpos==1) && A_set && !B_set ) { 
          digitalWrite(MyStepX, HIGH);
          delayMicroseconds(STEPONTIMEuS);          
          digitalWrite(MyStepX, LOW);
          delayMicroseconds(STEPONTIMEuS);
      }
      if ((btnpos==2) && A_set && !B_set ) { 
          digitalWrite(MyStepY, HIGH);
          delayMicroseconds(STEPONTIMEuS);          
          digitalWrite(MyStepY, LOW);
          delayMicroseconds(STEPONTIMEuS);
      }
      if ((btnpos==3) && A_set && !B_set ) { 
          digitalWrite(MyStepZ, HIGH);
          delayMicroseconds(STEPONTIMEuS);          
          digitalWrite(MyStepZ, LOW);
          delayMicroseconds(STEPONTIMEuS);
      }
  }
}
 
void doEncoderB() {
//  delay(DEBOUNCEmS);
  if ( digitalRead(encoderPinB) != B_set ) {
      B_set = !B_set;
      if (!enabled) {
#ifndef BORIS    
          digitalWrite(MyEnable, LOW);
#endif
#ifdef BORIS        
          digitalWrite(MyEnableX, LOW);
          digitalWrite(MyEnableY, LOW);
          digitalWrite(MyEnableZ, LOW);
#endif        
          delayMicroseconds(ENABLETIMEuS);         
          enabled=true;
      }
      if (!forward) {
          digitalWrite(MyDirX, HIGH);
          digitalWrite(MyDirY, HIGH);
          digitalWrite(MyDirZ, HIGH);
          forward=true;
      }
      //if ( B_set && !A_set ) encoderposition -= ENCODERINC;
      if ((btnpos==0) && B_set && !A_set ) { 
          digitalWrite(MyStepX, HIGH);
          digitalWrite(MyStepY, HIGH);
          digitalWrite(MyStepZ, HIGH);
          delayMicroseconds(STEPONTIMEuS);          
          digitalWrite(MyStepX, LOW);
          digitalWrite(MyStepY, LOW);
          digitalWrite(MyStepZ, LOW);
          delayMicroseconds(STEPONTIMEuS);
      }
      if ((btnpos==1) && A_set && !B_set ) { 
          digitalWrite(MyStepX, HIGH);
          delayMicroseconds(STEPONTIMEuS);          
          digitalWrite(MyStepX, LOW);
          delayMicroseconds(STEPONTIMEuS);
      }
      if ((btnpos==2) && A_set && !B_set ) { 
          digitalWrite(MyStepY, HIGH);
          delayMicroseconds(STEPONTIMEuS);          
          digitalWrite(MyStepY, LOW);
          delayMicroseconds(STEPONTIMEuS);
      }
      if ((btnpos==3) && A_set && !B_set ) { 
          digitalWrite(MyStepZ, HIGH);
          delayMicroseconds(STEPONTIMEuS);          
          digitalWrite(MyStepZ, LOW);
          delayMicroseconds(STEPONTIMEuS);
      }      
  }
}
 
void dosteps() {
    if (!enabled) {
#ifndef BORIS    
      digitalWrite(MyEnable, LOW);
#endif
#ifdef BORIS        
      digitalWrite(MyEnableX, LOW);
      digitalWrite(MyEnableY, LOW);
      digitalWrite(MyEnableZ, LOW);
#endif 
      delayMicroseconds(ENABLETIMEuS);         
      enabled=true;
    }  
    if (targetposition > actualposition) {
          digitalWrite(MyDirX, HIGH);
          digitalWrite(MyDirY, HIGH);
          digitalWrite(MyDirZ, HIGH);
          for (;actualposition<targetposition;actualposition++) dostep();
    }
    else {
          digitalWrite(MyDirX, LOW);
          digitalWrite(MyDirY, LOW);
          digitalWrite(MyDirZ, LOW);
          for (;actualposition>targetposition;actualposition--) dostep();
    }
}
 
void dostep() {
/*      if ((btnpos==0) || (btnpos == 1)) digitalWrite(MyStepX, HIGH);
      if ((btnpos==0) || (btnpos == 2)) digitalWrite(MyStepY, HIGH);
      if ((btnpos==0) || (btnpos == 3)) digitalWrite(MyStepZ, HIGH);
      delayMicroseconds(STEPONTIMEuS);
      if ((btnpos==0) || (btnpos == 1)) digitalWrite(MyStepX, LOW);
      if ((btnpos==0) || (btnpos == 2)) digitalWrite(MyStepY, LOW);
      if ((btnpos==0) || (btnpos == 3)) digitalWrite(MyStepZ, LOW);
      delayMicroseconds(STEPONTIMEuS*200);  */
      digitalWrite(MyStepX, HIGH);
      digitalWrite(MyStepY, HIGH);
      digitalWrite(MyStepZ, HIGH);
      delayMicroseconds(STEPONTIMEuS);
      digitalWrite(MyStepX, LOW);
      digitalWrite(MyStepY, LOW);
      digitalWrite(MyStepZ, LOW);
      delayMicroseconds(STEPONTIMEuS*200);  

      
  
}
 
void processCommand(String command) {
  memset(cmd, 0, MAXCOMMAND);
  memset(param, 0, MAXCOMMAND);
  int len = strlen(line);
  if (len >= 2) {
    strncpy(cmd, line, 2);
  }
  if (len > 2) {
    strncpy(param, line + 2, len - 2);
  }
 
  memset(line, 0, MAXCOMMAND);
  eoc = 0;
  idx = 0;
 
  // --------------------------------------------------------------------------------
  // get the current focuser position
  if (!strcasecmp(cmd, "GP")) {
    char tempString[6];
    sprintf(tempString, "%04X", actualposition);
    Serial.print(tempString);
    Serial.print("#");
  }
  // --------------------------------------------------------------------------------
  // whether half-step is enabled or not, always return "00"
  else if (!strcasecmp(cmd, "GH")) { Serial.print("00#"); }
  // --------------------------------------------------------------------------------
  // version
  else if (!strcasecmp(cmd, "GV")) { Serial.print("01#"); }    
  // --------------------------------------------------------------------------------
  // get the current temperature - moonlite compatible
  else if (!strcasecmp(cmd, "GT")) {
    char tempString[6];
    sprintf(tempString, "%04X", 20);
    Serial.print(tempString);
    Serial.print("#");
  } 
  // --------------------------------------------------------------------------------
  // get The current RED Led Backlight value, - moonlite compatible
  else if (!strcasecmp(cmd, "GB")) {
    char tempString[6];
    sprintf(tempString, "%04X", 111);
    Serial.print(tempString);
    Serial.print("#");
  } 
  
  // --------------------------------------------------------------------------------
  // get the current motor step delay, only values of 02, 04, 08, 10, 20
  // not used so just return 02
  else if (!strcasecmp(cmd, "GD")) {
    char tempString[6];
    sprintf(tempString, "%02X", 2);
    Serial.print(tempString);
    Serial.print("#");
  }
  
  // --------------------------------------------------------------------------------
  // motor is moving - 1 if moving, 0 otherwise
  else if (!strcasecmp(cmd, "GI")) {
    if (isMoving ) {
      Serial.print("01#");
    }
    else {
      Serial.print("00#");
    }
  }
  // initiate a move to the target position
  else if (!strcasecmp(cmd, "FG")) {
    targetposition = serialposition;
  }
  // stop a move - HALT
  else if (!strcasecmp(cmd, "FQ")) {
    //encoderposition = actualposition;
    serialposition = actualposition;
    targetposition = actualposition;          
  }
  // temperature calibration offset POXX in 0.5 degree increments (hex)
  //else if (!strcasecmp(cmd, "PO")) {}
  // set temperature co-efficient XX
  //else if (!strcasecmp(cmd, "SC")) {
 
  // set current position to received position - no move SPXXXX
  // in INDI driver, only used to set to 0 SP0000 in reset()
  else if (!strcasecmp(cmd, "SP")) {
    pos = hexstr2long(param);
    if ( pos > 65535 ) pos = 65535;
    if ( pos < 0 ) pos = 0;
    //encoderposition = pos;
    actualposition = pos;
    serialposition = pos;
  }
  // set new target position SNXXXX - this is a move command
  // but must be followed by a FG command to start the move
  else if (!strcasecmp(cmd, "SN")) {
    pos = hexstr2long(param);
    if ( pos > 65535 ) pos = 65535;
    if ( pos < 0 ) pos = 0;
    serialposition = pos;
  }
  else if (!strcasecmp(cmd, "SD")) {
    pos = hexstr2long(param);
    //stepdelay = (int) pos;
  }
 
  
}
 

// SerialEvent occurs whenever new data comes in the serial RX.
void serialEvent() {
  // : starts the command, # ends the command, do not store these in the command buffer
  // read the command until the terminating # character
  while (Serial.available() && !eoc) {
    char inChar = Serial.read();
    if (inChar != '#' && inChar != ':') {
      line[idx++] = inChar;
      if (idx >= MAXCOMMAND) {
        idx = MAXCOMMAND - 1;
      }
    }
    else {
      if (inChar == '#') {
        eoc = 1;
      }
    }
  }
}


long hexstr2long(char *line) {
  long ret = 0;

  ret = strtol(line, NULL, 16);
  return (ret);
}
