#define STEPONTIMEuS 5
#define ENABLETIMEuS 500
#define DEBOUNCEmS 200
#define ENCODERINC 10
#define MyDirX      5
#define MyStepX     2
#define MyDirY      6
#define MyStepY     3
#define MyDirZ      7
#define MyStepZ     4
#define MyEnable    8
#define encoderPinS    14
#define encoderPinA    18
#define encoderPinB    19
#define ledXpin   A8
#define ledYpin   A9
#define ledZpin   A10

int btnpos = 0;
volatile int actualposition = 0;
volatile int targetposition = 0;
volatile int encoderposition = 0;
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
  pinMode(MyEnable, OUTPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderPinS, INPUT);  

  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);
  
  digitalWrite(MyDirX, LOW);
  digitalWrite(MyStepX, LOW);
  digitalWrite(MyDirY, LOW);
  digitalWrite(MyStepY, LOW);
  digitalWrite(MyDirZ, LOW);
  digitalWrite(MyStepZ, LOW);
  digitalWrite(MyEnable, HIGH);

  digitalWrite(ledXpin, HIGH);
  digitalWrite(ledYpin, HIGH);
  digitalWrite(ledZpin, HIGH);

  Serial.begin (9600);


}

void loop() {
  if (encoderposition != actualposition) {
    Serial.println (encoderposition, DEC);  
    targetposition = encoderposition;
    dosteps();
  }    

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
  
  
}

void doEncoderA() {
  delay(DEBOUNCEmS);
  if ( digitalRead(encoderPinA) != A_set ) {
    A_set = !A_set;
    if ( A_set && !B_set ) encoderposition += ENCODERINC;
  }
}

void doEncoderB() {
  delay(DEBOUNCEmS);
  if ( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    if ( B_set && !A_set ) encoderposition -= ENCODERINC;
  }
}

void dosteps() {
    if (targetposition > actualposition) {
          digitalWrite(MyDirX, HIGH);
          digitalWrite(MyDirY, HIGH);
          digitalWrite(MyDirZ, HIGH);
          digitalWrite(MyEnable, LOW);  
          delayMicroseconds(ENABLETIMEuS);          
          for (;actualposition<targetposition;actualposition++) dostep();
          digitalWrite(MyEnable, HIGH);            
    }
    else {
          digitalWrite(MyDirX, LOW);
          digitalWrite(MyDirY, LOW);
          digitalWrite(MyDirZ, LOW);
          digitalWrite(MyEnable, LOW);  
          delayMicroseconds(ENABLETIMEuS);            
          for (;actualposition>targetposition;actualposition--) dostep(); 
          digitalWrite(MyEnable, HIGH);   
    } 
}

void dostep() {
      if ((btnpos==0) || (btnpos == 1)) digitalWrite(MyStepX, HIGH);
      if ((btnpos==0) || (btnpos == 2)) digitalWrite(MyStepY, HIGH);
      if ((btnpos==0) || (btnpos == 3)) digitalWrite(MyStepZ, HIGH);
      delayMicroseconds(STEPONTIMEuS);
      if ((btnpos==0) || (btnpos == 1)) digitalWrite(MyStepX, LOW);
      if ((btnpos==0) || (btnpos == 2)) digitalWrite(MyStepY, LOW);
      if ((btnpos==0) || (btnpos == 3)) digitalWrite(MyStepZ, LOW);
      delayMicroseconds(STEPONTIMEuS*200);   
  
}


