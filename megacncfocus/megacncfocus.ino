#define STEPONTIMEuS 5
#define ENABLETIMEuS 500
#define MyDirX      5
#define MyStepX     2
#define MyDirY      6
#define MyStepY     3
#define MyDirZ      7
#define MyStepZ     4
#define MyEnable    8
#define encoder0PinS    14
#define encoder0PinA    18
#define encoder0PinB    19
#define ledXpin   A8
#define ledYpin   A9
#define ledZpin   A10

int btnpos = 0;
int actualposition = 0;
int targetposition = 0;
int changeposition = 0;

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
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(encoder0PinS, INPUT);  

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE);
  
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

}

void loop() {
  
  if (changeposition != actualposition) {
    targetposition = changeposition;
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

}

void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
          changeposition++;
    }
    else {
          changeposition--;
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
          changeposition++;
    }
    else {
          changeposition--;
    }
  }

}

void doEncoderB() {

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {
          changeposition++;
    }
    else {
          changeposition--;
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
          changeposition++;
    }
    else {
          changeposition--;
    }
  }

}


void dostep () {
      if ((btnpos==0) || (btnpos == 1)) digitalWrite(MyStepX, HIGH);
      if ((btnpos==0) || (btnpos == 2)) digitalWrite(MyStepY, HIGH);
      if ((btnpos==0) || (btnpos == 3)) digitalWrite(MyStepZ, HIGH);
      delayMicroseconds(STEPONTIMEuS);
      if ((btnpos==0) || (btnpos == 1)) digitalWrite(MyStepX, LOW);
      if ((btnpos==0) || (btnpos == 2)) digitalWrite(MyStepY, LOW);
      if ((btnpos==0) || (btnpos == 3)) digitalWrite(MyStepZ, LOW);
      delayMicroseconds(STEPONTIMEuS);   
  
}

