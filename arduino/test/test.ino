int rupsFwdR = 6;
int rupsFwdL = 2;
int rupsRevR = 7;
int rupsRevL = 3;
int rupsEnR = 24;
int rupsEnL = 25;

int ledPin = 13;

int timer = 0;
int ledSwitch = 0;

void setup(){
  pinMode(rupsFwdR, OUTPUT);   // sets the pin as output
  pinMode(rupsFwdL, OUTPUT);   // sets the pin as output
  pinMode(rupsRevR, OUTPUT);   // sets the pin as output
  pinMode(rupsRevL, OUTPUT);   // sets the pin as output
  pinMode(ledPin, OUTPUT);
  pinMode(rupsEnR, OUTPUT);
  pinMode(rupsEnL, OUTPUT);
  
  digitalWrite(rupsEnR, HIGH);
  digitalWrite(rupsEnL, HIGH);
}

void loop(){
  /*
  if (ledSwitch == 0){
    analogWrite(ledPin, 100);
    analogWrite(rupsPin1, 255);
    analogWrite(rupsPin2, 255);
    delay(1000);
    analogWrite(rupsPin1, 128);
    analogWrite(rupsPin2, 128);
    ledSwitch = 1;
  } else{
    analogWrite(ledPin, 0);
    ledSwitch = 0;
    analogWrite(rupsPin1, 0);
    analogWrite(rupsPin2, 0);
  }
  */
  analogWrite(rupsRevR, 0);
  analogWrite(rupsRevL, 0);
}
