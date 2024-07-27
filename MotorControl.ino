// For 6V N20 micro motor
const int in1Pin = 2;
const int in2Pin = 3;

int enablePin = 3;
int buttonPin = 4;

const int motorSpeed = 100; // Max speed (0-255)
const unsigned long moveTime = 10000; // Time in milliseconds to move 5 cm (100 seconds)

bool platformRaised = false;
bool buttonPressed = false;



// For 12V DC motor
int E1 = 5;
int M1 = 6;

const int encoderAPin = 7;
const int encoderBPin = 8;

volatile long encoderValue = 0;
volatile bool lastAState = LOW;
volatile bool lastBState = LOW;



void setup()
{
  Serial.begin(9600);

  pinMode(M1, OUTPUT);
  pinMode(encoderAPin, INPUT);
  pinMode(encoderBPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderAPin), encoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBPin), encoderBISR, CHANGE);

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop()
{
  // int value;
  // for(value = 0 ; value <= 255; value+=5)
  // {
  //   digitalWrite(M1, HIGH);
  //   analogWrite(E1, value);   //PWM Speed Control
  //   delay(30);
  // }
  lastAState = digitalRead(encoderAPin);
  lastBState = digitalRead(encoderBPin);
  digitalWrite(M1, HIGH);
  analogWrite(E1, 100);   //PWM Speed Control

  // Display encoder value every second
  Serial.print("Encoder Value: ");
  Serial.println(encoderValue);

  // Check if button is pressed
  if (digitalRead(buttonPin) == LOW && !buttonPressed) {
    buttonPressed = true;
    if (!platformRaised) {
      raisePlatform();
      platformRaised = true;
    } else {
      lowerPlatform();
      platformRaised = false;
    }
  }

  // Reset button press state when button is released
  if (digitalRead(buttonPin) == HIGH && buttonPressed) {
    buttonPressed = false;
  }

  delay(10); // 1 second delay

}

void encoderAISR() {
  bool aState = digitalRead(encoderAPin);
  bool bState = digitalRead(encoderBPin);
  
  if (aState != lastAState) {
    if (aState == bState) {
      encoderValue++;
    } else {
      encoderValue--;
    }
  }
  
  lastAState = aState;
}

void encoderBISR() {
  bool aState = digitalRead(encoderAPin);
  bool bState = digitalRead(encoderBPin);
  
  if (bState != lastBState) {
    if (aState == bState) {
      encoderValue--;
    } else {
      encoderValue++;
    }
  }
  
  lastBState = bState;
}

void raisePlatform() {
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  analogWrite(enablePin, motorSpeed);
  delay(moveTime);
  stopMotor();
}

void lowerPlatform() {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  analogWrite(enablePin, motorSpeed);
  delay(moveTime);
  stopMotor();
}

void stopMotor() {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  analogWrite(enablePin, 0);
}
