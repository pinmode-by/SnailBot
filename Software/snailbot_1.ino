// SnailBot
// active obstacle robot
// arduino nano + DRV8833(black) + 6*TCRT5000

const int BUTTON = 7;   //start/stop buton
const int FLED = 4;     // front LED pin
const int BLED = 2;     // back LED pin
const int BUZZ = 8;     //active buzzer
const int POT = A6;     //potentiometer pin
const int VMETER = A7;  //voltage meter pin
const int ML1 = 10;     //left motor pin1
const int ML2 = 6;      //left motor pin2
const int MR1 = 9;      //right motor pin1
const int MR2 = 5;      //right motor pin2

int state = 0;  //state of robot, 0-stop, 1-forward, -1 - back
int speed = 100;
const int MINSPEED = 80;
const int MAXSPEED = 200;
const int NUMSENS = 6;
int sensorPins[NUMSENS] = { A5, A4, A3, A2, A1, A0 };
int sensor[NUMSENS];    //sensor values
int th[NUMSENS]= { 200, 200, 200, 200, 200, 200 };        //threshold for each sensor

int getSpeed() {  // return base robot speed
  int a = analogRead(POT);
  int v = map(a, 0, 1023, MINSPEED, MAXSPEED);
  return v;
}

void readSensors() {
  for (int i = 0; i < NUMSENS; i++) {
    sensor[i] = (analogRead(sensorPins[i])+analogRead(sensorPins[i]))/2;
  }
}

void calibration(int cycles){
  drive(50,-50);
  for(int i=0; i<cycles; i++){
    readSensors();
  }
  drive(0,0);
}


void printSensors() {
  for (int i = 0; i < NUMSENS; i++) {
    Serial.print(sensor[i]);
    Serial.print("  ");
  }
  Serial.println();
}

void drive(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  if (left > 0) {
    digitalWrite(ML1, 1);
    analogWrite(ML2, 255 - left);
  } else {
    digitalWrite(ML2, 1);
    analogWrite(ML1, 255 + left);
  }

  if (right > 0) {
    digitalWrite(MR1, 1);
    analogWrite(MR2, 255 - right);
  } else {
    digitalWrite(MR2, 1);
    analogWrite(MR1, 255 + right);
  }
}


void setup() {
  Serial.begin(9600);
  pinMode(FLED, OUTPUT);
  pinMode(BLED, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  digitalWrite(FLED, HIGH);
  digitalWrite(BLED, HIGH);
  tone(BUZZ, 440);
  delay(500);
  noTone(BUZZ);
  delay(1500);
  digitalWrite(FLED, LOW);
  digitalWrite(BLED, LOW);
}

void loop() {
  if (state == 0) {
    while (digitalRead(BUTTON)) {
      speed = getSpeed();
      Serial.print("Speed= ");
      Serial.println(speed);
      readSensors();
      printSensors();
      delay(500);
    }
    state = 1;

  } else if (state == 1) {
    digitalWrite(FLED, HIGH);
    digitalWrite(BLED, LOW);
    drive(100, 100);

  } else if (state == -1) {
    digitalWrite(FLED, LOW);
    digitalWrite(BLED, HIGH);
    drive(-100, -100);
  }
}
