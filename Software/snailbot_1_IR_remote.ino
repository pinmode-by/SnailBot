// SnailBot
// active obstacle robot
// arduino nano + DRV8833(black) + 6*TCRT5000
// Control with IRremote
#include <IRremote.hpp>

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
const byte IR_PIN = 1;  // IR remote pin (TSOP4038)

const unsigned long IR_START = 0xF9067984;      // HOME
const unsigned long IR_STOP = 0xED127984;       // POWER
const unsigned long IR_CALIBR = 0xF00F7984;     // SEARCH
const unsigned long IR_READ_SENS = 0xF50A7984;  // PLAY
const unsigned long IR_TEST = 0xF7087984;       // ENTER
const unsigned long IR_INFO = 0xE51A7984;       // OPTION

enum Command {
  cmdLFR,
  cmdStop,
  cmdCalibr,
  cmdReadSensBT,
  cmdReadSensIR,
  cmdTest,
  cmdInfoBT,
  cmdInfoIR,
  cmdNo,
  cmdReadSensSerial
};

int state = 0;  //state of robot, 0-stop, 1-forward, -1 - back
int speed = 100;
const int MINSPEED = 80;
const int MAXSPEED = 200;
const int NUMSENS = 6;
int sensorPins[NUMSENS] = { A2, A1, A0, A5, A4, A3 };
int sensor[NUMSENS];  //sensor values
int maxSens[NUMSENS];
int minSens[NUMSENS];

int th[NUMSENS] = { 100, 100, 100, 100, 100, 100 };  //threshold for each sensor

void beep(int freq, int ms) {
  tone(BUZZ, freq);
  delay(ms);
  noTone(BUZZ);
}

int getSpeed() {  // return base robot speed
  int a = analogRead(POT);
  int v = map(a, 0, 1023, MINSPEED, MAXSPEED);
  return v;
}

void readSensors() {
  for (int i = 0; i < NUMSENS; i++) {
    sensor[i] = analogRead(sensorPins[i]);  //+ analogRead(sensorPins[i])) / 2;
  }
}
bool isCmdIR(byte cmd) {
  bool result = false;

  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.decodedRawData == cmd) {
      result = true;
    }
    IrReceiver.resume();
  }
  return result;
}

int getCmdIR() {
  int retCmd = cmdNo;
  if (IrReceiver.decode()) {
    switch (IrReceiver.decodedIRData.decodedRawData) {
      case IR_START:
        retCmd = cmdLFR;
        break;
      case IR_STOP:
        retCmd = cmdStop;
        break;
      case IR_TEST:
        retCmd = cmdTest;
        break;
      case IR_CALIBR:
        retCmd = cmdCalibr;
        break;
      case IR_READ_SENS:
        retCmd = cmdReadSensIR;
        break;
      case IR_INFO:
        retCmd = cmdInfoIR;
        break;
    }
    IrReceiver.resume();
  }
  return retCmd;
}  //    IrReceiver.p

byte digitLine() {
  byte sum = 0B00000000;
  byte w = 0B00000001 << (NUMSENS - 1);
  for (int j = 0; j < NUMSENS; j++) {
    if (sensor[j] > th[j]) sum = sum + w;
    w = w / 2;
  }
  return sum;
}

void calibration(int cycles) {
  for (int j = 0; j < NUMSENS; j++) {
    minSens[j] = 1023;
    maxSens[j] = 0;
  }
  drive(80, -80);
  for (int i = 0; i < cycles; i++) {
    readSensors();
    for (int j = 0; j < NUMSENS; j++) {
      if (sensor[j] > maxSens[j]) maxSens[j] = sensor[j];
      if (sensor[j] < minSens[j]) minSens[j] = sensor[j];
    }
  }
  drive(0, 0);
  for (int j = 0; j < NUMSENS; j++) {
    th[j] = (minSens[j] + maxSens[j]) / 2;
    Serial.print(th[j]);
    Serial.print("  ");
  }
  Serial.println();
}


void printSensors() {
  for (int i = 0; i < NUMSENS; i++) {
    Serial.print(sensor[i]);
    Serial.print("  ");
  }
  Serial.print(" => ");
  Serial.println(digitLine(), BIN);
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
  IrReceiver.begin(IR_PIN);

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
  beep(440, 200);
  beep(880, 300);
  beep(1760, 400);

  delay(100);
  digitalWrite(FLED, LOW);
  digitalWrite(BLED, LOW);

  long t = millis();  // пропуск калибровки если кнопка не нажата втечении 3 сек
  while (millis() - t < 3000) {
    if (digitalRead(BUTTON) == 0) {
      calibration(10000);
    }
  }
  Serial.print("setup over");
}

bool pressButton() {
  return !digitalRead(BUTTON);
}

void LFR() {
  digitalWrite(FLED, HIGH);
  digitalWrite(BLED, LOW);
  readSensors();
  //speed = getSpeed();
  byte DL = digitLine() >> 3;

  if (DL == 0B010 || DL == 0B111) {
    drive(speed, speed);
  } else if (DL == 0B100) {
    drive(speed * 1.4, speed * 0.6);
  } else if (DL == 0B110) {
    drive(speed * 1.2, speed * 0.8);
  } else if (DL == 0B001) {
    drive(speed * 0.6, speed * 1.4);
  } else if (DL == 0B011) {
    drive(speed * 0.8, speed * 1.2);
  } else if (DL == 0B000 || DL == 0B101) {
    drive(0, 0);
    state = -1;
    beep(2000, 200);
  }
}
else if (state == -1) {
  digitalWrite(FLED, LOW);
  digitalWrite(BLED, HIGH);
  readSensors();
  byte DL = digitLine() & 0B00000111;
  if (DL == 0B010 || DL == 0B111) {
    drive(-speed, -speed);
  } else if (DL == 0B100) {
    drive(-speed * 0.6, -speed * 1.4);
  } else if (DL == 0B110) {
    drive(-speed * 0.8, -speed * 1.2);
  } else if (DL == 0B001) {
    drive(-speed * 1.4, -speed * 0.6);
  } else if (DL == 0B011) {
    drive(-speed * 1.2, -speed * 0.8);
  } else if (DL == 0B000 || DL == 0B101) {
    drive(0, 0);
    state = 1;
    beep(4000, 200);
  }
}
}
void loop() {
  byte cmd = cmdNo;
  //cmd = getCmdIR();
  if (pressButton()) {
    cmd = cmdLFR;
  }
  if (cmd == cmdNo) {
    cmd = getCmdBT();
  }

  switch (cmd) {
    case cmdLFR:
      digitalWrite(LED, HIGH);
      delay(START_DELAY);
      LFR();
      digitalWrite(LED, LOW);
      break;
    case cmdCalibr:
      calibration();
      break;
    case cmdReadSensBT:
      printSensorsBT();
      break;
    case cmdReadSensIR:
      printSensorsSerial();
      break;
    case cmdTest:
      digitalWrite(LED, HIGH);
      //test_drive();
      digitalWrite(LED, LOW);
      break;
    case cmdInfoBT:
      printInfoBT();
      break;
    case cmdInfoIR:
      printInfoSerial();
      break;

      //...
  }  // switch
  if (state == 0) {
    while (digitalRead(BUTTON)) {
      speed = getSpeed();
      //Serial.print("Speed= ");
      //Serial.println(speed);
      readSensors();
      printSensors();
      delay(200);
    }
    state = 1;

  } else if (state == 1) {
  }
