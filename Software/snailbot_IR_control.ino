// SnailBot
// active obstacle robot
// arduino nano + DRV8833(black) + 6*TCRT5000
// IR remote control
// Auto voltage level correction

#include <Arduino.h>
#include <IRremote.hpp>
#include <EEPROM.h>

// pinout:
const byte BUTTON = 7;   //start/stop buton
const byte FLED = 4;     // front LED pin
const byte BLED = 2;     // back LED pin
const byte BUZZ = 8;     //active buzzer
const byte POT = A6;     //potentiometer pin
const byte VMETER = A7;  //voltage meter pin, 8.0V =500, kV=0.0160
const byte ML1 = 10;     //left motor pin1
const byte ML2 = 6;      //left motor pin2
const byte MR1 = 9;      //right motor pin1
const byte MR2 = 5;      //right motor pin2
const byte IR_PIN = 12;  //IR remote TSOP pin

// IR remote codes:
const unsigned long IR_START = 0xBC43FF00;       // PLAY
const unsigned long IR_STOP = 0xBA45FF00;        // CH-
const unsigned long IR_CALIBR = 0xF609FF00;      // EQ
const unsigned long IR_SPEED_UP = 0xEA15FF00;    // +
const unsigned long IR_SPEED_DOWN = 0xF807FF00;  // -

enum Command {
  cmdLFR,
  cmdStop,
  cmdCalibr,
  cmdPlus,
  cmdMinus,
  cmdNo
};

String COM_STR[] = { "RUN", "STOP", "CALIBR", "PLUS", "MINUS", "" };

int state = 0;       //state of robot, 0-stop, 1-forward, -1 - back
int speed = 80;      //default robot speed
const int V8 = 500;  // 8V power source
const int MINSPEED = 40;
const int MAXSPEED = 160;
const int NUMSENS = 6;
const int START_DELAY = 300;
const int SPEED_ADR = 190;
const int CALIBR_ADR = 200;

int sensorPins[NUMSENS] = { A2, A1, A0, A5, A4, A3 };
int sensor[NUMSENS];  //sensor values
int maxSens[NUMSENS];
int minSens[NUMSENS];

int th[NUMSENS] = { 300, 300, 300, 300, 300, 300 };  //threshold for each sensor

// function
void readSensors();
byte digitLine();
void calibration(int cycles);
void readCalibration();
void printSensors();
void drive(int left, int right);
void LFR();
void loadEEPROM();
void saveEEPROM();
void plusSpeed();
void minusSpeed();
void printCommand(int com);


void printCommand(int com) {
  Serial.println(COM_STR[com]);
}

void plusSpeed() {
  speed = speed + 5;
  if (speed > MAXSPEED) speed = MAXSPEED;
  saveEEPROM();
  delay(200);
}

void minusSpeed() {
  speed = speed - 5;
  if (speed < MINSPEED) speed = MINSPEED;
  saveEEPROM();
  delay(200);
}

void saveEEPROM() {
  EEPROM.put(SPEED_ADR, speed);
}

void loadEEPROM() {
  int s;
  EEPROM.get(SPEED_ADR, s);
  if ((s < 0)) {
    speed = 80;
  } else speed = s;
}

void readSensors() {
  for (int i = 0; i < NUMSENS; i++) {
    sensor[i] = analogRead(sensorPins[i]);  //+ analogRead(sensorPins[i])) / 2;
  }
}

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
    digitalWrite(FLED, (i / 500) % 2);
    digitalWrite(BLED, !((i / 500) % 2));
    for (int j = 0; j < NUMSENS; j++) {
      if (sensor[j] > maxSens[j]) maxSens[j] = sensor[j];
      if (sensor[j] < minSens[j]) minSens[j] = sensor[j];
    }
  }
  drive(0, 0);
  int adr = CALIBR_ADR;
  for (int j = 0; j < NUMSENS; j++) {
    th[j] = (minSens[j] + maxSens[j]) / 2;
    EEPROM.put(adr, th[j]);
    adr = adr + sizeof(th[j]);
    Serial.print(th[j]);
    Serial.print("  ");
  }
  Serial.println();
  digitalWrite(FLED, LOW);
  digitalWrite(BLED, LOW);
}

void readCalibration() {
  int adr = CALIBR_ADR;
  int val;
  for (int j = 0; j < NUMSENS; j++) {
    EEPROM.get(adr, val);
    if (isnan(val) || (val > 1000) || (val < 0)) {
      th[j] = 300;
    } else {
      th[j] = val;
    }
    adr = adr + sizeof(th[j]);
  }
}

void printSensors() {
  for (int i = 0; i < NUMSENS; i++) {
    Serial.print(sensor[i]);
    Serial.print("  ");
  }
  Serial.print(" => ");
  Serial.print(digitLine(), BIN);
  Serial.print("  V= ");
  Serial.println(analogRead(VMETER));
}

void drive(int left, int right) {
  int v = analogRead(VMETER);    //voltage
  left = (long)left * V8 / v;    //voltage correction of left motor
  right = (long)right * V8 / v;  //voltage correction of right  motor
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  if (left > 0) {
    analogWrite(ML1, left);
    digitalWrite(ML2, 0);
  } else if (left < 0) {
    analogWrite(ML2, -left);
    digitalWrite(ML1, 0);
  } else {
    digitalWrite(ML1, 1);
    digitalWrite(ML2, 1);
  }

  if (right > 0) {
    analogWrite(MR1, right);
    digitalWrite(MR2, 0);
  } else if (right < 0) {
    analogWrite(MR2, -right);
    digitalWrite(MR1, 0);
  } else {
    digitalWrite(MR1, 1);
    digitalWrite(MR2, 1);
  }
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

      case IR_CALIBR:
        retCmd = cmdCalibr;
        break;
      case IR_SPEED_UP:
        retCmd = cmdPlus;
        break;
      case IR_SPEED_DOWN:
        retCmd = cmdMinus;
        break;
    }
    IrReceiver.resume();
  }
  return retCmd;
}

void LFR() {
  byte cmd;
  while (1) {
    cmd = getCmdIR();
    if (cmd != cmdNo) {
      printCommand(cmd);
    }
    switch (cmd) {
      case cmdLFR:
        digitalWrite(FLED, HIGH);
        delay(START_DELAY);
        state = 1;
        LFR();
        break;
      case cmdStop:
        drive(0, 0);
        state = 0;
        return;
      case cmdPlus:
        plusSpeed();
        break;
      case cmdMinus:
        minusSpeed();
        break;
    }


    if (state == 0) {
      cmd = getCmdIR();
      if (cmd != cmdNo) {
        printCommand(cmd);
      }

      if (cmd == cmdLFR) state = 1;
      readSensors();
      printSensors();
      delay(200);

    } else if (state == 1) {
      digitalWrite(FLED, HIGH);
      digitalWrite(BLED, LOW);
      readSensors();
      byte DL = digitLine() >> 3;

      if (DL == 0B010 || DL == 0B111) {
        digitalWrite(FLED, LOW);
        drive(speed, speed);
      } else if (DL == 0B100) {
        // digitalWrite(FLED, HIGH);
        drive(speed * 1.4, speed * 0.6);
      } else if (DL == 0B110) {
        // digitalWrite(FLED, HIGH);
        drive(speed * 1.2, speed * 0.8);
      } else if (DL == 0B001) {
        // digitalWrite(FLED, HIGH);
        drive(speed * 0.6, speed * 1.4);
      } else if (DL == 0B011) {
        // digitalWrite(FLED, HIGH);
        drive(speed * 0.8, speed * 1.2);
      } else if (DL == 0B000 || DL == 0B101) {
        // digitalWrite(FLED, HIGH);
        drive(0, 0);
        state = -1;
      }
    }

    else if (state == -1) {
      digitalWrite(FLED, LOW);
      digitalWrite(BLED, HIGH);
      readSensors();
      byte DL = digitLine() & 0B00000111;
      if (DL == 0B010 || DL == 0B111) {
        digitalWrite(BLED, LOW);
        drive(-speed, -speed);
      } else if (DL == 0B100) {
        //digitalWrite(BLED, HIGH);
        drive(-speed * 0.6, -speed * 1.4);
      } else if (DL == 0B110) {
        //digitalWrite(BLED, HIGH);
        drive(-speed * 0.8, -speed * 1.2);
      } else if (DL == 0B001) {
        // digitalWrite(BLED, HIGH);
        drive(-speed * 1.4, -speed * 0.6);
      } else if (DL == 0B011) {
        // digitalWrite(BLED, HIGH);
        drive(-speed * 1.2, -speed * 0.8);
      } else if (DL == 0B000 || DL == 0B101) {
        //digitalWrite(BLED, HIGH);
        drive(0, 0);
        state = 1;
      }
    }
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
  pinMode(IR_PIN, INPUT);
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
  delay(500);
  loadEEPROM();
  readCalibration();

  digitalWrite(FLED, LOW);
  digitalWrite(BLED, LOW);

  Serial.print("Speed=");
  Serial.println(speed);
  readSensors();
  printSensors();
}

void loop() {
  if (!digitalRead(BUTTON)) {
    digitalWrite(FLED, HIGH);
    delay(START_DELAY);
    state = 1;
    LFR();
    digitalWrite(FLED, LOW);
  }

  byte cmd = cmdNo;
  cmd = getCmdIR();
  if (cmd != cmdNo) {
    printCommand(cmd);
  }
  switch (cmd) {
    case cmdLFR:
      digitalWrite(FLED, HIGH);
      delay(START_DELAY);
      state = 0;
      LFR();
      digitalWrite(FLED, LOW);
      break;
    case cmdCalibr:
      calibration(10000);
      break;
  }
}
