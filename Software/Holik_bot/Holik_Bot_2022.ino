//Робот-препятствие для Большого путешествия
//двигается по черной линии до разрыва и обратно
//Изготовлено по заказу Алексея Голика
//2022. pinMode
// драйвер L298N

const int BUZZ = 4;
const int MOTOR_L_DIR = 13;
const int MOTOR_R_DIR = 12;
const int MOTOR_R_PWM = 10;
const int MOTOR_L_PWM = 11;
const int numSens = 6;      //число датчиков
int sensPin[] = {A3, A4, A5, A0, A1, A2};
const int SPEED = 40;       //скорость движения
int dir = 1;                //первоначальное направление движения

void beep(int ms) {
  digitalWrite(BUZZ, 1);
  delay(ms);
  digitalWrite(BUZZ, 0);
}
void drive(int left, int right) {       //управление моторами
  if (left >= 0) {
    digitalWrite(MOTOR_L_DIR, 1);
    analogWrite(MOTOR_L_PWM,  left);
  }
  else {
    digitalWrite(MOTOR_L_DIR, 0);
    analogWrite(MOTOR_L_PWM, - left);
  }

  if (right >= 0) {
    digitalWrite(MOTOR_R_DIR, 1);
    analogWrite(MOTOR_R_PWM,  right);
  }
  else {
    digitalWrite(MOTOR_R_DIR, 0);
    analogWrite(MOTOR_R_PWM, - right);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(BUZZ, OUTPUT);
  pinMode(MOTOR_L_DIR, OUTPUT);
  pinMode(MOTOR_R_DIR, OUTPUT);
  for (int i = 0; i < numSens; i++) {
    pinMode(sensPin[i], INPUT);
  }
  beep(500);
}

void loop() {
  byte DL = 0;
  byte    DLF;
  byte w = 32;
  for (int i = 0; i < numSens; i++) {
    DL = DL + w * digitalRead(sensPin[i]);
    w = w / 2;
  }
  //Serial.println(DL);
  if (DL == 0) {
    drive(-dir * SPEED, dir * SPEED);
    return;
  }

  if (dir == 1) { // едем вперед
    DLF = DL / 8;
    if (DLF == B010 || DLF == B111 ) { // линия по центру || DLF == B101
      drive ( SPEED,  SPEED);
    }
    else if (DLF == B110) {
      drive(0,  SPEED );
    }
    else if (DLF == B100) {
      drive(0,  SPEED);
    }
    else if (DLF == B011) {
      drive(SPEED , 0);
    }
    else if (DLF == B001) {
      drive( SPEED, 0);
    }
    else {// остальные случаи
      drive(0, 0);
      dir = -1 ;             //меняем направление
      //beep(100);
    }


  }
  else   {          //едем назад
    DLF = DL % 8;

    if (DLF == B010 || DLF == B111 ) { // линия по центру || DLF == B101
      drive (- SPEED, - SPEED);
    }
    else if (DLF == B110) {
      drive( - SPEED , 0);
    }
    else if (DLF == B100) {
      drive( - SPEED, 0);
    }
    else if (DLF == B011) {
      drive(0, - SPEED );
    }
    else if (DLF == B001) {
      drive(0, - SPEED);
    }
    else {// остальные случаи
      drive(0, 0);
      dir = 1;             //меняем направление
      //beep(100);
    }
  }
}
