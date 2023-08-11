#include <SoftwareSerial.h>
#include <Servo.h>
Servo myservo;
#define ea 2
#define eb 7
#define mrf 3
#define mrb 4
#define mlf 5
#define mlb 6
#define brx 12
#define btx 13
#define sl 9
#define sm 10
#define sr 11
#define srvo 8
#define pump A0
#define Led A1
char t;
int pos;
int slv, srv, smv;
boolean fire = false;
SoftwareSerial mySerial (btx, brx);
void setup() {
  pinMode(ea, OUTPUT);
  pinMode(eb, OUTPUT);
  pinMode(mlf, OUTPUT);
  pinMode(mlb, OUTPUT);
  pinMode(mrf, OUTPUT);
  pinMode(mrb, OUTPUT);
  pinMode(pump, OUTPUT);
  pinMode(srvo, OUTPUT);
  Serial.begin(9600);
  pinMode(btx, INPUT);
  pinMode(brx, OUTPUT);
  mySerial.begin(9600);
  digitalWrite(ea, 255);
  digitalWrite(eb, 255);
  pinMode(sl, INPUT);
  pinMode(sm, INPUT);
  pinMode(sr, INPUT);
  myservo.attach(srvo);
  myservo.write(90);
}

void loop() {
  myservo.write(90);
  if (mySerial.available() > 0) {
    t = mySerial.read();
    Serial.println(t);
    stop();


    if (t == 'F') {          //move  forward(all motors rotate in forward direction)
      GoF();
    }

    else if (t == 'B') {    //move reverse (all  motors rotate in reverse direction)
      GoB();
    }

    else if (t == 'L') {    //turn right (left side motors rotate in forward direction,  right side motors doesn't rotate)
      GoL();
    }

    else  if (t == 'R') {    //turn left (right side motors rotate in forward direction, left  side motors doesn't rotate)
      GoR();
    }

    else  if (t == 'G') {    //turn left (right side motors rotate in forward direction, left  side motors doesn't rotate)
      GoFL();
    }
    else  if (t == 'I') {    //turn left (right side motors rotate in forward direction, left  side motors doesn't rotate)
      GoFR();
    }
    else if (t ==  'H') {  //turn led on or off)
      GoBL();
    }
    else if (t ==  'J') {  //turn led on or off)
      GoBR();
    }
    else if (t == 'X') {
      senseAndDo();
    }
    else if (t == 'x') {
      exit;
      Serial.println("auto mode break ");
    }
    else if (t == 'V') {
      stop();
      PumpOn();

    }

    else if (t == 'v') {
      PumpOf();
    }
    else {      //STOP (all motors stop)
      stop();
    }
  }
  delay(10);
}

void GoF() {

  digitalWrite(ea, 255);
  digitalWrite(eb, 255);
  digitalWrite(mlf, HIGH);
  digitalWrite(mlb, LOW);
  digitalWrite(mrf, HIGH);
  digitalWrite(mrb, LOW);

}

void GoB() {
  digitalWrite(ea, 200);
  digitalWrite(eb, 200);
  digitalWrite(mlf, LOW);
  digitalWrite(mlb, HIGH);
  digitalWrite(mrf, LOW);
  digitalWrite(mrb, HIGH);

}
void GoL() {
  digitalWrite(ea, 200);
  digitalWrite(eb, 200);
  digitalWrite(mlf, LOW);
  digitalWrite(mlb, HIGH);
  digitalWrite(mrf, HIGH);
  digitalWrite(mrb, LOW);

}
void GoR() {
  digitalWrite(ea, 200);
  digitalWrite(eb, 200);
  digitalWrite(mlf, HIGH);
  digitalWrite(mlb, LOW);
  digitalWrite(mrf, LOW);
  digitalWrite(mrb, HIGH);

}

void GoFL() {
  digitalWrite(ea, 200);
  digitalWrite(eb, 200);
  digitalWrite(mlf, LOW);
  digitalWrite(mlb, LOW);
  digitalWrite(mrf, HIGH);
  digitalWrite(mrb, LOW);

}
void GoFR() {
  digitalWrite(ea, 200);
  digitalWrite(eb, 200);
  digitalWrite(mlf, HIGH);
  digitalWrite(mlb, LOW);
  digitalWrite(mrf, LOW);
  digitalWrite(mrb, LOW);

}
void GoBL() {
  digitalWrite(ea, 200);
  digitalWrite(eb, 200);
  digitalWrite(mlf, LOW);
  digitalWrite(mlb, HIGH);
  digitalWrite(mrf, LOW);
  digitalWrite(mrb, LOW);

}
void GoBR() {
  digitalWrite(ea, 200);
  digitalWrite(eb, 200);
  digitalWrite(mlf, LOW);
  digitalWrite(mlb, LOW);
  digitalWrite(mrf, LOW);
  digitalWrite(mrb, HIGH);

}
void stop() {
  digitalWrite(ea, 0);
  digitalWrite(eb, 0);
  digitalWrite(mlf, LOW);
  digitalWrite(mlb, LOW);
  digitalWrite(mrf, LOW);
  digitalWrite(mrb, LOW);

}

void PumpOn() {
  Serial.println("manual pump operation enabled");
    stop();
  digitalWrite(pump, HIGH); delay(500);
  for (pos = 50; pos <= 130; pos += 1) {
    myservo.write(pos);
    delay(10);
  }
  for (pos = 130; pos >= 50; pos -= 1) {
    myservo.write(pos);
    delay(10);
  }
  }
void PumpOf() {
  Serial.println("manual pump operation ended");
  digitalWrite(pump,LOW);
  myservo.write(90);
  }

void senseAndDo() {
  while (t != 'x') {
    t = mySerial.read();
    Serial.println(t);
    Serial.println("auto mode one ");
    slv=digitalRead(sl);
    srv=digitalRead(sr);
    smv=digitalRead(sm);
    Serial.println(slv);
    Serial.println(srv);
    Serial.println(smv);
    if (slv == 1 && srv == 1 && smv == 1) //If Fire not detected all sensors are zero
    {
      //Do not move the robot
      stop();
    }

    else if (smv == 0) //If Fire is straight ahead
    {
      //Move the robot forward
     GoF();
      fire = true;
    }
    else if (slv == 0) //If Fire is to the left
    {
      //Move the robot left
      GoL();
    }

    else if (srv == 0) //If Fire is to the right
    {
      //Move the robot right

      GoR();
    }

    delay(300); //Slow down the speed of robot
    while (fire == true)
    {
      put_off_fire();
    }
  }
}


void put_off_fire() {
  Serial.println("putting off fire");
  delay (500);
  stop();
  digitalWrite(pump, HIGH); delay(500);
  for (pos = 50; pos <= 130; pos += 1) {
    myservo.write(pos);
    delay(10);
  }
  for (pos = 130; pos >= 50; pos -= 1) {
    myservo.write(pos);
    delay(10);
  }
  digitalWrite(pump, LOW);
  myservo.write(90);
  fire = false;
}