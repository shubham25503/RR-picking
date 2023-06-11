/*
    shooter left    shooter right
  rack levels
  left   /         /  right
       /         /
      /         /
  (leftLs2)        (rightLs2) level 2 (resetLevel)

    /       /
   /       /
  /       /
  (leftLs1)        (rightLs1) level 1


  Servo Levels
              left         right  level 2 (reset level) (45 degree)
                    /      /
                   /      /
                  /      /

  left  --------- level 1 (90 degree)
  right ---------
*/
#include <EspNow.h>
#include <Motor.h>
#include <positionalnew.h>
#include <ESP32Servo.h>

Servo rightServo, leftServo;

Peer remote;
Peer rrShooter;
JSONVar feedback;
//Motor leftMotor(4, 18);
//Motor rightMotor(19, 21);
//UniversalEncoder rightEncoder(25, 33, -1);
//UniversalEncoder leftEncoder(26, 27, 1);
Motor leftMotor(23, 22);
Motor rightMotor(19, 18 );
UniversalEncoder rightEncoder(32, 33, -1);
UniversalEncoder leftEncoder(26, 25 , -1);
//Motor leftMotor(32,33);
//Motor rightMotor(25,26);
//UniversalEncoder rightEncoder(34,35, -1);
//UniversalEncoder leftEncoder(18,19, 1);
positionalnew rightMPID(&rightMotor);
positionalnew leftMPID(&leftMotor);

double AggKpLeft = 1.0, AggKiLeft = 0.0, AggKdLeft = 0;
double SoftKpLeft = 0.75, SoftKiLeft = 0.0, SoftKdLeft = 0;
double AggKpRight = 0.90, AggKiRight = 0.0, AggKdRight = 0;
double SoftKpRight = 0.675, SoftKiRight = 0, SoftKdRight = 0;

int rightLsUp = 17, leftLsUp = 4, pneumaticPin = 21;
int resetPulse = -50000, pulseDiff = 100, degreeDiff = 5;
int rightServoPin = 27, leftServoPin = 13 , degree, rakPWM = 0, pwmdiff = 5;
int degOffSet = -10 ,pulseoffset = -50;
int rlvl = 0;
long rightPulse = 0, leftPulse = 0, rLvl2Pulse = 1600, rLvl1Pulse = 0, rightDiffPulse, leftDiffPulse;

bool rLs1 = false, rLs2 = false, lLs1 = false, lLs2 = false;
bool init_ = false, upBool = false;
bool pnOC = true , startLvl1 = false , startLvl2 = false, intiUpperLimitLeft = false , intiUpperLimitRight = false, intiLowerLimitLeft = false , intiLowerLimitRight = false;
int currLeft, currRight;
void setup()
{
  Serial.begin(115200);
  pinMode(pneumaticPin, OUTPUT);
  pinMode(rightLsUp, INPUT_PULLUP);
  pinMode(leftLsUp, INPUT_PULLUP);

  rightServo.attach(rightServoPin);
  leftServo.attach(leftServoPin);

  leftMotor.invertDirection();
  leftMotor.setEncoder(&leftEncoder);
  rightMotor.setEncoder(&rightEncoder);

  //rightMPID.setThreshold(100);
  rightMPID.setOutputLimits(-35, 35);
  rightMPID.setAggTunings(AggKpLeft, AggKiLeft, AggKdLeft);
  rightMPID.setSoftTunings(SoftKpLeft, SoftKiLeft, SoftKdLeft);

  //leftMPID.setThreshold(100);
  leftMPID.setOutputLimits(-35, 35);
  leftMPID.setAggTunings(AggKpRight, AggKiRight, AggKdRight);
  leftMPID.setSoftTunings(SoftKpRight, SoftKiRight, SoftKdRight);

  setId("PiCSR");
  remote.init("ReCON");
   rrShooter.init("RsHTr");


  remote.setOnRecieve(rackLvl1, "Rlvl1");
  remote.setOnRecieve(rackLvl2, "Rlvl2");
  remote.setOnRecieve(servoLvl1, "Slvl1");
  remote.setOnRecieve(servoLvl2, "Slvl2");
  remote.setOnRecieve(rackExtraPPulse, "Rexpp");
  remote.setOnRecieve(rackExtraNPulse, "Rexpn");
  remote.setOnRecieve(servoExtraPDeg, "Sexdp");
  remote.setOnRecieve(servoExtraNDeg, "Sexdn");
  remote.setOnRecieve(pneumaticOC, "pnpicking");
  remote.setOnRecieve(servoSetting, "servoSet");
  remote.setOnRecieve(stopBot, "stopBot");
  remote.setOnRecieve(resetAll, "rst");
//  rrShooter.setOnRecieve(autoServo, "pnCls");
//  rrShooter.setOnRecieve(autoServo2, "pnOpn");
}
void loop()
{
  if (Serial.available() > 0) {
    degree = Serial.readString().toInt();
    leftServo.write(degree);
    rightServo.write(180 - degree);
    Serial.println(String(degree));
  }

  leftPulse = leftMotor.getReadings();
  rightPulse = rightMotor.getReadings();
//  Serial.println("left: " + String(leftPulse) + " right: " + String(rightPulse));
  rLs1 = !(bool)digitalRead(rightLsUp);
  lLs1 = !(bool)digitalRead(leftLsUp);
//  Serial.println(String(rLs1) + " " + String(lLs1));

  if (!init_)
  {
    if ((!lLs1 || !rLs1) && !startLvl1) // will go up initially to level 1
    {
      startLvl1 = true;
      leftMPID.setPulse(resetPulse);
      rightMPID.setPulse(resetPulse);
      Serial.println("Go to level 1");
    }

    else if (lLs1 && !intiUpperLimitLeft) // left motor reached lower
    {
      Serial.println("left reached level 1");
      intiUpperLimitLeft = true;
      leftEncoder.setReadings(rLvl1Pulse);
      leftMPID.setPulse(rLvl1Pulse);
      leftMotor.reset();
    }

    if (rLs1 && !intiUpperLimitRight) // right motor reached lower
    {
      Serial.println("right reached level 1");
      intiUpperLimitRight = true;
      rightEncoder.setReadings(rLvl1Pulse);
      rightMPID.setPulse(rLvl1Pulse);
      rightMotor.reset();

    }

    if (intiUpperLimitLeft && intiUpperLimitRight) // both motor reached level 1 and now goto level 2
    {

      Serial.println("REached levlel1");
      init_ = true;

    }
  }
  if (rlvl == 2 && leftPulse >= 500 && rightPulse >= 500)
  {
    degree = 35;
    leftServo.write(degree);
    rightServo.write(180 - degree - degOffSet);
    Serial.println(String(degree) + "servo2 automated");
    rlvl = 0;
  }
  rightMPID.compute();
  leftMPID.compute();

}

void rackLvl1(JSONVar msg)
{
  //  leftMPID.setPulse(rLvl1Pulse);
  //  rightMPID.setPulse(rLvl1Pulse);
  Serial.println("rak1");
  rightMPID.setThreshold(rLvl2Pulse / 2);
  rightMPID.setOutputLimits(-45, 45);
  rightMPID.setAggTunings(AggKpLeft, AggKiLeft, AggKdLeft);
  rightMPID.setSoftTunings(SoftKpLeft, SoftKiLeft, SoftKdLeft);

  leftMPID.setThreshold(rLvl2Pulse / 2);
  leftMPID.setOutputLimits(-45, 45);
  leftMPID.setAggTunings(AggKpRight, AggKiRight, AggKdRight);
  leftMPID.setSoftTunings(SoftKpRight, SoftKiRight, SoftKdRight);
  startLvl1 = false ;
  startLvl2 = false;
  intiUpperLimitLeft = false;
  intiUpperLimitRight = false;
  intiLowerLimitLeft = false;
  intiLowerLimitRight = false;
  //  rlvl = 1;
  init_ = false;
}
void rackLvl2(JSONVar msg)
{
  rightMPID.setThreshold(rLvl2Pulse / 2);
  rightMPID.setOutputLimits(-40, 40);
  rightMPID.setAggTunings(AggKpLeft, AggKiLeft, AggKdLeft);
  rightMPID.setSoftTunings(SoftKpLeft, SoftKiLeft, SoftKdLeft);

  leftMPID.setThreshold(rLvl2Pulse / 2);
  leftMPID.setOutputLimits(-40, 40);
  leftMPID.setAggTunings(AggKpRight, AggKiRight, AggKdRight);
  leftMPID.setSoftTunings(SoftKpRight, SoftKiRight, SoftKdRight);
  leftMPID.setPulse(rLvl2Pulse + pulseoffset);
  rightMPID.setPulse(rLvl2Pulse);
  rlvl = 2;
  Serial.println("rac2");
}
void servoLvl1(JSONVar msg)
{
  degree = 80;
  leftServo.write(degree);
  rightServo.write(180 - degree  - degOffSet);
  Serial.println(String(degree) + " servo1");
}
void servoLvl2(JSONVar msg)
{
  degree = 40;
  leftServo.write(degree);
  rightServo.write(180 - degree - degOffSet);
  Serial.println(String(degree) + "servo2");
}
void rackExtraNPulse(JSONVar msg)
{
  rightMPID.setThreshold(rightPulse + (pulseDiff / 2));
  rightMPID.setOutputLimits(-50, 50);
  rightMPID.setAggTunings(AggKpLeft, AggKiLeft, AggKdLeft);
  rightMPID.setSoftTunings(SoftKpLeft, SoftKiLeft, SoftKdLeft);
  rightMPID.setPulse(rightPulse + pulseDiff);

  leftMPID.setThreshold(leftPulse + (pulseDiff / 2));
  leftMPID.setOutputLimits(-50, 50);
  leftMPID.setAggTunings(AggKpRight, AggKiRight, AggKdRight);
  leftMPID.setSoftTunings(SoftKpRight, SoftKiRight, SoftKdRight);
  leftMPID.setPulse(leftPulse + pulseDiff);
  Serial.println("extra rak pos");
}
void rackExtraPPulse(JSONVar msg)
{
  rightMPID.setThreshold(rightPulse - (pulseDiff / 2));
  rightMPID.setOutputLimits(-50, 50);
  rightMPID.setAggTunings(AggKpLeft, AggKiLeft, AggKdLeft);
  rightMPID.setSoftTunings(SoftKpLeft, SoftKiLeft, SoftKdLeft);
  //  rightDiffPulse = ((rightPulse - pulseDiff) < 0) ? 0 : (rightPulse - pulseDiff);
  //  rightMPID.setPulse(rightDiffPulse);
  rightMPID.setPulse(rightPulse - pulseDiff);

  leftMPID.setThreshold(leftPulse - (pulseDiff / 2));
  leftMPID.setOutputLimits(-50, 50);
  leftMPID.setAggTunings(AggKpRight, AggKiRight, AggKdRight);
  leftMPID.setSoftTunings(SoftKpRight, SoftKiRight, SoftKdRight);
  //  leftDiffPulse = ((leftPulse - pulseDiff) < 0) ? 0 : (leftPulse - pulseDiff);
  //  leftMPID.setPulse(leftDiffPulse);
  leftMPID.setPulse(leftPulse - pulseDiff);
  Serial.println("extra rak neg");
}
void servoExtraPDeg(JSONVar msg)
{

  degree = degree + degreeDiff;
  degree = degree > 180 ? 180 : degree;
  leftServo.write(degree);
  rightServo.write(180 - degree - degOffSet);
  Serial.println(String(degree) + "extra servo pos");
}
void servoExtraNDeg(JSONVar msg)
{
  degree = degree - degreeDiff;
  degree = degree < 0 ? 0 : degree;
  leftServo.write(degree);
  rightServo.write(180 - degree - degOffSet);
  Serial.println(String(degree) + "extra servo neg");
}
void pneumaticOC(JSONVar msg)
{
  if (pnOC == true) {
    Serial.println("Pneumatic Open");
    digitalWrite(21, HIGH);
    pnOC = false;
  }
  else if (pnOC == false) {
    Serial.println("Pneumatic Close");
    digitalWrite(21, LOW);
    pnOC = true;
  }
  Serial.println(JSON.stringify(msg));
}
void resetAll(JSONVar msg)
{
  //   digitalWrite(pneumaticPin, HIGH);
  startLvl1 = false ;
  startLvl2 = false;
  intiUpperLimitLeft = false;
  intiUpperLimitRight = false;
  intiLowerLimitLeft = false;
  intiLowerLimitRight = false;
  init_ = false;
  rightMPID.setOutputLimits(-30, 30);
  rightMPID.setAggTunings(AggKpLeft, AggKiLeft, AggKdLeft);
  rightMPID.setSoftTunings(SoftKpLeft, SoftKiLeft, SoftKdLeft);

  leftMPID.setOutputLimits(-30, 30);
  leftMPID.setAggTunings(AggKpRight, AggKiRight, AggKdRight);
  leftMPID.setSoftTunings(SoftKpRight, SoftKiRight, SoftKdRight);
  Serial.println("rst");
}

void stopBot(JSONVar msg)
{
  leftMPID.setPulse(leftPulse);
  rightMPID.setPulse(rightPulse);
}
void servoSetting(JSONVar msg) {
  degree = 115;
  leftServo.write(degree);
  rightServo.write(180 - degree - degOffSet);
  Serial.println("servo2");
}
/*
void autoServo(JSONVar msg)
{
  degree = degree - 15;
  leftServo.write(degree);
  rightServo.write(180 - degree);
  Serial.print("Degree "+String(degree));
}
void autoServo2(JSONVar msg)
{
  degree = degree + 15;
  leftServo.write(degree);
  rightServo.write(180 - degree);
  Serial.print("Degree "+String(degree));
}*/
