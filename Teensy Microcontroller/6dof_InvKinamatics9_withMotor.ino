#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

int sr[] = {0,0,1,2,3,4,5};
int sMax[] = {0,495,350,500,360,500,226};
int sMid[] = {0,280,222,285,115,206,100};
int sMin[] = {0,75,130,70,90,70,100};
int servo[] = {0,0,0,0,0,0,100};

int angle[] = {0,0,0,0,0,0,0};
//int aMid[] = {0,0,90,90,90,0,100};  // Box Shape
int aMid[] = {0,0,131,51,92,0,100};   
int aMax[] = {0,45,150,300,180,75,220};
int aMin[] = {0,-90,-25,30,32,-137,100};

// for reciever
//int chPin[] = {0,23,22,21,20,17,16,15};
int chPin[] = {0,1,2,3,4,5,6,7,8,9,10};
int ch[] = {0,0,0,0,0,0,0,0,0,0,0};
int maxCh = 2010, minCh = 985, i;
int tol = 100, fwdTh = 1500+tol, bwdTh = 1500-tol;
  
volatile long StartTime[] = {0,0,0,0,0,0,0,0,0,0,0};
volatile long CurrentTime[] = {0,0,0,0,0,0,0,0,0,0,0};
volatile long Pulses[] = {0,0,0,0,0,0,0,0,0,0,0};

float endLen=121, a=129, b=112;
float x=0, y=b, z=a;
float m=1,c=0;
float alpha,beta,offsetX=0,offsetY=0,offsetYrad=0;
float dz=0,dy=0,zPrm,yPrm;
float theta=45,theta1=0,gama,p;
float r=1,r1=1;

void setup() {
  Serial.begin(9600);
  Serial6.begin(9600);
//  Serial.println("8 channel Servo test!");
//  digitalWrite(13,HIGH);
  rxPinSetup();
  srvoInit();
  angleInit();
  runSrvo();
  delay(10);
}
void loop() {
  delay(10);
  getRxVal(0);
  manual();
  runSrvo();
}
void manual(){
  if(1800<ch[5] && ch[5]<2020){
    if(maxCh>ch[8] && ch[8]>fwdTh+100){
      Serial.println("mode 4");
      motorControl();
    }
    else if(fwdTh+100>ch[8] && ch[8]>bwdTh-100){
      Serial.println("mode 5");
      motorControl();
//      autonomousYZ();
    }
    else if(minCh<ch[8] && ch[8]<bwdTh-100){
      Serial.println("mode 6");
      motorControl();
    }
  }
  else if(900<ch[5] && ch[5]<1100){
    
    if(maxCh>ch[8] && ch[8]>fwdTh+100){
//      Serial.println("mode 3");
      invKinGetValues(3);
      invKinCalc(3);
    }
    else if(fwdTh+100>ch[8] && ch[8]>bwdTh-100){
//      Serial.print("mode 2");
//      Serial.println("AI  !!!!!! ");
      invKinGetValues(2);
      invKinCalc(2);
    }
    else if(minCh<ch[8] && ch[8]<bwdTh-100){
//      Serial.print("mode 1");
      Serial.println("Manual ");
      manualOperation();
    }  
  }
}
void invKinGetValues(int mode){
  if(maxCh>ch[1] && ch[1]>fwdTh){
    if(x<a+b)x--;
//    Serial.print("base clk ");
  }
  else if(minCh<ch[1] && ch[1]<bwdTh){
    if(x>-a-b)x++;
//    Serial.print("base cclk ");
  }
  if(maxCh>ch[2] && ch[2]>fwdTh){
    y++;
  }
  else if(minCh<ch[2] && ch[2]<bwdTh){
    y--;
  }
  if(maxCh>ch[6] && ch[6]>fwdTh){
    if(z>0)z--;
  }
  else if(minCh<ch[6] && ch[6]<bwdTh-100){
    if(z<a+b)z++;
  }
    if(maxCh>ch[3] && ch[3]>fwdTh+100){
      if(mode>1){
        offsetY++;
        if(abs(offsetY)>90)offsetY=90;
      }
      else{
        angle[4]++;
      }
      delay(10);
//      Serial.print("elev up\t");
    }
    else if(minCh<ch[3] && ch[3]<bwdTh){
      if(mode>1){
        offsetY--;
        if(abs(offsetY)>90)offsetY=-90;
      }
      else{
        angle[4]--;
      }
      delay(10);
//      Serial.print("elev down\t");
    }
    if(maxCh>ch[4] && ch[4]>fwdTh){
      if(mode>1){
        offsetX--;
        if(abs(offsetX)>90)offsetX=-90;
      }
      else{
        angle[5]--;
      }
      delay(10);
//      Serial.print("rotate clk\t");
    }
    else if(minCh<ch[4] && ch[4]<bwdTh){
      if(mode>1){
        offsetX++;
        if(abs(offsetX)>90)offsetX=90;
      }
      else{
        angle[5]++;
      }
      delay(10);
//      Serial.print("rotate cclk\t");
    }
  if(maxCh>ch[7] && ch[7]>fwdTh){
    angle[6] = 226;
    Serial.print("Grip ");
  }
  else if(minCh<ch[7] && ch[7]<bwdTh){
    angle[6] = 100;
    Serial.print("Rel ");
  }
}
void invKinCalc(int mode){
  offsetYrad = offsetY*PI/180;
  dy = endLen*sin(offsetYrad);
  dz = endLen*cos(offsetYrad);
  yPrm = y - dy;
  zPrm = z + dz - endLen;

  if(mode>2){
    yPrm = constrain(yPrm, -20, a+b);
    r1 = constrain(sqrt(sq(x)+pow(yPrm,2)), 20, a+b);
    r = constrain(sqrt(sq(r1)+sq(zPrm)), 10, a+b);
  
    if(x>0&&yPrm>0)theta1 = 90-atan(yPrm/x)*180/PI;           //1st quadrant
    if(x<0&&yPrm>0)theta1 = -90+atan(abs(yPrm/x))*180/PI;     //2nd quadrant
      
    theta = atan(zPrm/r1)*180/PI;
    alpha = acos((sq(a) + sq(r) - sq(b)) / (2*a*r))*180/PI;
  
    beta = acos((sq(a) + sq(b) - sq(r)) / (2*a*b))*180/PI;
    beta = constrain(beta,aMin[3],aMax[3]);   // previous limit was only 70
  }
  else{
    y = constrain(y, -20, a+b);
    r1 = constrain(sqrt(sq(x)+pow(y,2)), 20, a+b);
    r = constrain(sqrt(sq(r1)+sq(z)), 20, a+b);
  
    if(x>0&&y>0)theta1 = 90-atan(y/x)*180/PI;           //1st quadrant
    if(x<0&&y>0)theta1 = -90+atan(abs(y/x))*180/PI;     //2nd quadrant
      
    theta = atan(z/r1)*180/PI;
    alpha = acos((sq(a) + sq(r) - sq(b)) / (2*a*r))*180/PI;
  
    beta = acos((sq(a) + sq(b) - sq(r)) / (2*a*b))*180/PI;
    beta = constrain(beta,aMin[3],aMax[3]);   // previous limit was only 70 
  }
  angle[1] = theta1;
  angle[2] = theta + alpha;
  angle[3] = beta;
  if(mode>1)angle[4] = 270+offsetY-(alpha+beta+theta);
  if(mode>1)angle[5] = -theta1*cos(offsetYrad)+offsetX;
  
//  limitAngles();
  char buf[50];
//  sprintf(buf, "r=%.2f r1=%.2f th=%.2f th1=%.2f an1=%d an2=%d an3=%d ( %.1f %.1f %.1f )", r, r1, theta, theta1, angle[1], angle[2], angle[3], x, y, z);
//  sprintf(buf, "al=%.1f bt=%.1f th=%.1f (an1=%d an2=%d an3=%d an4=%d)\t(%.1f %.1f -> %.1f , %.1f -> %.1f)", alpha, beta, theta, angle[1], angle[2], angle[3], angle[4], x, y, yPrm, z, zPrm);
  sprintf(buf, "(an1=%d an2=%d an3=%d an4=%d an5=%d)\t(%.1f %.1f -> %.1f , %.1f -> %.1f)", angle[1], angle[2], angle[3], angle[4], angle[5], x, y, yPrm, z, zPrm);
  Serial.println(buf);
}
void manualOperation(){
  printSrvoVal();  
  if(maxCh>ch[1] && ch[1]>fwdTh){
    angle[1]--;
    Serial.print("base clk ");
  }
  else if(minCh<ch[1] && ch[1]<bwdTh){
    angle[1]++;
    Serial.print("base cclk ");
  }
  
  if(maxCh>ch[2] && ch[2]>fwdTh){
    angle[2]--;
    Serial.print("j1 cclk\t");
  }
  else if(minCh<ch[2] && ch[2]<bwdTh){
    angle[2]++;
    Serial.print("j1 clk\t");
  }
  
  if(minCh<ch[7] && ch[7]<bwdTh){
    if(maxCh>ch[3] && ch[3]>fwdTh){
      angle[3]++;
      Serial.print("j2 cclk\t");
    }
    else if(minCh<ch[3] && ch[3]<bwdTh){
      angle[3]--;
      Serial.print("j2 clk\t");
      
    }
    if(maxCh>ch[4] && ch[4]>fwdTh){
      angle[6]++;
      Serial.print("grip\t");
    }
    else if(minCh<ch[4] && ch[4]<bwdTh){
      angle[6]--;
      Serial.print("release\t");
    }
  }
  else if(maxCh>ch[7] && ch[7]>fwdTh){
    if(maxCh>ch[3] && ch[3]>fwdTh){
      angle[4]++;
      Serial.print("elev up\t");
    }
    else if(minCh<ch[3] && ch[3]<bwdTh){
      angle[4]--;
      Serial.print("elev down\t");
      
    }
    if(maxCh>ch[4] && ch[4]>fwdTh){
      angle[5]--;
      Serial.print("rotate clk\t");
    }
    else if(minCh<ch[4] && ch[4]<bwdTh){
      angle[5]++;
      Serial.print("rotate cclk\t");
    }
  }
  if(maxCh>ch[6] && ch[6]>fwdTh){
    angle[6]++;
    Serial.print("grip\t");
  }
  else if(minCh<ch[6] && ch[6]<bwdTh){
    angle[6]--;
    Serial.print("release\t");
  }
//  Serial.println(" enabled ");
//  limitAngles();
}

void motorControl(){
//  runMotor(10,20);
//  delay(2000);
//  runMotor(0,-20);
//  delay(2000);
//  stopp();
//  delay(2000);
  int spd = 0, turn = 0;
  if(maxCh>ch[1] && ch[1]>fwdTh){
    turn = map(ch[1],fwdTh,maxCh,1,-100);
  }
  else if(minCh<ch[1] && ch[1]<bwdTh){
    turn = map(ch[1],bwdTh,minCh,0,100);
  }
  else spd = 0;
  if(maxCh>ch[2] && ch[2]>fwdTh){
    spd = map(ch[2],fwdTh,maxCh,0,100);
  }
  else if(minCh<ch[2] && ch[2]<bwdTh){
    spd = map(ch[2],bwdTh,minCh,1,-100);
  }
  Serial.print(spd);Serial.print(" ");Serial.println(turn);
  runMotor(spd,turn);
}
int prnt = 1;
void runMotor(int s, int t){
  int lspd, rspd, newSpeed;
  lspd = s-t; rspd = s+t;
  lspd = constrain(lspd,-100,100);
  rspd = constrain(rspd,-100,100);
  if(prnt){Serial.print("Left = ");Serial.print(lspd);Serial.print(" ");Serial.print("Right = ");Serial.println(rspd);}
  
  if(lspd<0){
    lspd = abs(lspd);  
    newSpeed=map(lspd,1,100,193,255);Serial6.write(newSpeed); // L bwd
  }
  else{
    newSpeed=map(lspd,0,100,192,128);Serial6.write(newSpeed); // L fwd 192 stop value
  }
  if(rspd<0){
    rspd = abs(rspd);  
    newSpeed=map(rspd,1,100,63,1);Serial6.write(newSpeed); // R bwd
  }
  else{
    newSpeed=map(rspd,0,100,64,127);Serial6.write(newSpeed);  // R fwd 64 stop value
  }

//  int newSpeed=map(rspd,0,100,65,127);Serial6.write(newSpeed);
//  newSpeed=map(lspd,0,100,191,128);Serial6.write(newSpeed);
}
void autonomousXY(){
  int del = 5;
  for(int i=-40;i<=40;i++){
    delay(del);
    circleXY(i,1);
  }
  for(int i=40;i>=-40;i--){
    delay(del);
    circleXY(i,-1);
  }
}
void autonomousYZ(){
  int del = 10;
  for(int i=35;i<=115;i++){
    delay(del);
    circleYZ(i,1);
    runSrvo();
  }
  for(int i=115;i>=35;i--){
    delay(del);
    circleYZ(i,-1);
    runSrvo();
  }
}
void circleXY(float x, int dir){
//  0,140,65  r=40
  z=65;
  y = dir*sqrt(sq(40) - sq(x))+140;
  
    y = constrain(y, -20, a+b);
    r1 = constrain(sqrt(sq(x)+pow(y,2)), 20, a+b);
    r = constrain(sqrt(sq(r1)+sq(z)), 20, a+b);
  
    if(x>0&&y>0)theta1 = 90-atan(y/x)*180/PI;           //1st quadrant
    if(x<0&&y>0)theta1 = -90+atan(abs(y/x))*180/PI;     //2nd quadrant
      
    theta = atan(z/r1)*180/PI;
    alpha = acos((sq(a) + sq(r) - sq(b)) / (2*a*r))*180/PI;
  
    beta = acos((sq(a) + sq(b) - sq(r)) / (2*a*b))*180/PI;
    beta = constrain(beta,aMin[3],aMax[3]);   // previous limit was only 70 
  
  angle[1] = theta1;
  angle[2] = theta + alpha;
  angle[3] = beta;
  angle[4] = 270+offsetY-(alpha+beta+theta);
  angle[5] = -theta1*cos(offsetYrad)+offsetX;
  
//  limitAngles();
  char buf[50];
//  sprintf(buf, "r=%.2f r1=%.2f th=%.2f th1=%.2f an1=%d an2=%d an3=%d ( %.1f %.1f %.1f )", r, r1, theta, theta1, angle[1], angle[2], angle[3], x, y, z);
  sprintf(buf, "al=%.1f bt=%.1f th=%.1f (an1=%d an2=%d an3=%d an4=%d)\t(%.1f -> %.1f , %.1f -> %.1f)", alpha, beta, theta, angle[1], angle[2], angle[3], angle[4], y, yPrm, z, zPrm);
  Serial.println(buf);
}
void circleYZ(float z, int dir){
//  0,140,75  r=40
  x = 0;
  y = dir*sqrt(sq(40) - sq(z-75))+140;

  y = constrain(y, -20, a+b);
  r1 = constrain(sqrt(sq(x)+pow(y,2)), 20, a+b);
  r = constrain(sqrt(sq(r1)+sq(z)), 20, a+b);

  if(x>0&&y>0)theta1 = 90-atan(y/x)*180/PI;           //1st quadrant
  if(x<0&&y>0)theta1 = -90+atan(abs(y/x))*180/PI;     //2nd quadrant
    
  theta = atan(z/r1)*180/PI;
  alpha = acos((sq(a) + sq(r) - sq(b)) / (2*a*r))*180/PI;

  beta = acos((sq(a) + sq(b) - sq(r)) / (2*a*b))*180/PI;
  beta = constrain(beta,aMin[3],aMax[3]);   // previous limit was only 70 

  angle[1] = theta1;
  angle[2] = theta + alpha;
  angle[3] = beta;
  angle[4] = 270+offsetY-(alpha+beta+theta);
  angle[5] = -theta1*cos(offsetYrad)+offsetX;
  
//  limitAngles();
  char buf[50];
//  sprintf(buf, "r=%.2f r1=%.2f th=%.2f th1=%.2f an1=%d an2=%d an3=%d ( %.1f %.1f %.1f )", r, r1, theta, theta1, angle[1], angle[2], angle[3], x, y, z);
//  sprintf(buf, "al=%.1f bt=%.1f th=%.1f (an1=%d an2=%d an3=%d an4=%d)\t(%.1f -> %.1f , %.1f -> %.1f)", alpha, beta, theta, angle[1], angle[2], angle[3], angle[4], y, yPrm, z, zPrm);
  sprintf(buf, "(an1=%d an2=%d an3=%d an4=%d an4=%d)\t(%.1f -> %.1f , %.1f -> %.1f)", angle[1], angle[2], angle[3], angle[4], angle[5], y, yPrm, z, zPrm);
  Serial.println(buf);
}

void printSrvoVal(){
  char line[100];
  sprintf(line, "%d %d %d %d %d %d , %d %d %d %d %d %d ", 
  servo[1],
  servo[2],
  servo[3],
  servo[4],
  servo[5],
  servo[6],
  angle[1],
  angle[2],
  angle[3],
  angle[4],
  angle[5],
  angle[6]
  );
  Serial.print(line);
}
void rxPinSetup(){
  for(int i=1;i<=8;i++){
    pinMode(chPin[i],INPUT_PULLUP);
  }
  // ch6-swD  ch7-swC  ch8-swA
  attachInterrupt(digitalPinToInterrupt(chPin[1]),PulseTimerA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(chPin[2]),PulseTimerB,CHANGE);
  attachInterrupt(digitalPinToInterrupt(chPin[3]),PulseTimerC,CHANGE);
  attachInterrupt(digitalPinToInterrupt(chPin[4]),PulseTimerD,CHANGE);
  attachInterrupt(digitalPinToInterrupt(chPin[5]),PulseTimerE,CHANGE);
  attachInterrupt(digitalPinToInterrupt(chPin[6]),PulseTimerF,CHANGE);
  attachInterrupt(digitalPinToInterrupt(chPin[7]),PulseTimerG,CHANGE);
  attachInterrupt(digitalPinToInterrupt(chPin[8]),PulseTimerH,CHANGE);
}
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}
void srvoInit(){
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}
void angleInit(){
  for(int i = 1; i<=6; i++){
    angle[i] = aMid[i];
  }
}
void limitAngles(){
  for(int i=1;i<=6;i++){
    angle[i] = constrain(angle[i],aMin[i],aMax[i]);
  }
}
void mapAngle(){
  limitAngles();
//280
  servo[1] = map(angle[1],0,-90,215,500);  
  servo[2] = map(angle[2],0,90,423,304);
  servo[3] = map(angle[3],90,180,150,280);
  servo[4] = map(angle[4],90,180,270,92);
  servo[5] = map(angle[5],-90,0,402,209);
  servo[6] = angle[6];
}
void runSrvo(){
  mapAngle();
  for(int i=1;i<=6;i++){
//    servo[i] = constrain(servo[i],sMin[i],sMax[i]);
    pwm.setPWM(sr[i], 0, servo[i]);
  }
}
void getRxVal(int prnt){
  for(int i=1;i<=8;i++){
    if(Pulses[i] < 2100)ch[i] = Pulses[i];
  }
  if(prnt){
    for(int i=1;i<=8;i++){
      Serial.print(ch[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
void PulseTimerA(){
  int i = 1;
  CurrentTime[i] = micros();
  if(CurrentTime[i] > StartTime[i]){
    Pulses[i] = CurrentTime[i] - StartTime[i];
    StartTime[i] = CurrentTime[i];
  }
}
void PulseTimerB(){
  int i = 2;
  CurrentTime[i] = micros();
  if(CurrentTime[i] > StartTime[i]){
    Pulses[i] = CurrentTime[i] - StartTime[i];
    StartTime[i] = CurrentTime[i];
  }
}
void PulseTimerC(){
  int i = 3;
  CurrentTime[i] = micros();
  if(CurrentTime[i] > StartTime[i]){
    Pulses[i] = CurrentTime[i] - StartTime[i];
    StartTime[i] = CurrentTime[i];
  }
}
void PulseTimerD(){
  int i = 4;
  CurrentTime[i] = micros();
  if(CurrentTime[i] > StartTime[i]){
    Pulses[i] = CurrentTime[i] - StartTime[i];
    StartTime[i] = CurrentTime[i];
  }
}
void PulseTimerE(){
  int i = 5;
  CurrentTime[i] = micros();
  if(CurrentTime[i] > StartTime[i]){
    Pulses[i] = CurrentTime[i] - StartTime[i];
    StartTime[i] = CurrentTime[i];
  }
}
void PulseTimerF(){
  int i = 6;
  CurrentTime[i] = micros();
  if(CurrentTime[i] > StartTime[i]){
    Pulses[i] = CurrentTime[i] - StartTime[i];
    StartTime[i] = CurrentTime[i];
  }
}
void PulseTimerG(){
  int i = 7;
  CurrentTime[i] = micros();
  if(CurrentTime[i] > StartTime[i]){
    Pulses[i] = CurrentTime[i] - StartTime[i];
    StartTime[i] = CurrentTime[i];
  }
}
void PulseTimerH(){
  int i = 8;
  CurrentTime[i] = micros();
  if(CurrentTime[i] > StartTime[i]){
    Pulses[i] = CurrentTime[i] - StartTime[i];
    StartTime[i] = CurrentTime[i];
  }
}
