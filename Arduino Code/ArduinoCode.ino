#include <Servo.h>

//Servo
//8 MG996 motors were used in the setup. 2 motors for each arm. One at the top and other ath the bottom
//Top Motor - Horizontal Movement. Bottom Motor - Vertical Movement.
//Abbreviations : L-Left,R-Right  | F-Front,B-Back  | T-Top,B-Bottom
//Example       : LFT - Left Front Top Servo Motor

Servo servoLFT;
Servo servoLFB;
Servo servoLBT;
Servo servoLBB;

Servo servoRFT;
Servo servoRFB;
Servo servoRBT;
Servo servoRBB;

int servoLFTPin = 4;
int servoLFBPin = 5;
int servoLBTPin = 6;
int servoLBBPin = 7;

int servoRFTPin = 8;
int servoRFBPin = 9;
int servoRBTPin = 10;
int servoRBBPin = 11;

int vAngle = 0;         //Vertical turning angle [ |\ ] from perpendicular to the ground.   //40
int hAngle = 60;        //Horizontal turning angle [ |/ ] from perpendicular to the body.   //60
int vOrigin=130;        //Angle at wich arm is perpendicular to the ground. Right Arm       //90
int motionDelay=200;    //Delay at each movvement in miliSeconds                            //200
int vOriginL=vOrigin-((vOrigin-90)*2);//Angle at wich arm is perpendicular to the ground. Left Arm       //90

//IR Sensor
//Two rows of IR sensors were initalied. One to detect the arrow and the other to detect the line.
//Arrow detection : 3 IR sensors. Left Front Ringt as a triangle.
//Line detection  : 5 IR sensors. Left MiddleLeft Middle MiddleRight Right
//irFL-IR Front Left

int irFL=50;
int irF=48;
int irFR=46;

int irL=40;
int irML=38;
int irM=36;
int irMR=34;
int irR=32;

int reading_irFL;
int reading_irF;
int reading_irFR;

int reading_irL;
int reading_irML;
int reading_irM;
int reading_irMR;
int reading_irR;

//Ultrasonic sensor
int trig=24;
int echo=22;

//flag
int flag=0;

void setup() {
  //servo setup
  pinMode(servoLFTPin,OUTPUT);
  pinMode(servoLFBPin,OUTPUT);
  pinMode(servoLBTPin,OUTPUT);
  pinMode(servoLBBPin,OUTPUT);

  pinMode(servoRFTPin,OUTPUT);
  pinMode(servoRFBPin,OUTPUT);
  pinMode(servoRBTPin,OUTPUT);
  pinMode(servoRBBPin,OUTPUT);

  servoLFT.attach(servoLFTPin);
  servoLFB.attach(servoLFBPin);
  servoLBT.attach(servoLBTPin);
  servoLBB.attach(servoLBBPin);

  servoRFT.attach(servoRFTPin);
  servoRFB.attach(servoRFBPin);
  servoRBT.attach(servoRBTPin);
  servoRBB.attach(servoRBBPin);

  //IR setup
  pinMode(irFL,INPUT);
  pinMode(irF,INPUT);
  pinMode(irFR,INPUT);  

  pinMode(irL,INPUT);
  pinMode(irML,INPUT);
  pinMode(irM,INPUT);
  pinMode(irMR,INPUT);
  pinMode(irR,INPUT);

  Serial.begin(115200);
  delay(2000);
  resetParameters();
  reset();
  delay(10000); 
}

void loop() {
  //latest logic to solve the maze
  calibrate();

  //to skip first white space
  if((reading_irL==1)&&(reading_irR==1)){
    flag=1;
  }
  
  if(distance(trig,echo)<10){
    //fowardHigh_stepCount(5);
    fowardHigh();
  }else{
    if((reading_irL==1)&&(reading_irML==1)&&(reading_irM==1)&&(reading_irMR==1)&&(reading_irR==1)&& flag==1){
      //all white. stop
      while(true){
        h90();
        v90();
      }
    }else if((reading_irL==0)&&(reading_irML==0)&&(reading_irM==1)&&(reading_irMR==0)&&(reading_irR==0)){
      //middle is detected go straight
      fowardLow();
    }else if((reading_irL==0)&&(reading_irML==1)&&(reading_irM==1)&&(reading_irMR==0)&&(reading_irR==0)){
      //shifted right. so turn left.
      turnLeft();
    }else if((reading_irL==0)&&(reading_irML==0)&&(reading_irM==1)&&(reading_irMR==1)&&(reading_irR==0)){
      //shifted left. so turn right.
      turnRight();
    }else if((reading_irL==0)&&(reading_irML==1)&&(reading_irM==1)&&(reading_irMR==1)&&(reading_irR==0)){
      if((reading_irFL==1)&&(reading_irF==1)&&(reading_irFR==1)||(reading_irFL==1)&&(reading_irF==1)&&(reading_irFR==0)||(reading_irFL==0)&&(reading_irF==1)&&(reading_irFR==1)||(reading_irFL==0)&&(reading_irF==0)&&(reading_irFR==0)){
        //match. move foward 111 110 011 000
        //while exiting circle this works since its 111 senarion to go straight
        fowardLow();
      }else if((reading_irFL==1)&&(reading_irF==0)&&(reading_irFR==0)){
        //left match. move left 100
        turnLeft90();
      }
      else if((reading_irFL==0)&&(reading_irF==0)&&(reading_irFR==1)){
        //right match. move right 001
        turnRight90();
      }else{
        fowardLow();
      }
    }else{
      turnLeft();
    }    
  }
}

void calibrate(){
  //calibrate the device according to the surface
  vAngle=80;
  vOrigin=120;
  
  RFoward(servoRFT,servoRFB,50,hAngle,vOrigin,vAngle,motionDelay);//90h  
  LFoward(servoLFT,servoLFB,140,hAngle,vOriginL,vAngle,motionDelay);//100h
  RFoward(servoRBT,servoRBB,145,hAngle,vOrigin,vAngle,motionDelay);//95h
  LFoward(servoLBT,servoLBB,55,hAngle,vOriginL,vAngle,motionDelay);//95h
  delay(500);

  getIRReading();
  delay(500);
  
  resetParameters(); 
}

void resetParameters(){
  //reset a arm to the starting position
  vAngle = 50;//40
  hAngle = 60;//60
  vOrigin=110;//90
  motionDelay=200;//200
}

void fowardHigh_stepCount(int steps){
  //go foward in high position for a given number of steps
  delay(1000);
  v90();
  delay(1000);    
  int i=0;   
  while(i<steps){
    fowardHigh();
    i++;
  } 
}

void fowardLow_stepCount(int steps){
  //go foward in low position for a given number of steps
  int i=0;   
    while(i<steps){
      fowardLow();
      i++;
    } 
}

void turnLeft90(){
  //turn left by 90 degrees once
  int i=0;
  while(i<8){
    turnLeft();
  }
}

void turnRight90(){
  //turn right by 90 degrees once
  int i=0;
  while(i<8){
    turnRight();
  }
}

void turnLeft(){
  //turn left by 15 degrees once
  vOrigin=90;
  vAngle=40;
  hAngle=40;
  
  delay(500);
  reset();
  delay(500);
  RFoward(servoRFT,servoRFB,70,hAngle,vOrigin,vAngle,motionDelay);//90h
  LBackward(servoLFT,servoLFB,140,hAngle,vOriginL,vAngle,motionDelay);//100h  
  LBackward(servoLBT,servoLBB,75,hAngle,vOriginL,vAngle,motionDelay);//95h  
  RFoward(servoRBT,servoRBB,145,hAngle,vOrigin,vAngle,motionDelay);//95h 
  
  resetParameters();
}

void turnRight(){
  //turn right by 15 degrees once
  vOrigin=90;
  vAngle=40;
  hAngle=40;
  
  delay(500);
  reset();
  delay(500);
  RBackward(servoRBT,servoRBB,145,hAngle,vOrigin,vAngle,motionDelay);//95h
  LFoward(servoLBT,servoLBB,75,hAngle,vOriginL,vAngle,motionDelay);//95h
  LFoward(servoLFT,servoLFB,140,hAngle,vOriginL,vAngle,motionDelay);//100h
  RBackward(servoRFT,servoRFB,70,hAngle,vOrigin,vAngle,motionDelay);//90h  
  
  resetParameters();
}

void reset(){   
  turnServo(servoRFT,50);  
  turnServo(servoRBT,145);  
  turnServo(servoLFT,140); 
  turnServo(servoLBT,55);   
}

void fowardHigh(){
  //move foward while keeping the body heigh from the ground.  
  vOrigin=90;
  vAngle=15;
  hAngle=80;

  reset();
  delay(500);
  
  RFoward(servoRFT,servoRFB,50,hAngle,vOrigin,vAngle,motionDelay);//90h  
  LFoward(servoLFT,servoLFB,140,hAngle,vOriginL+20,vAngle,motionDelay);//100h 
  RFoward(servoRBT,servoRBB,145,hAngle,vOrigin,vAngle,motionDelay);//95h
  LFoward(servoLBT,servoLBB,55,hAngle,vOriginL+20,vAngle,motionDelay);//95h

  resetParameters();
}

void fowardLow(){
  //move foward while keeping the body low from the ground.  
  reset();
  delay(500);
  
  RFoward(servoRFT,servoRFB,50,hAngle,vOrigin,vAngle,motionDelay);//90h  
  LFoward(servoLFT,servoLFB,140,hAngle,vOriginL,vAngle,motionDelay);//100h
  RFoward(servoRBT,servoRBB,145,hAngle,vOrigin,vAngle,motionDelay);//95h
  LFoward(servoLBT,servoLBB,55,hAngle,vOriginL,vAngle,motionDelay);//95h   
}

void LFoward(Servo servoTop,Servo servoBottom, int servoTop_HAngle,int hDeviation,int servoBottom_VAngle,int vDeviation,int delayTime){
    //move a left leg foward once
    servoBottom.write(servoBottom_VAngle-vDeviation);//move bottom leg up
    delay(delayTime);
    servoTop.write(servoTop_HAngle+hDeviation);//rotate top leg foward          
    delay(delayTime);
    servoBottom.write(servoBottom_VAngle);//move bottom leg down
    delay(delayTime);
}

void RFoward(Servo servoTop,Servo servoBottom, int servoTop_HAngle,int hDeviation,int servoBottom_VAngle,int vDeviation,int delayTime){
    //move a right leg foward once
    servoBottom.write(servoBottom_VAngle+vDeviation);//move bottom leg up
    delay(delayTime);
    servoTop.write(servoTop_HAngle-hDeviation);//rotate top leg foward          
    delay(delayTime);
    servoBottom.write(servoBottom_VAngle);//move bottom leg down
    delay(delayTime);
}

void LBackward(Servo servoTop,Servo servoBottom, int servoTop_HAngle,int hDeviation,int servoBottom_VAngle,int vDeviation,int delayTime){
    //move a left leg backward once
    servoBottom.write(servoBottom_VAngle-vDeviation);//move bottom leg up
    delay(delayTime);
    servoTop.write(servoTop_HAngle-hDeviation);//rotate top leg backward          
    delay(delayTime);
    servoBottom.write(servoBottom_VAngle);//move bottom leg down
    delay(delayTime);
}

void RBackward(Servo servoTop,Servo servoBottom, int servoTop_HAngle,int hDeviation,int servoBottom_VAngle,int vDeviation,int delayTime){
  //move a right leg backward once
  servoBottom.write(servoBottom_VAngle+vDeviation);//move bottom leg up
  delay(delayTime);
  servoTop.write(servoTop_HAngle+hDeviation);//rotate top leg backward          
  delay(delayTime);
  servoBottom.write(servoBottom_VAngle);//move bottom leg down
  delay(delayTime);
}

int distance(int trig,int echo){
  //obtain trig,echo pins of the ultrasonic sensor as input parameters and return the measured distance in cm
  long microseconds, cm;

  pinMode(trig, OUTPUT);  
  pinMode(echo, INPUT);
   
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  pinMode(echo, INPUT);
  microseconds = pulseIn(echo, HIGH);  

  cm = (microseconds/29)/2; 
  return cm;
}

void getIRReading(){
  //obtain the readings of the IR sensors
  reading_irFL = digitalRead(irFL);
  reading_irF = digitalRead(irF);
  reading_irFR = digitalRead(irFR);

  reading_irL = digitalRead(irL);
  reading_irML = digitalRead(irML);
  reading_irM = digitalRead(irM);
  reading_irMR = digitalRead(irMR);
  reading_irR = digitalRead(irR);
}

void printIRReading(){
  //print the readings of the IR sensors
  Serial.print(reading_irFL);
  Serial.print(reading_irF);
  Serial.print(reading_irFR);
  
  Serial.print(" ");

  Serial.print(reading_irL);
  Serial.print(reading_irML);
  Serial.print(reading_irM);
  Serial.print(reading_irMR);
  Serial.println(reading_irR);  
}

void h90(){
  //turn all top servo motors perpendicular to the device  
  turnServo(servoRFT,90);  
  turnServo(servoLFT,100); 
  turnServo(servoLBT,95);   
  turnServo(servoRBT,95);
}

void v90(){
  //turn all bottom servo motors perpendicular to the ground 
  turnServo(servoRFB,90);
  turnServo(servoLFB,90);
  turnServo(servoLBB,90);
  turnServo(servoRBB,90);
}
    
void turnServo(Servo servo,int angle){
  servo.write(angle);
}
