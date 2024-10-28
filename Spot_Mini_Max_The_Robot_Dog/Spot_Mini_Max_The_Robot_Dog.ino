/*
Spot Robot controller
Jan De Coster
07/01/2021

More info on www.jandecoster.com
This code is part of an online course. Using this code on your robot requires proper knowledge of the platform.
Do not forget to calibrate your robot to avoid catastrophic failure ;) 

RC read based on 
Kelvin Nelson's example 24/07/2019
https://create.arduino.cc/projecthub/kelvineyeone/read-pwm-decode-rc-receiver-input-and-apply-fail-safe-6b90eb

*/

#include <Servo.h>

//RC stuff
//RC controller pins
const int CH_0_PIN = A0;
const int CH_1_PIN = A1;
const int CH_2_PIN = A2;
const int CH_3_PIN = A3;
const int deadzone = 20;  //center stick when close to center

int ch_0;
int ch_1;
int ch_2;
int ch_3;
//End RC stuff

//Gyro stuff
#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw, oldroll, oldpitch, oldyaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

bool safetymode = false;
//End Gyro stuff



Servo servo[12]; //array of servo's

const int servo_pin[12] =  {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};  //arrays of servo pins

const int directions[12] = {1, -1,-1,  1 ,1,1,  -1 ,1,1, -1,-1,-1};  //reflection correction for opposite sides of the robot

const int base[12] = {90 ,100 ,102 ,  90 ,88 ,97 ,  90.00 ,78 ,95 ,  95 ,108 ,100 }; 
//callibration   these are the base values for each servo, that put Spot in a comfortable starting position



/* Servo's kan not keep their current position, so we store these in an array angleCurrent
 * The target of each servo is placed in angleTarget, and angleSpeed holds each change in angle for each servo, per function call
 * arcPos holds different poses , with values relative to the base values in array base
 */

float angleCurrent[12] = {base[0], base[1], base[2], base[3], base[4], base[5], base[6], base[7], base[8], base[9], base[10], base[11]}; //real-time arcs
float angleTarget[12] = {base[0], base[1], base[2], base[3], base[4], base[5], base[6], base[7], base[8], base[9], base[10], base[11]}; //expected coordinates of the end of the leg
float angleSpeed[12] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00}; //each axis' speed, needs to be recalculated before each movement

//stap sequentie vooruit
int sb1 = -10;  int so1 = 22;    //sb upperarm  so lowerarm
int sb2 = 2;    int so2 = 1;
int sb3 = -9;   int so3 = 1;      //only for 4 step moves
int sb4 = -21;  int so4 = 19;

//intertia compensation after running
int brb = 1;  int bro = -12;

//brace animation when stopping, to prevent tipping over
int brcb = -14;  int brco = 24;

int pootb = 53;  int pooto = -16;





float arcPos[14][12] = {   //2D array met standen voor elke servo, pas initialisatie aan wanneer je stappen toevoegt !
        //Leg Left Front------Leg right Front-----Leg Left rear----Leg Left rear    
        // 0    //base stand
          {0,      0,     0,     0,     0,     0,     0,     0,     0,     0,      0,    0},   
        //shldr   up    down  shldr  up    down     shldr  up    down    shldr  up    down  
           
        // 1    //lay down, safety position when RC is lost
          {0,     0,    45,     0,     0,    45,     0,     0,     45,    0,      0,   45}, 
        //shldr   up    down  shldr  up    down     shldr  up    down    shldr  up    down  
           
        // 2    //stand back , rease back
          {0,     0,    45,     0,     0,    45,     0,      0,     0,    0,      0,     0},
      //shldr   up    down  shldr  up    down     shldr  up    down    shldr  up    down  
          
        // 3    //sit up
          {0,     0,    -15,      0,     0,   -15,     0,      0,    25,    0,      0,   25},
        //shldr   up    down  shldr  up    down     shldr  up    down    shldr  up    down  


        // 4    //step 1   
          {0,    sb1,    so1,     0,   sb2,   so2,   0,    sb1,   so1,    0,    sb2,   so2},
        //shldr   up    down  shldr  up    down     shldr  up    down    shldr  up    down  

        
        // 5    //step 2    
          {0,    sb2,    so2,    0,   sb1,  so1,     0,    sb2,   so2,    0,      sb1,   so1},
        //shldr   up    down  shldr  up    down     shldr  up    down    shldr  up    down  
        
         // 6    //step 3
          {0,    sb3,    so3,    0,   sb1,  so1,    0,    sb3,   so3,    0,    sb1 ,   so1  },
        //shldr   up    down  shldr  up    down     shldr  up    down    shldr  up    down  

         // 7    //step 4
          {0,    sb4,    so4,   0,     sb2,    so2,   0,   sb3,   so3,    0,    sb2,     so2},
        //shldr   up    down  shldr  up    down     shldr  up    down    shldr  up    down  

         // 8    //stand turn
          {0,    0,     0,      0,      0,    0,        0,   0,   0,      0,    0,     0},
        //shldr   up    down  shldr  up    down     shldr  up    down    shldr  up    down  
        
         // 9    //break forward
          {0,    0,     0,      0,      0,    0,        0,   sb1/3,   so1/3,      0,    sb1/3,   so1/3},
        //shldr   up    down  shldr  up    down     shldr  up    down    shldr  up    down  

        // 10    //break backward
          {0,    0,  0,   0,   0,  0,   0,   brb,   bro,      0,    brb,  bro },
        //shldr   up    down  shldr  up    down     shldr  up    down    shldr  up    down  

        // 11    //brace 
          {0,    brcb,  brco,   0,   brcb,  brco,   0,   brcb,   brco,      0,  brcb,  brco},
        //shldr   up    down  shldr  up    down     shldr  up    down    shldr  up    down  
        
          // 12    //sniff 
          {0,    brcb,  brco,   0,   brcb,  brco,   0,   0,   0,      0,  0,  0},
        //shldr   up    down  shldr  up    down     shldr  up    down    shldr  up    down  

         // 13    //raise 
          {0,    0,  0,   0,   0,  0,   0,   brcb,   brco,      0,  brcb,  brco}
        //shldr   up    down  shldr  up    down     shldr  up    down    shldr  up    down  
          };

        
bool zitten = false; //is Spot sitting down?
bool moving = false; //is Spot moving? then we should wait
bool staan = false; //is Spot standing?
bool movingforward = false; //is Spot moving forward?
bool movingbackward = false; //is Spot moving backward?
bool jumped = false; //did Spot jump?
bool sniffing = false; //is Spot sniffing?
bool raised = false; //is Spot looking up?

int draaihoek = 0;  //left right
int tilthoek = 0;   // up down
int fspeed = 0;     //speed

int currentstep = 1;

void setup() {
  Serial.begin(9600); 

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  Serial.println("Gyro");
  calculate_IMU_error();
  delay(20);
  
  Serial.println("relay");
  
  //pinMode(1, OUTPUT); //1 when pin available connect this to a relay for extra safety
  //digitalWrite(1, LOW);
  delay(500);
  Serial.println("attach");
  servo_attach();

  setTargets(1,10); //go to lay down, without filling target arrays
  
  //digitalWrite(1, HIGH); //start motors with relay
  delay(1000);
  Serial.println("set");

  
  setTargets(1,10); //go to lay down
  zitten = true;

  delay(1000);
 
  

}

void loop() {

checkRC();  //read RC

readGyro();

if(!safetymode){

draaihoek = ch_0/40;
fspeed = -ch_1/25;
tilthoek = ch_3/20;
forwardarray();
turnarray();

  //walking? switch step 1 and 2
  if(abs(ch_1)>40 && !zitten && currentstep==1){
      if(ch_1>0){
        movingforward = true;
        }else{
          movingbackward = true;
          }
    currentstep = 2;
    setTargets(4,10);
    } 
  else if(abs(ch_1)>40 && !zitten && currentstep==2){
     if(ch_1>0){
        movingforward = true;
        }else{
           movingbackward = true;
          }
    currentstep = 1;
    setTargets(5,10);
    }
  //not walking ?  
  else{
  
        if(ch_2>0){                 //gaat channel 2 boven 0?
          if(zitten){               //Spot sits? stand
            anim_lig();
            anim_standup();
            zitten = false;
            }
            else{                     //Spot not sitting? go to stand
              
              zitten = false;
                 
              if(abs(draaihoek)>2 && !zitten){
                
                
                setTargets(8,10);
                }else{
                  if(movingforward){
                    setTargets(9,5);
                    movingforward = false;
                    }
                  else if(movingbackward){
                    setTargets(10,5);
                    movingbackward = false;
                    }
                  anim_stand();
                  }
                  
              } 
          }
        
        if(ch_2<-100 && !zitten){     //gaat channel 2 onder -100?
          zitten = true;
          anim_zit();                 // Spot sit
          }
          
        else if(ch_2<-200 && zitten){      //gaat channel 2 onder -200?
          anim_lig();                 //Spot lay down
          }

        else if(ch_2>220 && !jumped){      //gaat channel 2 boven 200?
          anim_jump();                 //Spot jump
          jumped = true;
          } 
        else if(ch_2<200){
          jumped = false;
          }  
        if(ch_3>220 && !sniffing && !zitten){
         anim_sniff();    
         sniffing = true;
          }   
        else if(ch_3<200){
         sniffing = false;
          }
        if(ch_3<-200 && !raised && !zitten){
         anim_raise();    
         raised = true;
          }   
        else if(ch_3>-180){
         raised = false;
          }      
          
        
    }
}

else if(ch_2<-200){  //leave safety by going to sitting
  zitten= true;
  safetymode = false;
  
  }
}

void forwardarray(){ //adjust angles accoridng to input
int comp = -draaihoek*0.9;
  
if(fspeed>0){  //
sb1 = -10-abs(fspeed/1.5);  so1 = 22-abs(fspeed/1.5);
sb2 = 2-abs(fspeed/1.5);    so2 = 1-abs(fspeed/1.5);
}else{      //achteruit niet te snel
sb1 = -10-abs(fspeed/3);  so1 = 22-abs(fspeed/3);
sb2 = 2-abs(fspeed/3);    so2 = 1-abs(fspeed/3);
  }

  arcPos[4][1] = sb1-draaihoek;         arcPos[4][2] = so1-draaihoek-comp;                  arcPos[4][4] = sb2+fspeed+draaihoek;  arcPos[4][5] = so2+draaihoek+comp+fspeed/2;   arcPos[4][7] = sb1+draaihoek;          arcPos[4][8] = so1+draaihoek+comp;             arcPos[4][10] = sb2+fspeed-draaihoek; arcPos[4][11] = so2+fspeed/2-draaihoek+comp; 
  arcPos[5][1] = sb2+fspeed-draaihoek;  arcPos[5][2] = so2+fspeed/2-draaihoek-comp;         arcPos[5][4] = sb1+draaihoek;         arcPos[5][5] = so1+draaihoek+comp;            arcPos[5][7] = sb2+fspeed+draaihoek;   arcPos[5][8] = so2+fspeed/2+draaihoek+comp;    arcPos[5][10] = sb1-draaihoek;        arcPos[5][11] = so1-draaihoek+comp; 
}

void turnarray(){  //adjust angles accoridng to input

  int comp = -draaihoek*0.9;

 
  arcPos[4][0] = 0;      arcPos[4][3] = -draaihoek;     arcPos[4][6] = 0;           arcPos[4][9] = draaihoek; 
  arcPos[5][0] = -draaihoek;      arcPos[5][3] = 0;     arcPos[5][6] = draaihoek;   arcPos[5][9] = 0; 
  
  arcPos[8][0] = -draaihoek; 
  arcPos[8][1] = -draaihoek;      
  arcPos[8][2] = -draaihoek-comp;       
  arcPos[8][3] = -draaihoek;
  arcPos[8][4] = draaihoek;       
  arcPos[8][5] = draaihoek+comp;         
  arcPos[8][6] = draaihoek;
  arcPos[8][7] = draaihoek;  
  arcPos[8][8] = draaihoek+comp;          
  arcPos[8][9] = draaihoek;
  arcPos[8][10] = -draaihoek; 
  arcPos[8][11] = -draaihoek-comp;  
  }


void anim_standup(){  //sit up, back first, then front legs
   setTargets(2,5);
   setTargets(0,8);
  }

void anim_lig(){   //lay down
  setTargets(1,8);
  }
  
void anim_zit(){ //sit
  setTargets(3,2);
  }

void anim_stand(){ //go to stand not from sitting
   setTargets(0,2);
  }

void anim_trappel(){ //go to stand not from sitting
   setTargets(4,15);
   setTargets(5,15);
  }

void anim_jump(){
  setTargets(0,2);
  setTargets(11,4);
  setTargets(0,20);
  }

void anim_sniff(){
  setTargets(12,2);
  }  
  
void anim_raise(){
  setTargets(13,2);
  }  

        
void setTargets(int pos, float myspeed){    // pos = row in array    myspeed = speed from 1 to 20

moving = true;
float delta[12];
float maxDelta = 0;

if(myspeed>20){ myspeed = 20;}
if(myspeed<1){ myspeed = 1;}

for (int i=0; i < 12; i++){
 
  angleTarget[i] = base[i]+arcPos[pos][i]*directions[i];
  delta[i] = base[i]+arcPos[pos][i]*directions[i] - angleCurrent[i]; 
  
  if(abs(delta[i])>maxDelta){
    maxDelta = abs(delta[i]);
    }
  
  if(delta[i]>0){
    angleSpeed[i] = myspeed;
    }  
  else if(delta[i]<0){
    angleSpeed[i] = -myspeed;
    } 
  else{
      angleSpeed[i] = 0;
      }  
 
  } 
 
  while(moving){  //repeat function while servo's not at destination
    setServos();
    delay(5); //give it some time
    }
 
}

void setServos(){
  
  int speedcount = 0; //count all servo's that are still moving

  //for each servo, what is the distance between current angle and target angle
  //smaller then 1? make current angle target angle en put speed to 0
  //if not, add speed to current angle with factor 1/10 adjusted with speed

  
  for (int i=0; i < 12; i++){
    if(abs(angleTarget[i]-angleCurrent[i])<=1){
        angleCurrent[i] = angleTarget[i];
        angleSpeed[i] = 0;
        }else{
         angleCurrent[i] += angleSpeed[i]/10; 
          }
  
    servo[i].write(constrain(angleCurrent[i],10,170));  //angles are put in the array, now put the servo at right position
  }

  //are servo's still moving?
  for (int i=0; i < 12; i++){
    speedcount += abs(angleSpeed[i]);
    }
  //if not  (speedcount = 0) stop moving, while loop stops
  if(speedcount==0){
    moving = false;
    }  
    
}


void servo_attach(void){  
  for (int i = 0; i < 12; i++)
  {
    servo[i].attach(servo_pin[i]);
    
    delay(100);
  }
}


void checkRC(){ //convert RC pulses to variables
   // Read pulse width from receiver
  ch_0 = pulseIn(CH_0_PIN, HIGH, 25000);
  ch_1 = pulseIn(CH_1_PIN, HIGH, 25000);
  ch_2 = pulseIn(CH_2_PIN, HIGH, 25000);
  ch_3 = pulseIn(CH_3_PIN, HIGH, 25000);

  ch_0 = pulseToPWM(ch_0);
  ch_1 = pulseToPWM(ch_1);
  ch_2 = pulseToPWM(ch_2);
  ch_3 = pulseToPWM(ch_3);
  

  }


// Convert RC pulse value to motor PWM value
int pulseToPWM(int pulse) {
  
  // pulses between 1000 and 2000 converted to -255 and 255
  
  if ( pulse > 1000 ) {
    pulse = map(pulse, 1000, 2000, -500, 500);
    pulse = constrain(pulse, -255, 255);
  } else {
    pulse = 0;
  }

  // Anything in deadzone should stop the motor
  if ( abs(pulse) <= deadzone ) {
    pulse = 0;
  }

  return pulse;
}



void readAcc(){
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  
  }
  
void readGyro(){
// === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
//  // Print the values on the serial monitor
//  Serial.print(roll);
//  Serial.print("/");
//  Serial.print(pitch);
//  Serial.print("/");
//  Serial.println(yaw);

    if(abs(GyroX)>200){
      Serial.println(GyroX);
      safetymode = true;
      setTargets(1,5);
      
      }
  if(abs(GyroY)>200){
      Serial.println(GyroY);
      safetymode = true;
      setTargets(1,5);
      }
    if(abs(GyroZ)>200){
      Serial.println(GyroZ);
      safetymode = true;
      setTargets(1,5);
      }  
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}