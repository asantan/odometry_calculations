#include <SPI.h>
#include <BasicLinearAlgebra.h>

#define CLOCKWISE -1
#define COUNTERCLOCKWISE 1

#define PINA 5  //motor pin
#define PINB 11 //motor pin

#define ir_left 6  //ir pin number
#define ir_right 12 //ir pin number

#define radius    6 //wheel radius in cm
#define baseline  6 //baseline length in cm 
#define pi      3.1415926535
#define two_pi  6.2831850718 

#define tick_per_revolution 2 //number is not considered accurate right now
#define revolutions_per_360 80 
   
const float degrees_per_tick = 360/(tick_per_revolution*revolutions_per_360);
const float radians_per_tick = degrees_per_tick*(pi/180); //change in degree per tick
const float arclength = radius*radians_per_tick;    //Arclength can be viewed as line bc of small angle change
const float delta_phi_per_tick = 1/(tan(arclength/baseline));    //angle between current position and origin


float phi_global_matrix = 0.0; 
float x_global_matrix = 0.0; 
float y_global_matrix = 0.0; 

//BasicLinearAlgebra is wrapped in namespace BLA
//BLA specifies you are using BasicLinearAlgebra Matrix
BLA:: Matrix<4,4> r_rot = {
                      0, 0, 0, 0, 
                      0, 0, 0, baseline/2, 
                      0, 0, 0, 0,
                      0, 0, 0, 1
                    };


BLA:: Matrix<4,4> l_rot = {
                      0, 0, 0, 0, 
                      0, 0, 0, -baseline/2, 
                      0, 0, 0, 0,
                      0, 0, 0, 1
                    };


BLA:: Matrix<4,4> r_trans = {
                      1, 0, 0, 0, 
                      0, 1, 0, -baseline, 
                      0, 0, 1, 0,
                      0, 0, 0, 1
                    };


BLA:: Matrix<4,4> l_trans = {
                      1, 0, 0, 0, 
                      0, 1, 0, baseline, 
                      0, 0, 1, 0,
                      0, 0, 0, 1
                    };

BLA:: Matrix<4, 4> T = {
                    cos(phi_global_matrix), -sin(phi_global_matrix), 0, x_global_matrix,
                    sin(phi_global_matrix), cos(phi_global_matrix), 0, y_global_matrix, 
                    0, 0, 1, 0, 
                    0, 0, 0, 1 
                    
                    
                    };
 

int currentDirection = 0;

float phi_global = 0.0; 
float x_global = 0.0; 
float y_global = 0.0; 

void setup(){
  Serial.begin(9600);
  SPI.begin();
  pinMode(PINA, OUTPUT);
  pinMode(PINB, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ir_left), left_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ir_right), right_tick, RISING);
}

//calculates distance covered by left wheel
void left_tick(){ //tick detected
  float local_phi = -delta_phi_per_tick;
  float delta_x = (baseline/2)*sin(local_phi);         //x and y are change along origin
  float delta_y = (baseline/2) - (baseline/2)*cos(local_phi); //sign of phi is important 
  float delta_x_prime = delta_x*cos(phi_global) + delta_y*sin(phi_global);
  float delta_y_prime = delta_x*sin(phi_global) + delta_y*cos(phi_global);
  matrix_l(); //matrix method
}

//calculates distance covered by right weel
void right_tick(){
  float local_phi = delta_phi_per_tick;
  float delta_x = (baseline/2)*sin(local_phi);         //x and y are change along origin
  float delta_y = (baseline/2) - (baseline/2)*cos(local_phi); //sign of phi is important 
  float delta_x_prime = delta_x*cos(phi_global) + delta_y*sin(phi_global);
  float delta_y_prime = delta_x*sin(phi_global) + delta_y*cos(phi_global);
  rise_r(local_phi, delta_x_prime, delta_y_prime); //analytical method
  matrix_r();//matrix method
}

/* 
 * begin section for analytical method of obtaining x, y, theta_z 
 */

//update global position based off movement of right wheel
void rise_r(float delta_phi, float delta_x_prime, float delta_y_prime){
  phi_global = phi_global + delta_phi; 
  x_global = x_global + delta_x_prime; //x and y are global change vs in reference 
  y_global = y_global + delta_y_prime; //to robot
  printGlobalPosition();
}

//update global position based off movement of left wheel
void rise_l(float phi, float delta_x_prime, float delta_y_prime){
  phi_global = phi_global - phi; 
  x_global = x_global + delta_x_prime; 
  y_global = y_global - delta_y_prime; 
  printGlobalPosition();
}
/* 
 * end section for analytical method of obtaining x, y, theta_z 
 */


/*
 * begin matrix method section for x, y, theta-z
 */
  void matrix_l(){
    T = T*l_rot*l_trans;      
      
  }

  void matrix_r(){
    T = T*r_rot*r_trans; 
  };
 /*  
  * end matrix ethod for x, y, theta_z 
  */
int setSpeed(float s){
  if (s > 1.0) s = 1.0; 
  if (s < 0.0) s = 0.0; 
  int out = (int) (s*255.0);
  return out; 
}

void setVelocity(float velocity){
  float s = velocity; 
  if(s < 0) {
    s = -s;
    currentDirection = CLOCKWISE;  
  }
  else {
    currentDirection = COUNTERCLOCKWISE; 
  }
  int sp = setSpeed(s); 

  if(velocity >= 0){ //forward
    analogWrite(PINB, 0);
    //digitalWrite(PINB, LOW); //important to set low first to avoid shorting
    analogWrite(PINA, sp);
  }
  else { //backward
    analogWrite(PINA, 0);
    //digitalWrite(PINA, LOW);
    analogWrite(PINB, sp);    
  }
}

void printGlobalPosition(){
  Serial.print("Global phi ");
  Serial.println(phi_global);
  Serial.print("X location ");
  Serial.println(x_global);
  Serial.print("Y location");
  Serial.println(y_global);
}

void loop(){
    while(Serial.available()){
      float vel = Serial.parseFloat();
      setVelocity(vel);
    }
}
