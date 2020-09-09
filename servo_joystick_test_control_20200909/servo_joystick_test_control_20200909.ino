

#include <Servo.h>
#include <math.h>

#define NUM_SERVOS 3

Servo servo[NUM_SERVOS];

struct angles {
  double tetta;
  double alpha;
  double gamma;
};

struct coordinates {
  double x4;
  double y4;
  double z4;
};

// передняя левая нога
uint8_t front_left_coxa_init_pos = 90;
uint8_t front_left_femur_init_pos = 140;
uint8_t front_left_tibia_init_pos = 180;

// текущие положения сервоприводов
uint8_t coxa_curr_pos;
uint8_t femur_curr_pos;
uint8_t tibia_curr_pos;

float r = 15.0; //mm
float h = 45.0 + 55.0; //h + h_coxa (mm)
float l_coxa = 55.0; //mm
float l_femur = 70.0; //mm
float l_tibia = 70.0; //mm
float sn_f = 60.0; // поправка в 60 градусов

const int DELAY_INTERVAL = 15; // ms

const bool STARTING_POSITION = 0;
const bool WORKING_POSITION = 1;

float x_point, y_point, z_point, l, l1, alpha, alpha_1, alpha_2, beta, gamma;

void initial_pose(String limb){
  if(limb == "coxa"){
    coxa_pin.write(coxa_init_pos);
    coxa_curr_pos = coxa_init_pos;
  }
  else if(limb == "femur"){
    femur_pin.write(femur_init_pos);
    femur_curr_pos = femur_init_pos;
  }
  else if(limb == "tibia"){
    tibia_pin.write(tibia_init_pos);
    tibia_curr_pos = tibia_init_pos;
  }
}

uint16_t angle_to_microsec(uint16_t angle){
  return (uint16_t)map(angle, 0, 180, USMIN, USMAX);
}

void angles_control(float x, float y, float z){
  l1 = sqrt(sq(y) + sq(z + l_coxa));
  l = sqrt(sq(x) + sq(l1 - l_coxa));
  alpha_1 = degrees(acos(x / l));
  alpha_2 = degrees(acos( (sq(l)+sq(l_femur)-sq(l_tibia))/(2.0*l*l_femur) ));
  alpha = alpha_1 + alpha_2;
  beta = degrees(acos( (sq(l_femur)+sq(l_tibia)-sq(l))/(2.0*l_femur*l_tibia) ));
  gamma = degrees(atan(y / z));
  
  alpha = constrain(alpha, 0.0, 180.0);
  beta = constrain(beta, 0.0, 180.0);
  gamma = constrain(gamma, 0.0, 180.0);

  Serial.println("");
  Serial.println("*************************");
  Serial.println("X: " + String(x) + " " + "Y: " + String(y) + " " + "Z: " + String(z));
  Serial.println("Alpha: " + String(alpha));
  Serial.println("Beta: " + String(beta));
  Serial.println("Gamma: " + String(gamma));
  Serial.println("*************************");
  Serial.println("");
  
  if(reverse == false){
    femur_curr_pos = (uint8_t)round( 180.0 - alpha );
    tibia_curr_pos = (uint8_t)round( beta - 45.0 );
  }
  else{
    femur_curr_pos = (uint8_t)round( alpha );
    tibia_curr_pos = (uint8_t)round( 180.0 - (beta - 45.0) );
  }
}

void movement(String limb){
  if(limb == "coxa"){         
    coxa_pin.writeMicroseconds(angle_to_microsec(coxa_curr_pos));
  }
  else if(limb == "femur"){
    femur_pin.writeMicroseconds(angle_to_microsec(femur_curr_pos));
  }
  else if(limb == "tibia"){
    tibia_pin.writeMicroseconds(angle_to_microsec(tibia_curr_pos));
  }
}

void initial_pose(){
  front_left.initial_pose("tibia");
  front_right.initial_pose("tibia");
  back_left.initial_pose("tibia");
  back_right.initial_pose("tibia");

  delay(500);

  front_left.initial_pose("femur");
  front_right.initial_pose("femur");
  back_left.initial_pose("femur");
  back_right.initial_pose("femur");

  delay(500);

  front_left.initial_pose("coxa");
  front_right.initial_pose("coxa");
  back_left.initial_pose("coxa");
  back_right.initial_pose("coxa");
  
  state = STARTING_POSITION;
}

void setup() {
  Serial.begin(9600);
  pinMode(19, INPUT_PULLUP);
  for(int i=0; i<NUM_SERVOS; i++){
    servo[i].attach(2+i, 500, 2500);
  }
  delay(15);
  initial_pose();
}

void loop() {
  button_handler();

  if(state == WORKING_POSITION){
    joystick_handler();
  }
}

void joystick_handler(){

  int vrx = analogRead(A0);
  
  if(vrx < 100){
    walking_forward();
  }
}
