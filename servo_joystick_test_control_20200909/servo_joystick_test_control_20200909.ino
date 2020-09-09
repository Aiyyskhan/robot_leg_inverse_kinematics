/*
 * 
 * Inverse kinematics algorithm
 * 
 * Example of leg control using a joystick
 * 
 * create in 09.09.2020
 * by Aiyyskhan Alexeev
 * 
 * 
 */

#include <Servo.h>
#include <math.h>

// количество сервоприводов
#define NUM_SERVOS 3

// минимальное и максимальное значения импульсов сервопривода
#define USMIN 500
#define USMAX 2500

// крайние значения возможных координат кончика ноги
#define XMIN -100.0
#define XMAX 100.0
#define YMIN -75.0
#define YMAX 75.0
#define ZMIN 120.0
#define ZMAX 180.0

Servo servo[NUM_SERVOS];

struct Angles {
  float tibia;
  float femur;
  float coxa;
};

struct Coordinates {
  float x;
  float y;
  float z;
};

// начальные и текущие углы ноги (передней правой)
Angles init_angles_FR;
Angles curr_angles_FR;

// начальные и текущие координаты кончика ноги (передней правой)
Coordinates init_coord_FR;
Coordinates curr_coord_FR;

// координаты джойстика
Coordinates joy_coord;

// длины конечностей ноги
float l_coxa = 55.0; //mm
float l_femur = 70.0; //mm
float l_tibia = 70.0; //mm

// поправочный угол конечности "femur"
float sn_f = 60.0; // поправка в 60 градусов

// интервал основного цикла
const int DELAY_INTERVAL = 5; // ms

// режимы
const bool STARTING_POSITION = 0;
const bool WORKING_POSITION = 1;
bool state;

int currentValue, prevValue;

void initial_pose(String limb){
  // функция установки конечностей в начальное положение
  if(limb == "coxa"){
    servo[0].write(init_angles_FR.tibia);
    curr_angles_FR.tibia = init_angles_FR.tibia;
  }
  else if(limb == "femur"){
    servo[1].write(init_angles_FR.femur);
    curr_angles_FR.femur = init_angles_FR.femur;
  }
  else if(limb == "tibia"){
    servo[2].write(init_angles_FR.coxa);
    curr_angles_FR.coxa = init_angles_FR.coxa;
  }
}

float angle_to_microsec(double ang){
  // функция конвертирующая углы (градусы) в импульсы (микросекунды) 
  return (float)map(ang, 0.0, 180.0, USMIN, USMAX);
}

Coordinates joystick_sig_convert(Coordinates sig){
  // функция конвертирующая сигналы джойстика в координаты кончика ноги

  Serial.println("");
  Serial.println("*************************");
  Serial.println("pot X: " + String(sig.x) + " " + "pot Y: " + String(sig.y) + " " + "pot Z: " + String(sig.z));
  Serial.println("*************************");
  Serial.println("");
  
  Coordinates coord;
  coord.x = (float)map(sig.x, 0, 1023, XMIN, XMAX);
  coord.y = (float)map(sig.y, 0, 1023, YMIN, YMAX);
  coord.z = (float)map(sig.z, 0, 1023, ZMIN, ZMAX);
  return coord;
}

Angles angles_control(Coordinates coord, bool reverse){
  // функция обратной кинематики
  // принимает координаты, на основе которых вычисляются углы сервоприводов конечностей
  // аргумент reverse необходим для инвертирования углов (необходим для передней левой и задней правой ног)
  Angles ang;
  float l, l1, alpha, alpha_1, alpha_2, beta, gamma;
  
  l1 = sqrt(sq(coord.y) + sq(coord.z));
  l = sqrt(sq(coord.x) + sq(l1 - l_coxa));
  alpha_1 = acos(coord.x / l);
  alpha_2 = acos( (sq(l)+sq(l_femur)-sq(l_tibia))/(2.0*l*l_femur) );
  alpha = degrees(alpha_1 + alpha_2);
  beta = degrees(acos( (sq(l_femur)+sq(l_tibia)-sq(l))/(2.0*l_femur*l_tibia) ));
  gamma = degrees(atan(coord.y / coord.z));

  Serial.println("");
  Serial.println("*************************");
  Serial.println("X: " + String(coord.x) + " " + "Y: " + String(coord.y) + " " + "Z: " + String(coord.z));
  Serial.println("Alpha: " + String(alpha));
  Serial.println("Beta: " + String(beta));
  Serial.println("Gamma: " + String(gamma));
  Serial.println("*************************");
  Serial.println("");
  
  if(reverse){
    ang.tibia = 180.0 - (beta - 45.0);
    ang.femur = alpha;
    ang.coxa = 90.0 + gamma;
  }
  else{
    ang.tibia = beta - 45.0;
    ang.femur = 180.0 - alpha;
    ang.coxa = 90.0 + gamma;
  }

  return ang;
}

void movement(){
  // функция приводящая в движение сервоприводы согласно текущим углам
  servo[0].writeMicroseconds(angle_to_microsec(curr_angles_FR.tibia));
  servo[1].writeMicroseconds(angle_to_microsec(curr_angles_FR.femur));
  servo[2].writeMicroseconds(angle_to_microsec(curr_angles_FR.coxa));
}

void setup() {
  Serial.begin(9600);
  analogReadResolution(10);

  // инициализация пина кнопки джойстика с подтягивающим резистором
  pinMode(19, INPUT_PULLUP);

  // инициализация сервоприводов конечностей
  for(int i=0; i<NUM_SERVOS; i++){
    servo[i].attach(8+i, 500, 2500);
  }
  
  delay(15);
  
  // передняя левая нога
  init_angles_FR.tibia = 0;
  init_angles_FR.femur = 40;
  init_angles_FR.coxa = 90;

  // установка конечностей в начальное положение
  initial_pose("tibia");
  delay(500);
  initial_pose("femur");
  delay(500);
  initial_pose("coxa");

  // переключение режима в STARTING_POSITION
  state = STARTING_POSITION;
}

void loop() {
  joystick_handler();
  delay(DELAY_INTERVAL);
}

void joystick_handler(){
  currentValue = digitalRead(19);
  if(currentValue != prevValue){
    delay(15);
    currentValue = digitalRead(19);
  }
  prevValue = currentValue;

  // переключение режима
  if(currentValue == 0){
    if(state == STARTING_POSITION){
      state = WORKING_POSITION;
    }
    else{
      state = STARTING_POSITION;
    }
  }

  // управление джойстиком
  if(state == WORKING_POSITION){
  
    joy_coord.x = analogRead(A0);
    joy_coord.y = analogRead(A1);
    joy_coord.z = 548.0;
    
    curr_coord_FR = joystick_sig_convert(joy_coord);
    curr_angles_FR = angles_control(curr_coord_FR, false);
    movement();
  }
}
