/*
 * 
 * Inverse kinematics algorithm
 * 
 * Simple test sketch 
 * 
 * created in 02.12.2020
 * by Aiyyskhan Alexeev
 * 
 * 
 */

#include <Servo.h>
#include <math.h>

// количество сервоприводов
#define NUM_SERVOS 3

// минимальное и максимальное значения импульсов сервопривода
#define USMIN 500.0
#define USMAX 2500.0

// крайние значения возможных координат кончика ноги
#define XMIN -100.0
#define XMAX 100.0
#define YMIN -50.0
#define YMAX 50.0
#define ZMIN 130.0
#define ZMAX 200.0

// длины суставов ноги
float l_coxa = 60.0; //mm
float l_femur = 70.0; //mm
float l_tibia = 100.0; //mm

// интервал основного цикла
const long interval = 300; // milliseconds
float lateralGain = 15;
double increment = 0.0001;
unsigned long previousMillis = 0;

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

// текущие углы ноги (передней правой)
Angles angles_FR;

// начальные и текущие координаты кончика ноги (передней правой)
Coordinates init_coord_FR;
Coordinates step_coord_FR;
Coordinates coord_FR;

void initial_pose(){
  // функция установки конечностей в начальное положение
  angles_FR = angles_control(init_coord_FR, false);
  movement();
}

float angle_to_microsec(float ang){
  // функция конвертирующая углы (градусы) в импульсы (микросекунды) 
  return (float)map(ang, 0.0, 180.0, USMIN, USMAX);
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

//  Serial.println("");
//  Serial.println("*************************");
//  Serial.println("X: " + String(coord.x) + " " + "Y: " + String(coord.y) + " " + "Z: " + String(coord.z));
//  Serial.println("Alpha: " + String(alpha));
//  Serial.println("Beta: " + String(beta));
//  Serial.println("Gamma: " + String(gamma));
//  Serial.println("*************************");
//  Serial.println("");
  
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
  servo[0].writeMicroseconds(angle_to_microsec(angles_FR.tibia));
  servo[1].writeMicroseconds(angle_to_microsec(angles_FR.femur));
  servo[2].writeMicroseconds(angle_to_microsec(angles_FR.coxa));
}

void setup() {
  Serial.begin(9600);

  // инициализация сервоприводов конечностей
  for(int i=0; i<NUM_SERVOS; i++){
    servo[i].attach(11+i, 500, 2500);
  }
  
  delay(15);
  
  // начальные углы передней правой ноги
  init_coord_FR.x = 0;
  init_coord_FR.y = 0;
  init_coord_FR.z = 150;
  
  // установка конечностей в начальное положение
  initial_pose();
  delay(1000);
}


void loop() {
  // проверка по оси X
  delay(2000);
  coord_FR.x = XMIN;
  coord_FR.y = 0;
  coord_FR.z = 150;      

  angles_FR = angles_control(coord_FR, false);
  movement();
  
  delay(2000);
  coord_FR.x = XMAX;
  coord_FR.y = 0;
  coord_FR.z = 150;      

  angles_FR = angles_control(coord_FR, false);
  movement();

  /*
  // проверка по оси Y
  delay(2000);
  coord_FR.x = 0;
  coord_FR.y = YMIN;
  coord_FR.z = 150;      

  angles_FR = angles_control(coord_FR, false);
  movement();
  
  delay(2000);
  coord_FR.x = 0;
  coord_FR.y = YMAX;
  coord_FR.z = 150;      

  angles_FR = angles_control(coord_FR, false);
  movement();
  
  // проверка по оси Z
  delay(2000);
  coord_FR.x = 0;
  coord_FR.y = 0;
  coord_FR.z = ZMIN;      

  angles_FR = angles_control(coord_FR, false);
  movement();
  
  delay(2000);
  coord_FR.x = 0;
  coord_FR.y = 0;
  coord_FR.z = ZMAX;      

  angles_FR = angles_control(coord_FR, false);
  movement();
  */
}
