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
#define YMIN -75.0
#define YMAX 75.0
#define ZMIN 100.0
#define ZMAX 180.0

// длины суставов ноги
float l_coxa = 55.0; //mm
float l_femur = 70.0; //mm
float l_tibia = 70.0; //mm

// интервал основного цикла
const long interval = 1; // milliseconds
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

// начальные и текущие углы ноги (передней правой)
Angles init_angles_FR;
Angles curr_angles_FR;

// начальные и текущие координаты кончика ноги (передней правой)
Coordinates init_coord_FR;
Coordinates curr_coord_FR;

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

float angle_to_microsec(float ang){
  // функция конвертирующая углы (градусы) в импульсы (микросекунды) 
  return (float)map(ang, 0.0, 180.0, USMIN, USMAX);
}

//Angles angles_control(Coordinates coord, bool reverse){
//  // функция обратной кинематики
//  // принимает координаты, на основе которых вычисляются углы сервоприводов конечностей
//  // аргумент reverse необходим для инвертирования углов (необходим для передней левой и задней правой ног)
//  Angles ang;
//  float l, l1, alpha, alpha_1, alpha_2, beta, gamma;
//  
//  l1 = sqrt(sq(coord.y) + sq(coord.z));
//  l = sqrt(sq(coord.x) + sq(l1 - l_coxa));
//  alpha_1 = acos(coord.x / l);
//  alpha_2 = acos( (sq(l)+sq(l_femur)-sq(l_tibia))/(2.0*l*l_femur) );
//  alpha = degrees(alpha_1 + alpha_2);
//  beta = degrees(acos( (sq(l_femur)+sq(l_tibia)-sq(l))/(2.0*l_femur*l_tibia) ));
//  gamma = degrees(atan(coord.y / coord.z));
//
////  Serial.println("");
////  Serial.println("*************************");
////  Serial.println("X: " + String(coord.x) + " " + "Y: " + String(coord.y) + " " + "Z: " + String(coord.z));
////  Serial.println("Alpha: " + String(alpha));
////  Serial.println("Beta: " + String(beta));
////  Serial.println("Gamma: " + String(gamma));
////  Serial.println("*************************");
////  Serial.println("");
//  
//  if(reverse){
//    ang.tibia = 180.0 - (beta - 45.0);
//    ang.femur = alpha;
//    ang.coxa = 90.0 + gamma;
//  }
//  else{
//    ang.tibia = beta - 45.0;
//    ang.femur = 180.0 - alpha;
//    ang.coxa = 90.0 + gamma;
//  }
//
//  return ang;
//}

Angles angles_control(Coordinates coord, bool reverse){
  // функция обратной кинематики
  // принимает координаты, на основе которых вычисляются углы сервоприводов конечностей
  // аргумент reverse необходим для инвертирования углов (необходим для передней левой и задней правой ног)
  Angles ang;
  float alpha, alpha_1, alpha_2, beta, gamma;

  float femur = 70.0;
  float tibia = 11.0;

  float l1 = coord.x; //50.0 + x4;
  float l2 = coord.y; //-60.0 + y4;
  float l3 = coord.z; //0.0;

  float l = sqrt((l1 * l1) + (l2 * l2) + (l3 * l3));
  
  alpha_1 = asin(l1 / l);
  alpha_2 = acos(((l * l) + (femur * femur) - (tibia * tibia)) / (2.0 * l * femur));
  alpha = degrees(alpha_1 + alpha_2);
  beta = degrees(acos(((femur * femur) + (tibia * tibia) - (l * l)) / (2.0 * femur * tibia)));
  gamma = degrees(atan(l3 / l2));

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
  servo[2].writeMicroseconds(angle_to_microsec(curr_angles_FR.tibia));
  servo[1].writeMicroseconds(angle_to_microsec(curr_angles_FR.femur));
  servo[0].writeMicroseconds(angle_to_microsec(curr_angles_FR.coxa));
}

void setup() {
  Serial.begin(9600);

  // инициализация сервоприводов конечностей
  for(int i=0; i<NUM_SERVOS; i++){
    servo[i].attach(11+i, 500, 2500);
  }
  
  delay(15);
  
  // начальные углы передней левой ноги
  init_angles_FR.tibia = 0;
  init_angles_FR.femur = 40;
  init_angles_FR.coxa = 90;

  // установка конечностей в начальное положение
  initial_pose("tibia");
  delay(500);
  initial_pose("femur");
  delay(500);
  initial_pose("coxa");
  delay(1000);
}


void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // stepper();
    // back_and_forth();
    circulation();
  }
}


// #### пример линейного движения ####
int delta_x = 0;
bool forward = true;

void back_and_forth(){
  curr_coord_FR.y = 0.0;
  curr_coord_FR.z = 150.0; // среднее между ZMIN и ZMAX
  curr_coord_FR.x = (float)delta_x;
  curr_angles_FR = angles_control(curr_coord_FR, false);
  movement();

  if(forward){
    delta_x += 2;
  }
  else{
    delta_x -= 2;
  }
  if(delta_x >= XMAX){
    forward = false;
  }
  else if(delta_x <= XMIN){
    forward = true;
  }
}

// #### пример кругового движения ####
int circle_angle = 360;
float r = 30.0; // радиус (mm)
float h = 145.0; // среднее между ZMIN и ZMAX (mm)

void circulation(){
  curr_coord_FR.y = 0.0;

  curr_coord_FR.x = r * sin(radians(circle_angle));
  curr_coord_FR.z = (r * cos(radians(circle_angle))) + h;

  curr_angles_FR = angles_control(curr_coord_FR, false);
  movement();

  circle_angle -= 5;
  if(circle_angle <= 0){
    circle_angle = 360;
  }
}

// #### незаконченный и неправильно работающий пример ####
//float t = 0.0;
//char sig = 'u';
//
//void stepper(){
//  float x0, x1, x2, x3, y0, y1, y2, y3;
//  float L;
//
//  L = 80;
//  if (sig == 'u') {
//    // Кривая Безье с 4 точками
//    x0 = 0;
//    y0 = 0;
//
//    x1 = -40;
//    y1 = 20;
//
//    x2 = 120;
//    y2 = 20;
//
//    x3 = 80;
//    y3 = 0;
//    
//    curr_coord_FR.x = (1 - t) * ((1 - t) * ((1 - t) * x0 + t * x1) + t * ((1 - t) * x1 + t * x2)) +
//               t * ((1 - t) * ((1 - t) * x1 + t * x2) + t * ((1 - t) * x2 + t * x3));
//    curr_coord_FR.y = 0;
//    curr_coord_FR.z = (1 - t) * ((1 - t) * ((1 - t) * y0 + t * y1) + t * ((1 - t) * y1 + t * y2)) +
//               t * ((1 - t) * ((1 - t) * y1 + t * y2) + t * ((1 - t) * y2 + t * y3));
//
//  }
//  else if (sig == 'd') {
//    curr_coord_FR.x = L - L * t / 3;
//    curr_coord_FR.y = 0;
//    curr_coord_FR.z = 145;
//  }
//
//  curr_angles_FR = angles_control(curr_coord_FR, false);
//  movement();
//  
//  t += 0.01;
//  if(t >= 0.99){
//    t = 0.0;
//    sig = (sig=='u') ? 'd' : 'u';
//  }
//}
