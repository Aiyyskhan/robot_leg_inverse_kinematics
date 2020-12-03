/*
 * 
 * Inverse kinematics algorithm
 * 
 * Simple test sketch 
 * 
 * created in 09.09.2020
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
const long interval = 30; // milliseconds
float lateralGain = 15;
double increment = 0.01;
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
  init_coord_FR.x = 0.0;
  init_coord_FR.y = 0.0;
  init_coord_FR.z = 180.0;

  coord_FR.x = 0.0;
  coord_FR.y = 0.0;
  coord_FR.z = 0.0;
  
  // установка конечностей в начальное положение
  initial_pose();
  delay(1000);
}

float i = 0.0;
char sig = 'd';

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    //    if(i >= 0.99){
    //      i = 0.0;
    //      sig = (sig == 'd') ? 'u':'d';
    //    }

    //    step_coord_FR = stepper(i, sig);
    //    coord_FR.x = init_coord_FR.x + step_coord_FR.x;
    //    coord_FR.y = init_coord_FR.y + step_coord_FR.y;
    //    coord_FR.z = init_coord_FR.z + step_coord_FR.z; 

    if(coord_FR.x >= XMAX){
      coord_FR.x = XMAX;
      i = 0.0;
      sig = 'd';
    }
    else if(coord_FR.x <= XMIN){
      coord_FR.x = XMIN;
      i = 0.0;
      sig = 'u';
    }

    if(sig=='d'){
      coord_FR.x = coord_FR.x - 0.5;
      coord_FR.y = init_coord_FR.y;
      coord_FR.z = init_coord_FR.z; 
    }
    else if(sig=='u'){
      coord_FR.x = coord_FR.x + 0.5;
      coord_FR.y = init_coord_FR.y;
      coord_FR.z = init_coord_FR.z - 20.0; 
    }
    
    angles_FR = angles_control(coord_FR, false);
    movement();

    i += 0.5; //increment;
  }
  
  //  for (double i = 0 ; i <= 0.99 ; i += increment){
  //    unsigned long currentMillis = millis();
  //    if (currentMillis - previousMillis >= interval) {
  //      previousMillis = currentMillis;
  //
  //      step_coord_FR = stepper(i+2, 'd');
  //      coord_FR.x = init_coord_FR.x + step_coord_FR.x;
  ////      coord_FR.y = init_coord_FR.y + lateralGain * i;/ 
  //      coord_FR.z = init_coord_FR.z + step_coord_FR.z;      
  //
  //      angles_FR = angles_control(coord_FR, false);
  //      movement();
  //    }
  //  }
  //
  //  for (double i = 0 ; i <= 0.99 ; i += increment){
  //    unsigned long currentMillis = millis();
  //    if (currentMillis - previousMillis >= interval) {
  //      previousMillis = currentMillis;
  //
  //      step_coord_FR = stepper(i, 'u');
  //      coord_FR.x = init_coord_FR.x + step_coord_FR.x;
  ////      coord_FR.y = init_coord_FR.y + lateralGain * (1-i);/
  //      coord_FR.z = init_coord_FR.z + step_coord_FR.z;
  //
  //      angles_FR = angles_control(coord_FR, false);
  //      movement();
  //    }
  //  }
  //
  //  for (double i = 0 ; i <= 0.99 ; i += increment){
  //    unsigned long currentMillis = millis();
  //    if (currentMillis - previousMillis >= interval) {
  //      previousMillis = currentMillis;
  //
  //      step_coord_FR = stepper(i, 'd');
  //      coord_FR.x = init_coord_FR.x + step_coord_FR.x;
  ////      coord_FR.y = init_coord_FR.y - lateralGain * i;/
  //      coord_FR.z = init_coord_FR.z + step_coord_FR.z;
  //
  //      angles_FR = angles_control(coord_FR, false);
  //      movement();
  //    }
  //  }
  //
  //  for (double i = 0 ; i <= 0.99 ; i += increment){
  //    unsigned long currentMillis = millis();
  //    if (currentMillis - previousMillis >= interval) {
  //      previousMillis = currentMillis;
  //
  //      step_coord_FR = stepper(i+1, 'd');
  //      coord_FR.x = init_coord_FR.x + step_coord_FR.x;
  ////      coord_FR.y = init_coord_FR.y - lateralGain * (1-i);/
  //      coord_FR.z = init_coord_FR.z + step_coord_FR.z;
  //
  //      angles_FR = angles_control(coord_FR, false);
  //      movement();
  //    }
  //  }
}

// #### пример кругового движения ####
int circle_angle = 360;
float r = 30.0; // радиус (mm)
float h = 145.0; // среднее между ZMIN и ZMAX (mm)

void circulation(){
  coord_FR.y = 0.0;

  coord_FR.x = r * sin(radians(circle_angle));
  coord_FR.z = (r * cos(radians(circle_angle))) + h;

  angles_FR = angles_control(coord_FR, false);
  movement();

  circle_angle -= 5;
  if(circle_angle <= 0){
    circle_angle = 360;
  }
}

// #### незаконченный и неправильно работающий пример ####

Coordinates stepper(double t, char sig){
  Coordinates coord;
  double x0, x1, x2, x3, z0, z1, z2, z3;
  double L;

  L = 70.0;
  if (sig == 'u') {
    // Кривая Безье с 4 точками
    //    x0 = 0.0;
    //    z0 = 180.0;
    //
    //    x1 = -10.0;
    //    z1 = 150.0;
    //
    //    x2 = 80.0;
    //    z2 = 150.0;
    //
    //    x3 = 70.0;
    //    z3 = 180.0;
    //    
    //    double oneMinusT = 1.0 - t;
    //
    //    /*
    //    coord.z = oneMinusT * (oneMinusT * (oneMinusT * x0 + t * x1) + t * (oneMinusT * x1 + t * x2)) +
    //               t * (oneMinusT * (oneMinusT * x1 + t * x2) + t * (oneMinusT * x2 + t * x3));
    //    coord.y = 0;
    //    coord.x = oneMinusT * (oneMinusT * (oneMinusT * z0 + t * z1) + t * (oneMinusT * z1 + t * z2)) +
    //               t * (oneMinusT * (oneMinusT * z1 + t * z2) + t * (oneMinusT * z2 + t * z3));
    //    */
    //
    //    coord.z = oneMinusT * oneMinusT * oneMinusT * x0 + 
    //              3.0 * oneMinusT * oneMinusT * t * x1 +
    //              3.0 * oneMinusT * t * t * x2 +
    //              t * t * t * x3;
    //    coord.y = 0;
    //    coord.x = oneMinusT * oneMinusT * oneMinusT * z0 + 
    //              3.0 * oneMinusT * oneMinusT * t * z1 +
    //              3.0 * oneMinusT * t * t * z2 +
    //              t * t * t * z3;
    coord.x = L * t / 3;
    coord.y = 0;
    coord.z = 0;    
  }
  else if (sig == 'd') {
    coord.x = L - L * t / 3;
    coord.y = 0;
    coord.z = 0;
  }

  return coord;
}
