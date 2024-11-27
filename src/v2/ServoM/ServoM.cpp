/*###########################################
## Author's github:    madport             ##
#############################################
*/
#include "ServoM.h"
#include "Arduino.h"

/* States of turn */ 
enum
{
  FORWARD = 0,
  BACKWARD = 1,
};

ServoM::ServoM(float time_turn, int min_pos, int max_pos)
{
  timeTurn_ = time_turn;
  min_pos_ = min_pos;
  max_pos_ = max_pos;
}

void ServoM::Forward()
{
  if (!acelerated_)
  {
    long mills = millis();    
    
    for (int i = 1; i <= STEP_; i++)
    {
      if (millis() <= (TACEL_ * i) + timef_){ this->write(LEFT_ - i + 1); }
      if (millis() <= (TACEL_ * STEP_) + timef_){ acelerated_ = true; }
    }
  }
  Serial.print("FORW:");
  Serial.println(pos_);
}

void ServoM::Backward()
{
  if (!acelerated_)
  {
    long mills = millis();    
    
    for (int i = 1; i <= STEP_; i++)
    {
      if (millis() <= (TACEL_ * i) + timef_){ this->write(RIGHT_ + i - 1); }
      if (millis() <= (TACEL_ * STEP_) + timef_){ acelerated_ = true; }
    }
  }
  Serial.print("BACK:");
  Serial.println(pos_);
}

void ServoM::Stop(){ this->write(STOP_); acelerated_ = false; }

double ServoM::check_grades(double grades)
{
   /* Limit forward*/
   if (pos_ + grades > max_pos_){ grades = max_pos_ - pos_; }
  
   /* Limit backward and changes mode */
   if (grades < 0 && (pos_ + grades) < min_pos_){ grades = min_pos_ - pos_; }  

   return grades;
}

void ServoM::turn(double grades)
{
  long actual_millis = millis();
  grades = check_grades(grades);
  if (grades > 0) { mode = FORWARD; }
  if (grades < 0) { mode = BACKWARD; grades *= -1; }

  if (!turning_ && grades != 0)
  {
    time0_ = millis();
    timef_ = ((grades / 180.0) * timeTurn_) + time0_;
    target_ = grades;
    turning_ = true;
  }

  if (!acelerated_ && turning_)
  {
    if (mode == FORWARD) { pos_ += grades; Forward(); }
    if (mode == BACKWARD) { pos_ -= grades; Backward(); }
  }
  
  if (timef_ <= actual_millis)
  {
    Stop();
    target_ = 0;
    turning_ = false;
  }
  Serial.print("TURN:");
  Serial.println(pos_);
}

int currentPos = 180;    // Variable que representa la posición actual estimada del servo (en grados)
int targetPos = 0;
int speedLeft = 80;    // Velocidad para rotación hacia la izquierda (< 90)
int speedRight = 100;  // Velocidad para rotación hacia la derecha (> 90)
int stopSpeed = 90;    // Velocidad para detener el servo

unsigned long moveTime;  // Tiempo estimado para moverse

unsigned long previousMillis = 0;  // Tiempo previo en que se ejecutó la última acción
unsigned long moveInterval = 0;    // Intervalo de tiempo calculado para el movimiento

bool isMoving = false;


// Función para estimar el tiempo de movimiento (ajusta la fórmula según tu sistema)
unsigned long ServoM::estimateMoveTime(int start, int end) {
  int angleDifference = abs(end - start);
  return map(angleDifference, 0, 180, 0, 1000);  // Suponemos que tarda 2 segundos en moverse 180 grados
}

// Función para mover el servo hacia la posición deseada
void ServoM::moveServoTo(int targetPos, unsigned long moveTime) {

  previousMillis = millis();  // Registrar el tiempo actual
  isMoving = true;

  if (targetPos > currentPos) {
    // Si el objetivo es mayor que la posición actual, moverse a la derecha
    this->write(speedRight);  // Mover a la derecha
  } else if (targetPos < currentPos) {
    // Si el objetivo es menor que la posición actual, moverse a la izquierda
    this->write(speedLeft);   // Mover a la izquierda
  }
}


void ServoM::updateServoMovement() {
  if (isMoving) {
    unsigned long currentMillis = millis();  // Obtener el tiempo actual

    // Verificar si ya hemos alcanzado el tiempo de movimiento necesario
    if (currentMillis - previousMillis >= moveInterval) {
      this->write(stopSpeed);  // Detener el servo cuando se alcance el tiempo objetivo
      currentPos = targetPos;    // Actualizar la posición actual
      isMoving = false;  // Indicar que el movimiento ha terminado
    }
  }
}

void ServoM::goTo(double angle)
{
  if (!isMoving) {
    targetPos = angle;
    moveTime = estimateMoveTime(currentPos, angle);
    moveServoTo(angle, moveTime);
  }
  updateServoMovement();
}



