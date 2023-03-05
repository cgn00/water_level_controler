#include <Arduino.h>
#include <SoftwareSerial.h>

#define EchoPin 7
#define TriggerPin 6

/*working variables*/
unsigned long lastTime;
double errSum, lastErr;
double Input, Output;
double Setpoint = 5; // reference in cm of water's level 
double kp = 1, ki , kd;

int CalculateDistance(int trigger_pin, int echo_pin); //forward declartion 

/// @brief Code to run once
void setup() 
{
  Serial.begin(9600);
  pinMode(TriggerPin, OUTPUT);
  pinMode(EchoPin, INPUT);
}

/// @brief Function to repeat forever
void loop() 
{
  int measurement = CalculateDistance(TriggerPin, EchoPin);
  Serial.print("Distancia: ");
  Serial.println(measurement);
  delay(1000);
}

/// @brief Calculate the level of the water
int CalculateDistance(int trigger_pin, int echo_pin)
{
  long duration;
  int distanceCm;
   
  digitalWrite(trigger_pin, LOW);  //para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds(4);
  digitalWrite(trigger_pin, HIGH);  //generamos Trigger (disparo) de 10us
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);
  
  duration = pulseIn(echo_pin, HIGH);  //medimos el tiempo entre pulsos, en microsegundos
  
  distanceCm = duration * 10 / 292/ 2;   //convertimos a distancia, en cm
  return distanceCm;
}

void Compute()
{
   /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);
  
   /*Compute all the working error variables*/
   double error = Setpoint - Input;
   errSum += (error * timeChange);
   double dErr = (error - lastErr) / timeChange;
  
   /*Compute PID Output*/
   Output = kp * error + ki * errSum + kd * dErr;
  
   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;
}
  
void SetTunings(double Kp, double Ki, double Kd)
{
   kp = Kp;
   ki = Ki;
   kd = Kd;
}