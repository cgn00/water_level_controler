#include <Arduino.h>
#include <SoftwareSerial.h>

// ultrasonic
#define EchoPin 7
#define TriggerPin 6

// driver's pump
#define PWM_PIN 11
#define INPUT_1 12
#define INPUT_2 10

// bluetooth
SoftwareSerial Bluetooth(3, 2); // Rx Tx password = 7580

/*working variables*/
unsigned long lastTime;
double errSum, lastErr;
//double Input = 0, Output = 0;
double PWM_output = 0;
double reference = 5; // reference in cm of water's level 
double kp = 10, ki = 0.002 , kd = 0;

//
double CalculateDistance(int trigger_pin, int echo_pin); //forward declartion 
double Compute(double setpoint, int input);

/// @brief Code to run once
void setup() 
{
  Serial.begin(9600);

  // bluetooth
  Bluetooth.begin(9600);
  
  // ultrasonic
  pinMode(TriggerPin, OUTPUT);
  pinMode(EchoPin, INPUT);

  // driver's pump
  pinMode(PWM_PIN, OUTPUT);
  pinMode(INPUT_1, OUTPUT);
  pinMode(INPUT_2, OUTPUT);
  digitalWrite(INPUT_1, LOW); // direction of the L298 driver
  digitalWrite(INPUT_2, HIGH); // direction of the L298 driver

}

/// @brief Function to repeat forever
void loop() 
{
  double measurement = CalculateDistance(TriggerPin, EchoPin);
  measurement = 17 - measurement; // 17 cm is the height of the tank, then the label of the water is: (17 - the measurement)

  Bluetooth.print("Distancia: ");
  Bluetooth.println(measurement);
  

  PWM_output = Compute(reference, measurement);
  analogWrite(PWM_PIN, PWM_output);

  Bluetooth.print("PWM_output = ");
  Bluetooth.println(PWM_output);
}

/// @brief Calculate the level of the water
double CalculateDistance(int trigger_pin, int echo_pin)
{
  double duration;
  double distanceCm;
   
  digitalWrite(trigger_pin, LOW);  //para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds(4);
  digitalWrite(trigger_pin, HIGH);  //generamos Trigger (disparo) de 10us
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);
  
  duration = pulseIn(echo_pin, HIGH);  //medimos el tiempo entre pulsos, en microsegundos
  
  distanceCm = duration * 10 / 292/ 2;  //convertimos a distancia, en cm
  distanceCm -=1 ; // to calibrate the mistake of the measurement
  return distanceCm;
}

/// @brief Output the PWM signal control that a PID algorithm compute
double Compute(double setpoint, int input)
{
   /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);

   double output = 0;
  
   /*Compute all the working error variables*/
   double error = (setpoint - input);
   errSum += (error * timeChange);
   double dErr = (error - lastErr) / timeChange;
  
   /*Compute PID Output*/
   output = kp * error + ki * errSum + kd * dErr;
  
   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;

   if(output > 240)
   {
    output = 240;
   }
   else if(output < 130)
   {
    output = 0;
   }

   return output;
}
  
void SetTunings(double Kp, double Ki, double Kd)
{
   kp = Kp;
   ki = Ki;
   kd = Kd;
}