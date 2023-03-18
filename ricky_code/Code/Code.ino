#include <SoftwareSerial.h>
#include <TimerOne.h>
//--------------------------------------------------------
//TANQUE
int altura = 10;//cm
//--------------------------------------------------------
//--------------------------------------------------------
//Ultranic Configurations
const int trigPin = 9;
const int echoPin = 10;
float duration, distance;//cm
//--------------------------------------------------------


//--------------------------------------------------------
//Timer Configuration
int timertime = 10000; //0.01 seconds
int input;
void UltrasonicCompute(void){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  input = altura - distance;

  
  } 

  // to read a variable which the interrupt code writes, we
  // must temporarily disable interrupts, to be sure it will
  // not change while we are reading.  To minimize the time
  // with interrupts off, just quickly make a copy, and then
  // use the copy while allowing the interrupt to keep working.
  
  //noInterrupts();
  //blinkCopy = blinkCount;
  //interrupts();
//--------------------------------------------------------

//---------------------------------------------------------
//Bluetooth Configurations
//PINS 
//RX 3;TX 2;KEY 5;STATE 4
SoftwareSerial Bluetooth(3,2); //RX, TX
String dato;
String degC;
//---------------------------------------------------------

//---------------------------------------------------------
//PID Configurations

float T1,aux;       //Temperatura del Heater 1
float r1=0.0;  //Referencia del Heater 1
volatile float u=0.0,u_1=0.0;    //Acción de Control
byte Ts = 8; //Periodo de muestreo
//Parámetros del PID
float kp,ti,td;
volatile float q0,q1,q2;  
volatile float e=0.0,e_1=0.0,e_2=0.0;
      
float k=1.04,tau=160,theta=10+Ts/2;   //Parámetros del Modelo del sistema
float Tlc,eps,Wn,P1,P2,tau_d;            //Parámetros del diseño por asignación de polos

void PID(void)
{
    
    e=(r1-T1);
    // Control PID
      u = u_1 + q0*e + q1*e_1 + q2*e_2; //Ley del controlador PID discreto
    
    if (u >= 100.0)        //Saturo la accion de control 'uT' en un tope maximo y minimo
     u = 100.0;
    
    if (u <= 0.0 || r1==0)
     u = 0.0;
        
     //Retorno a los valores reales
     e_2=e_1;
     e_1=e;
     u_1=u;
     
     //La accion calculada la transformo en PWM
     
     analogWrite(PWM,map(u, 0,100, 0,255));
     
}

//---------------------------------------------------------


void setup() {
  //-------------------------------------------------------
  //Ultrasonic setup pins
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin, INPUT);
  //-------------------------------------------------------
  
  //-------------------------------------------------------
  //Bluetooth setup
  Bluetooth.begin(9600);
  //-------------------------------------------------------
  
  //-------------------------------------------------------
  //Timer setup
  Timer1.initialize(timertime);
  Timer1.attachInterrupt(UltrasonicCompute);
  //-------------------------------------------------------

  //-------------------------------------------------------
  //*************************************************************************//
  //*****************   SINTONIA POR ZIEGLER y NICHOLS    *******************//
  //*************************************************************************//
  kp=(1.2*tau)/(k*theta);
  ti=2.0*theta;
  td=0.5*theta;
  // Calculo do controle PID digital
  q0=kp*(1+Ts/(2.0*ti)+td/Ts);
  q1=-kp*(1-Ts/(2.0*ti)+(2.0*td)/Ts);
  q2=(kp*td)/Ts;
  //-------------------------------------------------------
}

void loop() {
  //------------------------------------------------------
  //Datos recibidos por Bluetooth
  //SETPOINT - kp - ki - kd
  if(Bluetooth.available()){
    
    dato = Bluetooth.readString();
    
    //Si el dato que se manda es una constante
    if(dato[0]=='k'){
      
      //kp para la constante Proporcional
      if(dato[1]=='p'){
        degC = dato.substring(2,(dato.length()-1));
        kp = degC.toInt();
        
        }
      
      //ki para la constante integral
      else if (dato[1]=='i'){
        degC = dato.substring(2,(dato.length()-1));
        ki = degC.toInt();
        }
      
      //kd para la constante derivativa
      else if (dato[1]=='d'){
        degC = dato.substring(2,(dato.length()-1));
        kd = degC.toInt();
        } 
      
      }
    
    //Si el dato que se manda es el Setpoint
    else if (dato[0]=='S'){
             for(i=0;i<10;i++){
                  if(dato[i]=='S'){
                    ini=i+1;
                    i=10;
                  }
                 }
                 for(i=ini;i<10;i++){
                  if(dato[i]=='$'){
                    fin=i;
                    i=10;
                  }
                 }
                 // salvo en degC el caracter con el escalon
                degC=dato.substring(ini, fin);
                r1 = degC.toInt();   // Convert character string to integers
                }
    }
    //--------------------------------------------------------------
    //--------------------------------------------------------------
      Serial.print("I");
      Serial.print(T1);
      Serial.print("F");
      Serial.print("I");
      Serial.print(T1);
      Serial.print("F");
  
      Serial.print("C");
      Serial.print(u);
      Serial.print("R");
      Serial.print("C");
      Serial.print(u);
      Serial.print("R");
    //--------------------------------------------------------------
    

    
}
