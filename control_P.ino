#define P_Kp A1 // Se redefinen las entradas analogicas
#define P_Pos A2
#define P_Ref A0

int V_P_Kp=0;
int V_P_Pos=0;  //Variable para obtener valores de un potenciometro de posición
int V_P_Ref=0;  //variable para tomar el valor de referencia

//int PWM1 = 5;  //Pin digital controlar el PWM del motor
#include <Servo.h>
Servo servo1;
int M1=3;   // Pines para el sentido del motor
int M2=2;

int Error = 0;  //Variable para almacenar el valor del error (V_P_Ref-V_P_Pos)
int V_PWM1=0;  //Variable para almacenar el valor del PWM

int aux_read=0;

void setup()
{
  Serial.begin(9600);
  //pinMode(PWM1, OUTPUT);
  pinMode(M1,OUTPUT);
  pinMode(M2,OUTPUT);

  //Motor apagado
  //analogWrite(PWM1,0);
  servo1.attach(5);
  servo1.write(0);
  digitalWrite(M1,LOW);
  digitalWrite(M2,LOW);
}

void loop()
{

  if(aux_read==0)
    {
     for(int i=0; i<20;i++) 
        {
          //Leer entradas analogicas
            V_P_Pos=analogRead(P_Pos);
            V_P_Ref=analogRead(P_Ref);
            V_P_Kp=analogRead(P_Kp);
            V_P_Kp=map(V_P_Kp,0,1023,0,10);
            Serial.print("P_Ref= ");
            Serial.print(V_P_Ref);
            Serial.print("  P_Pos= ");
            Serial.print(V_P_Pos);
            Serial.print("  P_Kp= ");
            Serial.print(V_P_Kp);
            Serial.print("   ---Error= ");
            Serial.print(Error);
            Serial.print("  PWM= ");
            Serial.println(V_PWM1);
            Serial.println("---------------------------"); 
            delay(200);
        }
      aux_read=1;
    }
  else
    {
      control();  
    }
  
}




void control()
{
  //Leer entradas analogicas
  V_P_Pos=analogRead(P_Pos);
  V_P_Ref=analogRead(P_Ref);
  V_P_Kp=analogRead(P_Kp);
  V_P_Kp=map(V_P_Kp,0,1023,0,10);

  Error=V_P_Ref-V_P_Pos;
  V_PWM1=abs(Error*V_P_Kp);
  V_PWM1=map(V_PWM1,0,1023,0,180); 
  if(Error==0)
     {
      V_PWM1=0;
      //analogWrite(PWM1,V_PWM1);
      servo1.write(0);
      digitalWrite(M1,LOW);
      digitalWrite(M2,LOW);
     }
  else
     {
      if(Error<0)
         {
          //analogWrite(PWM1,V_PWM1);
          servo1.write(V_PWM1);
          digitalWrite(M1,LOW);
          digitalWrite(M2,HIGH);
         }
       else
         {
          //analogWrite(PWM1,V_PWM1);
          servo1.write(V_PWM1);
          digitalWrite(M1,HIGH);
          digitalWrite(M2,LOW);
         }
       Serial.print("P_Ref= ");
       Serial.print(V_P_Ref);
       Serial.print("  P_Pos= ");
       Serial.print(V_P_Pos);
       Serial.print("  P_Kp= ");
       Serial.print(V_P_Kp);
       Serial.print("   --- Error= ");
       Serial.print(Error);
       Serial.print("  PWM= ");
       Serial.println(V_PWM1);
       Serial.println("---------------------------"); 
     }
  delay(10);
}

