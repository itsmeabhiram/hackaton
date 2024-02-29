#include <BluetoothSerial.h>
#define pwma 18
#define in1 2
#define in2 15

#define in3 4
#define in4 5
#define pwmb 17

BluetoothSerial SerialBT;

String data = "";
float x,y;
int mode = 0;

int initspeed = 175;

void motor1(int pwm1){
  if(pwm1>0){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }else{
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  analogWrite(pwma,abs(pwm1));
}

void motor2(int pwm2){
  if(pwm2>0){
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
  }else{
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
  }
  analogWrite(pwmb,abs(pwm2));
}

void stop(){
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  analogWrite(pwma,0);
  analogWrite(pwmb,0);
}

void init_motor(){
  pinMode(pwma,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(pwmb,OUTPUT);
}

void setup(){
  Serial.begin(115200);
  SerialBT.begin("16BIT");
  init_motor();
}

void loop(){
  if(SerialBT.available()){
    data = SerialBT.readStringUntil('\n');
    sscanf(data.c_str(), "%f,%f,%d", &y, &x, &mode);
    Serial.println(x);
    motor1(initspeed + (x*2));
    motor2(initspeed - (x*2));
    delay(50);

  }
  else stop();
}