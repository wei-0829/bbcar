#include <Servo.h>   //載入函式庫

Servo servoLeft;   // 建立SERVO物件(左馬達)
Servo servoRight;   // 建立SERVO物件(右馬達)
int ultrasoundPin = 9;    // 超音波感測器 pin 9

long duration, inches, cm;  //宣告計算距離時，需要用到的兩個實數

void setup() {
  Serial.begin(9600);
  //設定要將伺服馬達接到哪一個PIN腳
  servoLeft.attach(13);
  servoRight.attach(12);  

  //1400 是順時針旋轉，逆時針旋轉為 1580 ，靜止為 1450
  servoLeft.writeMicroseconds(1700);   
  servoRight.writeMicroseconds(1300);   
}

void loop() {

  pinMode(ultrasoundPin, OUTPUT);
  digitalWrite(ultrasoundPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasoundPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(ultrasoundPin, LOW);

  pinMode(ultrasoundPin, INPUT);
  duration = pulseIn(ultrasoundPin, HIGH);

  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  if(inches < 10){
        servoLeft.writeMicroseconds(1300);   
        servoRight.writeMicroseconds(1700); 
  }
  else if(inches > 100){
    servoLeft.writeMicroseconds(1700);   
    servoRight.writeMicroseconds(1300); 
  }
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  delay(1000);
}

long microsecondsToInches(long microseconds)
 {
     return microseconds / 74 / 2;
  }

long microsecondsToCentimeters(long microseconds) 
{
   return microseconds / 29 / 2;
 }