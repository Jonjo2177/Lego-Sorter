#include <Servo.h>

Servo ARM;


int ENA = 3;
int IN1 = 4;
int IN2 = 5;
int IN3 = 6;
int IN4 = 7;
int ENB = 8;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);

  ARM.attach(10);
  ARM.write(180);  // set servo to mid-point

}

void loop() {
  // put your main code here, to run repeatedly:

      analogWrite(ENA, 100); // Speed: 0–255


      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 150); // Speed: 0–255

      delay(100);

      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);

      delay(1000);


      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);

      delay(3000);

//Checks if there is data from python script
  if(Serial.available() > 0){
    String msg = Serial.readString(); //puts data into string 
      analogWrite(ENA, 100); // Speed: 0–255



      // Rotate forward off 1st belt


      //2nd belt rotates


    //using else-if or match case situations RGB&discard
    if (msg == "Blue") {
      for(int i = 0; i < 2; i++){
      digitalWrite(LED_BUILTIN,HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN,LOW);
      delay(100);
      }

      Serial.println("This is blue");
      //Sets Arms to BLUE BIN
      ARM.write(40);
      delay(100);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);

      delay(1000);
        // Rotate forward off 1st belt
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);

      delay(1000);

      //2nd belt rotates
      

    } 

    else if(msg == "Green"){
      for(int i = 0; i < 3; i++){
      digitalWrite(LED_BUILTIN,HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN,LOW);
      delay(100);
      }
      //Sets Arms to BLUE BIN
      ARM.write(50);
      delay(100);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);

      delay(1000);
        // Rotate forward off 1st belt
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);

      delay(1000);
    }
    
    else if (msg == "Red"){
      for(int i = 0; i < 4; i++){
      digitalWrite(LED_BUILTIN,HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN,LOW);
      delay(100);
      }
      
      //Sets Arms to Red BIN
      ARM.write(120);
      delay(100);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);

      delay(1000);
        // Rotate forward off 1st belt
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      delay(1000);

    }

    //Discard Bin
    else{
      
      for(int i = 0; i < 5; i++){
      digitalWrite(LED_BUILTIN,HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN,LOW);
      delay(100);
      }


      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);

      delay(1000);
        // Rotate forward off 1st belt
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);

      delay(1000);

    }
  }

}
