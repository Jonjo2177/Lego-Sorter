int ENA = 3;
int IN1 = 4;
int IN2 = 5;
int IN3 = 6;
int IN4 = 7;
int ENB = 8;

int ARM = 10;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ARM, OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly:

    //1st belt begins rotating
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 100); // Speed: 0–255
    //2nd belt rotates
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 100); // Speed: 0–255

    //----stops for scanning----
    delay(1000); 

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 100); // Speed: 0–255
    //2nd belt rotates
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 100); // Speed: 0–255

    delay(3000); 

/*
//Checks if there is data from python script
  if(Serial.available() > 0){
    String msg = Serial.readString(); //puts data into string 


    //using else-if or match case situations RGB&discard
    if (msg == "Blue") {
      for(int i = 0; i < 2; i++){
      digitalWrite(LED_BUILTIN,HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN,LOW);
      delay(100);
      }
      //Sets Arms to BLUE BIN
        digitalWrite(ARM, 30);

        // Rotate forward off 1st belt
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 100); // Speed: 0–255
      //2nd belt rotates
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 100); // Speed: 0–255

    } 

    else if(msg == "Green"){
      for(int i = 0; i < 2; i++){
      digitalWrite(LED_BUILTIN,HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN,LOW);
      delay(100);
      }
      //Sets Arms to BLUE BIN
        digitalWrite(ARM, 30);

        // Rotate forward off 1st belt
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 100); // Speed: 0–255
      //2nd belt rotates
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 100); // Speed: 0–255    
    }
    
    else if (msg == "Red"){
      for(int i = 0; i < 3; i++){
      digitalWrite(LED_BUILTIN,HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN,LOW);
      delay(100);
      }
      //Sets Arms to BLUE BIN
      digitalWrite(ARM, 30);

      // Rotate forward off 1st belt
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 100); // Speed: 0–255
      //2nd belt rotates
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 100); // Speed: 0–255
    }

    //Discard Bin
    else{
      
      for(int i = 0; i < 5; i++){
      digitalWrite(LED_BUILTIN,HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN,LOW);
      delay(100);
      }

        // Rotate forward off 1st belt
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 100); // Speed: 0–255
      //2nd belt rotates
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 100); // Speed: 0–255
    }
  }*/

}
