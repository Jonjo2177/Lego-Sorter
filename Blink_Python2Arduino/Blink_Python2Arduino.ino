void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

//Checks if there is data from python script
  if(Serial.available() > 0){
    String msg = Serial.readString(); //puts data into string 

    //using else-if or match case situations
    if (msg == "ON") {
      digitalWrite(LED_BUILTIN,HIGH);

    } 

    else if(msg == "OFF"){
      digitalWrite(LED_BUILTIN,LOW);
    }

    else{

      for(int i = 0; i < 5; i++){
      digitalWrite(LED_BUILTIN,HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN,LOW);
      delay(100);
      }
    }
  }

}
