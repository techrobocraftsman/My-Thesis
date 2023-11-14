String msg;
int value=10;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN,OUTPUT);
}

void loop() {
  if(Serial.available()>0){
    msg = Serial.readStringUntil('\n');
    /*
    if(msg == "u"){
      value++;
//      Serial.print(value);
    }
    else if(msg == "d"){
      value--;
//      Serial.print(value);
    }
    else if(msg == "l"){
      value=0;
//      Serial.print(value);
    }
    else if(msg == "r"){
      value=255;
//      Serial.print(value);
    }
    else Serial.write("invalid command");   
    */
    delay(10);
    Serial.print("the msg was: " + msg);
  }
//  delay(10);
//  value = constrain(value,0,255);
//  analogWrite(LED_BUILTIN,value);
}
