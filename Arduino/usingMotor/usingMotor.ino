
int buttonState = 0;
int motorPin = 9;
int buttonPin = 2;

void setup() {
  // put your setup code here, to run once:
  pinMode (motorPin, OUTPUT);
  pinMode (buttonPin, INPUT);
    Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH){
    digitalWrite(motorPin, HIGH);
    Serial.println("in High");
   
    
  }else{
    digitalWrite(motorPin,LOW);
        Serial.println("in Low");
  }

}
