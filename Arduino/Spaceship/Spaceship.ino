

int switchStateOld=HIGH;
int switchStateNew=LOW;
int Toggle = 1;

void setup() {
  // put your setup code here, to run once:
  pinMode (3,OUTPUT);
  pinMode (4,OUTPUT);
  pinMode (5,OUTPUT);
  pinMode (2,INPUT);
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  switchStateNew = digitalRead(2);

  if (switchStateOld == LOW && switchStateNew==HIGH ){
    if(Toggle == 0){
      Toggle=1;
    }else{
      Toggle=0;
    }
  }
  
  switchStateOld=switchStateNew;
  
  if (Toggle == 1){
    digitalWrite(3,1);
    digitalWrite(4,0);
  }
  else{
    digitalWrite(3,0);
    digitalWrite(4,1);

  }
  
  

}
