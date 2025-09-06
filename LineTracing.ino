void Led_On(){
  digitalWrite(10, HIGH);
}
void Led_OFF(){
  digitalWrite(10, LOW);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(10, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  Led_On();
  delay(100);
  Led_OFF();
  delay(100);
}
