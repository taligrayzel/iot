int i = 0;
void setup() {
  Serial.begin(9600);
}
void loop() {
  for (i = 0; i <= 9; i++)
  {
    Serial.print("debug test ");
    Serial.println(i);
    delay(500);         
  }
}

