void setup()
{
  Serial.begin(9600);
}

void loop()
{
  long randNumber = random(300);
  Serial.println(randNumber);
  
  delay(1000);
}
