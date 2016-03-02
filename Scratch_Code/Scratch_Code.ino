long int start,stopp,timer,value;
void setup() {
  DDRD|=B11011000;
  DDRB|=B00110000;
  PORTB|=B00110000;
  
  value=2400;
  timer=millis();
  while((millis()-timer)<10000)
  {
  start=micros();
  PORTD|=B11011000;
  while((micros()-start)<value);
  PORTD&=B00000011;
  while(micros()-start<50000);
  } 
timer=millis();
  value=600;
  while(millis()-timer<3000)
  {
  start=micros();
  PORTD|=B11011000;
  while((micros()-start)<value);
  PORTD&=B00000011;
  while(micros()-start<20000);
  } 
 }
void loop()
{
  value=2000;
  start=micros();
  PORTD|=B11011000;
  while((micros()-start)<value);
  PORTD&=B0000011;
  while(micros()-start<20000);
}
