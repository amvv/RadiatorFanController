//Controller for the Briza radiator fan
//reads input and output water temperatures and controls the fan speed based on these
//Done:
// - Reads two DS18B20: finds the ID of the two sensors attached in the bus, and periodically reads them.
// - Calculate the PWM duty cycle based on the read temperatures.
//Todo:
// - Blink power light if DS18B20 are not found
// - change the output smoothly to the desired setpoint to avoid big bursts in the initial air circulation
// - Cleanup the code a bit
#include <OneWire.h>

#define  PWM_PIN  (0)
#define  LED_FAN  (1) //red
#define  LED_PWR  (2) //green

// DS18S20 Temperature chip i/o
OneWire ds(4);  // on pin 10

  byte addr[3][8];


int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
int pinstate = false;

void setup(void) {
  // initialize inputs/outputs
  // start serial port
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  pinMode(LED_FAN, OUTPUT);

  digitalWrite(LED_PWR, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_FAN, HIGH);    // turn the LED off by making the voltage LOW
  delay(500);               // wait for a second
  digitalWrite(LED_PWR, LOW);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_FAN, LOW);    // turn the LED off by making the voltage LOW
  delay(500);               // wait for a second

  
  byte i,k = 0;

  uint8_t searchresult = 0;


searchresult = ds.search(addr[0]);

while (searchresult > 0)
{

  if ( OneWire::crc8( addr[k], 7) != addr[k][7]) {
      digitalWrite(LED_FAN, HIGH);   // turn the LED on (HIGH is the voltage level)
      return;
  }

  if ( addr[k][0] == 0x10) {
    //digitalWrite(LED_FAN, HIGH);    // turn the LED off by making the voltage LOW
  }
  else if ( addr[k][0] == 0x28) {
    //digitalWrite(LED_FAN, HIGH);    // turn the LED off by making the voltage LOW
  }
  else {
      //digitalWrite(LED_FAN, HIGH);    // turn the LED off by making the voltage LOW
      //digitalWrite(LED_PWR, HIGH);    // turn the LED off by making the voltage LOW
      return;
  }
  digitalWrite(LED_PWR, HIGH);    // turn the LED off by making the voltage LOW
  delay(100);               // wait for a second
  digitalWrite(LED_PWR, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(500);               // wait for a second
  
  k+=1;
searchresult = ds.search(addr[k]);
}
ds.reset_search();

if (k>0)
{
    digitalWrite(LED_PWR, HIGH);    // turn the LED off by making the voltage LOW
}

}

void loop(void) {
  byte i, k=0;
  byte present = 0;
  byte data[12];
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract, pwm;
  float temperatures[2];
  float delta, maximum;

for (k=0; k<2;k++)
{
  ds.reset();
  ds.select(addr[k]);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr[k]);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative0
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25

  Whole = Tc_100 / 100;  // separate off the whole and fractional portions
  //Fract = Tc_100 % 100;

 //    Serial.print((int )k);
 //    Serial.print(": ");


  
  temperatures[k] = Whole;// + ((float )Fract)/100;
  if (SignBit) // If its negative
  {
     temperatures[k] = temperatures[k] * (-1);
  }
 // Serial.print(temperatures[k]);
 //    Serial.print("        ");
  
  
  
}
  
  
delta = abs(temperatures[0] - temperatures[1]);

//Serial.print(delta);
  
maximum = max(temperatures[0], temperatures[1]);
//Serial.print(maximum);
  

pwm = 0;

if (maximum > 25.0)
{
  pwm = 7*((int)(maximum)) - 70;
  if (pwm > 255)
  {
    pwm = 255;
  }
  digitalWrite(LED_FAN, true);
}
else
{
  digitalWrite(LED_FAN, false);
}
//Serial.print(pwm);
//Serial.println("pwm");
 
 // here control of the PWM
 
 
   analogWrite(PWM_PIN, pwm);    
  
}
