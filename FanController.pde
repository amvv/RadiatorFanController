//Controller for the Briza radiator fan
//reads input and output water temperatures and controls the fan speed based on these
//Done:
// - Reads two DS18B20: finds the ID of the two sensors attached in the bus, and periodically reads them.
//Todo:
// - Calculate the PWM duty cycle based on the read temperatures.
// - change the output smoothly to the desired setpoint to avoid big bursts in the initial air circulation
// - Cleanup the code a bit


#include <OneWire.h>

// DS18S20 Temperature chip i/o
OneWire ds(6);  // on pin 10

  byte addr[3][8];


int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
int pinstate = false;


void setup(void) {
  // initialize inputs/outputs
  // start serial port
  Serial.begin(9600);
  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(3, pinstate);
  
  
  byte i,k = 0;

  uint8_t searchresult = 0;


searchresult = ds.search(addr[0]);

while (searchresult > 0)
{

  Serial.print("R=");
  for( i = 0; i < 8; i++) {
    Serial.print(addr[k][i], HEX);
    Serial.print(" ");
  }

  if ( OneWire::crc8( addr[k], 7) != addr[k][7]) {
      Serial.print("CRC is not valid!\n");
      return;
  }

  if ( addr[k][0] == 0x10) {
      Serial.print("Device is a DS18S20 family device.\n");
  }
  else if ( addr[k][0] == 0x28) {
      Serial.print("Device is a DS18B20 family device.\n");
  }
  else {
      Serial.print("Device family is not recognized: 0x");
      Serial.println(addr[k][0],HEX);
      return;
  }
  
  k+=1;
searchresult = ds.search(addr[k]);
}
ds.reset_search();

      Serial.print("finished serialisation");
Serial.print((int )k);
      Serial.print("devices");
}

void loop(void) {
  byte i, k=0;
  byte present = 0;
  byte data[12];
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;
  float temperatures[2];
  float delta;

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
  Fract = Tc_100 % 100;

     Serial.print((int )k);
     Serial.print(": ");


  
  temperatures[k] = Whole + ((float )Fract)/100;
  if (SignBit) // If its negative
  {
     temperatures[k] = temperatures[k] * (-1);
  }
  Serial.print(temperatures[k]);
     Serial.print("        ");
  
  
  
}
  
  
delta = abs(temperatures[0] - temperatures[1]);

Serial.print(delta);
  
Serial.print("\n");
 
 
 // here control of the PWM
 
 
   analogWrite(5, brightness);    

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade: 
  if (brightness == 0 || brightness == 255) {
    delay(2000);
    fadeAmount = -fadeAmount ; 
    pinstate = !pinstate;
    digitalWrite(3, pinstate);
  }     
  // wait for 30 milliseconds to see the dimming effect    
  //delay(30);                            

  
}
