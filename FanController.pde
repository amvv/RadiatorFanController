//Controller for the Briza radiator fan
//reads input and output water temperatures and controls the fan speed based on these
//Done:
// - Reads two DS18B20: finds the ID of the two sensors attached in the bus, and periodically reads them.
//Todo:
// - Calculate the PWM duty cycle based on the read temperatures.
// - change the output smoothly to the desired setpoint to avoid big bursts in the initial air circulation
// - Cleanup the code a bit
#include <OneWire.h>
#include <RF12.h>
#include <Ports.h>

#define  PWM_PIN  (5)
#define  LED_PIN  (3)

// DS18S20 Temperature chip i/o
OneWire ds(6);  // on pin 10

  byte addr[3][8];


int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
int pinstate = false;

typedef struct {
        unsigned int house;
  unsigned int device;
	unsigned int seq;
	byte intemp;
	byte outtemp;
	byte pwm;
} RadiatorData;

RadiatorData buf;
byte data_to_send = false;
int counter = 0;
void setup(void) {
  // initialize inputs/outputs
  // start serial port
  Serial.begin(9600);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, pinstate);
  
  
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

  rf12_initialize(11, RF12_868MHZ, 212); 
  
  
  buf.house = 192;
  buf.device = 10;
  buf.seq = 1;
  
  rf12_sleep(0);



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
  
maximum = max(temperatures[0], temperatures[1]);
Serial.print(maximum);
  

pwm = 0;

if (maximum > 25.0)
{
  pwm = 6*((int)(maximum)) - 25;
  if (pwm > 255)
  {
    pwm = 255;
  }
  digitalWrite(LED_PIN, false);
}
else
{
  digitalWrite(LED_PIN, true);
}
Serial.print(pwm);
Serial.println("pwm");
 
 // here control of the PWM
 
 
   analogWrite(PWM_PIN, pwm);    

counter = counter + 1;
if (counter == 10)
{
  data_to_send = true;
  counter = 0;
}


  if (data_to_send == true)
  {
    buf.intemp = temperatures[0];
    buf.outtemp= temperatures[1];
    buf.pwm = pwm;
    data_to_send = false;
    
    rf12_sleep(-1);
    buf.seq++;
      while (!rf12_canSend())	// wait until sending is allowed
       rf12_recvDone();

       rf12_sendStart(0, &buf, sizeof buf);

      while (!rf12_canSend())	// wait until sending has been completed
         rf12_recvDone();
         delay(5);
      rf12_sleep(0);
  }
  
}
