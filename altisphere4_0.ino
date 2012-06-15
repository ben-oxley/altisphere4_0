// ___________________________
//|        Ben Oxley          |
//|AltiSphere Pre Release Code|
//|___________________________|
//Must initalise some variables on day of flight, may not work over midnight

//FIX: The release notes includes a handy pre-compiler directive to check of the arduino flavour you are using.

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <MemoryFree.h>
#include <SD.h>
#include <Servo.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
//#include <Time.h>
#include <WString.h> //
#include <ctype.h> 
#include <util/crc16.h> //Includes for crc16 cyclic redundancy check to validate serial comms
#include <stdio.h>
#include <stdlib.h>
#include <Arduino.h>

/*
 Output  0 - GPS_RXD          Output  7 - NICHROME_ENABLE
 Output  1 - GPS_TXD          Output  8 - SERVO_ENABLE 
 Output  2 - SERVO DATA       Output  9 - NOT LDO_ENABLE
 Output  3 - RADIO_TXD        Output 10 - SD_CS
 Output  4 - RADIO_ENABLE     Output 11 - MOSI
 Output  5 - GPS_TIMEPULSE    Output 12 - MISO
 Output  6 - N/C              Output 13 - SCK
*/
#define P_GPS_RXD 0
#define P_GPS_TXD 1
#define P_SERVO_DATA 2
#define P_RADIO_TXD 3
#define P_RADIO_EN 4
#define P_GPS_TIMEPULSE 5
#define P_CUTDOWN 7
#define P_SERVO_EN 8
#define P_LDO_EN 9
#define P_SD_CS 10
#define P_MOSI 11
#define P_MISO 12
#define P_SCK 13
/*
 Servo Goes between 18 and 90 Degrees
 
 Analog 0 - Pressure Sensor   Vout = VS × (0.2 × P(kPa)+0.5) ± 6.25% VFSS  Voltage is divided by two. Range 0.25-->2.25v Gives a resolution of 6 Pa (4000/(2/(3.3/1024)))
 Analog 1 - V_BATT_SERVO
 Analog 2 - N/C
 Analog 3 - N/C
 Analog 4 - Temperature Sensors SDA
 Analog 5 - Temperature Sensors SCL
 Analog 6 - N/C
 Analog 7 - V_BATT_MAIN
 */
#define A_PRES A0
#define V_SERVO A1
#define V_MAIN A7
/*
  GPS Level Convertor Board Test Script
 03/05/2012 2E0UPU
 
 Initialise the GPS Module in Flight Mode and then echo's out the NMEA Data to the Software Serial.
 
 This example code is in the public domain.
 Additional Code by J Coxon (http://ukhas.org.uk/guides:falcom_fsa03)
*/

//Define GPS Objects
TinyGPS gps;
SoftwareSerial mySerial(4, 5);
byte gps_set_sucess = 0 ;

//Servo Settings
#define servoOpen 100 //Servo open position
#define servoClosed 10 //Servo closed position
Servo vservo; //Create servo object for valve servo
int lastservopos = 90; //Integer to store the last given position of the servo

int tmp102Address = 0x48;

boolean haslaunched = false;

long lat, lon;
unsigned long fix_age, time, date, speed, course, alt;
int years;
byte months, days, hour, minutes, second, hundredths;

unsigned int packetNum = 1;


//Speed Variables
float speedarray[20]; //Declare array to store speed values to create moving average
int floatindex; //Declare the current place in the speedarray[]
int timelast; //Initialise time of last valid recieved packet
int lastservomove;
float altlast; //Initialise last recorded altitude variable
#define MOVINGAVG 10//Define size of moving average

//Pre-Flight Variables
#define datenow "220412" //Define date today (DDMMYY)
#define arraysize 6 //Define size of flight altitude array
int flightalt[] = {
  0,5000,10000,15000,20000,30000}; //Initialise variable array for altitude control points
int flightspd[] = {
  100,5,4,2,2,2}; //Initialise variable array for controlled speeds in m/s
  
  
char packet[100];

uint16_t crc;

void setup()
{
  analogReference(INTERNAL); //Use internal 1.1V reference voltage
  delay(10);
  mySerial.begin(9600);
  Serial.begin(9600);
  mySerial.println("Initialising....");
  pinMode(P_RADIO_TXD, OUTPUT);
  pinMode(P_RADIO_EN, OUTPUT);
  pinMode(P_SERVO_EN, OUTPUT);
  pinMode(P_LDO_EN, OUTPUT);
  vservo.attach(P_SERVO_DATA);          // attaches the servo on pin 9 to the servo object 
  vservo.write(servoClosed);                  // sets the servo position according to the scaled value 
  delay(15);                           // waits for the servo to get there 

  // THIS COMMAND SETS FLIGHT MODE AND CONFIRMS IT 
  mySerial.println("Setting uBlox nav mode: ");
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                        };
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
  }
  gps_set_sucess=0;

  // THE FOLLOWING COMMANDS DO WHAT THE $PUBX ONES DO BUT WITH CONFIRMATION
  /*
  debug.println("Switching off NMEA GLL: ");
  uint8_t setGLL[] = { 
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B                     };
  while(!gps_set_sucess)
  {		
    sendUBX(setGLL, sizeof(setGLL)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setGLL);
  }
  gps_set_sucess=0;

  debug.println("Switching off NMEA GSA: ");
  uint8_t setGSA[] = { 
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32                     };
  while(!gps_set_sucess)
  {	
    sendUBX(setGSA, sizeof(setGSA)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setGSA);
  }
  gps_set_sucess=0;
  debug.println("Switching off NMEA GSV: ");
  uint8_t setGSV[] = { 
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39                     };
  while(!gps_set_sucess)
  {
    sendUBX(setGSV, sizeof(setGSV)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setGSV);
  }
  gps_set_sucess=0;
  debug.print("Switching off NMEA RMC: ");
  uint8_t setRMC[] = { 
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40                     };
  while(!gps_set_sucess)
  {
    sendUBX(setRMC, sizeof(setRMC)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setRMC);
  }
  gps_set_sucess=0;
  debug.print("Switching off NMEA VTG: ");
  uint8_t setVTG[] = { 
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46                     };
  while(!gps_set_sucess)
  {
    sendUBX(setVTG, sizeof(setRMC)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setVTG);

  }
  */
}


void loop()
{

}     
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
    mySerial.print(MSG[i], HEX);
  }
  Serial.println();
}

void getgps() {
  gps.get_position(&lat, &lon, &fix_age);
  alt = gps.altitude();
  gps.crack_datetime(&years, &months, &days,
  &hour, &minutes, &second, &hundredths, &fix_age);
}

void valvecontrol() 
{
  if (haslaunched) {
      if (fix_age < 3600000) { //If the last GPS lock was less than an hour ago
    if ((lat < 5223) && (lon < 20) && (lon > 1)) {
      if ((fix_age > 60000) && (speedavg() > 1))
      {
        servopos(servoClosed);
      } 
      else {
        if (speedavg() > flightplan())
        {
          servopos(servoOpen);
        } else {
          servopos(servoClosed);
        }   
      }
    } 
    else {
      servopos(servoOpen);
    }
  } 
  else { 
    servopos(servoOpen); //open the valve to dump helium
  }
  } else if(alt > 500) {
      haslaunched = true;
  } else if(millis() - lastservomove > 10000) {
    servopos(servoClosed);
  }

}

void servopos(int pos) {
  digitalWrite(P_SERVO_EN,HIGH); //Turn the MOSFET on
  int x = 0;
  if (pos != lastservopos) //If the desired servo position has changed
    if (pos >= lastservopos) //If the servo position has increased
    {
      for (x = lastservopos; x <= pos; x++) //Slowly increase the servo's position
      {
        vservo.write(x);
        delay(10);
      }
    }
    else //Else the servo position has decreased
  {
    for (x = lastservopos; x >= pos; x--) //Slowly decrease the servo's position
    {
      vservo.write(x);
      delay(10);
    }
  }
  digitalWrite(P_SERVO_EN,LOW); //Turn the MOSFET off
  lastservopos = pos; // write the new servo position to the register
}
  


// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  mySerial.print(" * Reading ACK response: ");

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      mySerial.println(" (SUCCESS!)");
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      mySerial.println(" (FAILED!)");
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        mySerial.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}

int readTemperature()
{
  ADCSRA |= _BV(ADSC); // start the conversion
  while (bit_is_set(ADCSRA, ADSC)); // ADSC is cleared when the conversion finishes
  return (ADCL | (ADCH << 8)) - 342; // combine bytes & correct for temp offset (approximate)}
}

float averageTemperature()
{
  readTemperature(); // discard first sample (never hurts to be safe)
  float averageTemp; // create a float to hold running average
  for (int i = 1; i < 100; i++) // start at 1 so we dont divide by 0
    averageTemp += ((readTemperature() - averageTemp)/(float)i); // get next sample, calculate running average

  return averageTemp; // return average temperature reading
} 

void checkmem()
{
 mySerial.println( freeMemory() );
}

//from http://bildr.org/2011/01/tmp102-arduino/
float getextTemperature(){
  Wire.begin();
  delay(10);
  Wire.requestFrom(tmp102Address,2); 

  byte MSB = Wire.read();
  byte LSB = Wire.read();

  //it's a 12bit int, using two's compliment for negative
  int TemperatureSum = ((MSB << 8) | LSB) >> 4; 

  float celsius = TemperatureSum*0.0625;
  return celsius;
}


//******************************************
//Program to calculate the speed based on a
//moving average array.
//******************************************
float speedavg(){
  float speedcalc; //initalise float for speed
  for (int i = 0; i <= MOVINGAVG ; i++) { //loop through the values to add together
    if ((floatindex - i) < 0) { // if the index drops off the bottom of the array
      speedcalc += speedarray[ 20 - i + floatindex ]; // look at values decreasing from 99
    } 
    else {
      speedcalc +=speedarray[floatindex - i];
    }
  }
  speedcalc /= MOVINGAVG ; //divide by the amount of values added together
  return speedcalc; //return the value
}

//******************************************
//Program to verify the CRC check of the
//form CRC-CCITT. the program breaks when it
//finds * or \0
//******************************************
uint16_t CRC16 (char *c)
{
  uint16_t crc = 0xFFFF;
  while (*c && *c != '*') crc = _crc_xmodem_update(crc, *c++);
  return crc;
}

//******************************************
//Check flightspeed that is relevant to the
//current altitude. Needs an extended
//function to be written for descent.
//******************************************
float flightplan(){
  float targetspeed;
  for (int i = 0; i < arraysize; i++) //lookup which array element relates to the current altitude
  {
    if ( flightalt[i] <= alt) 
    {
      targetspeed = flightspd[i]; //looks up the target speed for this altitude
    }
  }
  return targetspeed; //returns the target speed to the calling function
}

void serialEvent(){
  gps.encode(Serial.read());
}

void transmit(){
  int result = sprintf(packet,"UUUU$$ALTI,%u,%u:%u:%u,%f,%f,%u*",packetNum,hour,minutes,second,lat,lon,alt);
  crc = (CRC16(&packet[6]));
  result = sprintf(&packet[result],"%04X\n",crc);
  //set timers
  //return
}


