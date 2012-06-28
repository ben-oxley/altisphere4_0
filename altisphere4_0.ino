// ___________________________
//|        Ben Oxley          |
//|AltiSphere Pre Release Code|
//|___________________________|
//Must initalise some variables on day of flight, may not work over midnight

//#define DEBUG
//#define W_TEMP_PROG

#ifdef DEBUG
#define DEBUG_PRINT(x)     Serial.print (x)
#define DEBUG_PRINTDEC(x)     Serial.print (x, DEC)
#define DEBUG_PRINTHEX(x)  Serial.print (x,HEX)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#define DEBUG_WRITE(x) Serial.write (x)
#define DEBUG_SD_WRITE(x) logchar (x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTHEX(x)
#define DEBUG_WRITE(x)
#define DEBUG_SD_WRITE(x)
#endif  

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <MemoryFree.h>
#include <SD.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <Servo.h>
#include <util/crc16.h> //Includes for crc16 cyclic redundancy check to validate serial comms

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
 Analog 0 - Pressure Sensor
 Analog 1 - V_BATT_SERVO
 Analog 2 - N/C
 Analog 3 - N/C
 Analog 4 - Temperature Sensor's SDA
 Analog 5 - Temperature Sensor's SCL
 Analog 6 - N/C
 Analog 7 - V_BATT_MAIN
 */
#define A_PRES A0
#define V_SERVO A1
#define V_MAIN A7

//Define GPS Objects
TinyGPS gps;

//Servo Settings
#define servoOpen 100 //Servo open position
#define servoClosed 10 //Servo closed position
Servo vservo; //Create servo object for valve servo
int lastservopos = 10; //Integer to store the last given position of the servo

int tmp102Address = 0x48; //(1001000 or A0 = A1 = A2 = LOW)

//GPS Variables
long lat, lon;
float f_lat, f_lon, f_alt;
unsigned long fix_age, time, date, speed, course, alt;
int years;
byte months, days, hour, minutes, second, hundredths;

//Speed Variables


int floatindex; //Declare the current place in the speedarray[]
int lastservomove;
float altlast; //Initialise last recorded altitude variable
boolean haslaunched = false;
#define MOVINGAVG 10//Define size of moving average

//Pre-Flight Variables
#define arraysize 6 //Define size of flight altitude array
unsigned long cutdowntimer = 0;
unsigned long cutdownon = 0;
unsigned long launchtimer = 0;
const float cutdownalt = 20000.0;
const unsigned long floattime = 1800000; //Half an hour in millis


//Radio variables
byte debugmsg = 0; //[8:cutdown|7:valvestate|6:geofence|5:timefence|4:floatlimit|3:speedlimit|2:gpsfix|1:launched]
#define msg_cutdown 1<<7
#define msg_valvestate 1<<6
#define msg_geofence 1<<5
#define msg_timefence 1<<4
#define msg_floatlimit 1<<3
#define msg_speedlimit 1<<2
#define msg_gpsfix 1<<1
#define msg_launched 1<<0

uint16_t crc;
unsigned int packetNum = 0;

//Pressure Sensor Variables
unsigned int sensorValue; //Integer value for ADC reading from pressure sensor
int pressure; //Integer value for pressure given by the mapping formula below
boolean cardavailable;

//battery voltages
int v_in;
int vs_in;

void setup()
{
  
  analogReference(INTERNAL); //Use internal 1.1V reference voltage
  ADMUX = 0xC8;
  delay(10);
  //mySerial.begin(9600);
  Serial.begin(4800);
  DEBUG_PRINTLN("Initialising....");
  pinMode(P_RADIO_TXD, OUTPUT);
  pinMode(P_RADIO_EN, OUTPUT);
  pinMode(P_SERVO_EN, OUTPUT);
  pinMode(P_LDO_EN, OUTPUT);
  pinMode(P_SD_CS, OUTPUT);
  digitalWrite(P_RADIO_EN,LOW);
  digitalWrite(P_LDO_EN,HIGH);
  // see if the card is present and can be initialized:
  if (!SD.begin(P_SD_CS)) {
    DEBUG_PRINTLN("Card failed, or not present");
    cardavailable = false;
    // don't do anything more:
  } 
  else {
    DEBUG_PRINTLN("card initialized.");
    cardavailable = true;
  }
  Wire.begin();
  delay(10);
  //Wire.beginTransmission(0x00); These 3 lines reset all registers
  //Wire.write(0x06); 
  //Wire.endTransmission();
  
  //vservo.attach(P_SERVO_DATA);          // attaches the servo on pin 9 to the servo object 
  //vservo.write(servoClosed);                  // sets the servo position according to the scaled value 
  delay(15);                           // waits for the servo to get there 
  
  
}


void loop()
{
  digitalWrite(P_RADIO_EN,HIGH);
  getgps();
  getvtg();
  readpressure();
  valvecontrol();
  transmit();
  logit();
  DEBUG_PRINTLN();
  DEBUG_PRINT(lat);
  DEBUG_PRINT(lon);
  DEBUG_PRINTLN(alt);

}     

void getgps() {
  Serial.println("$EIGPQ,GGA*27");
  while (Serial.available())
  {
    int c = Serial.read();
    DEBUG_WRITE(c); 
    DEBUG_SD_WRITE(c);
    if (gps.encode(c))
    {
      logchar('!');
      gps.get_position(&lat, &lon, &fix_age);
      gps.f_get_position(&f_lat, &f_lon, &fix_age);
      alt = gps.altitude();
      f_alt = gps.f_altitude();
      gps.crack_datetime(&years, &months, &days,
      &hour, &minutes, &second, &hundredths, &fix_age);
      DEBUG_PRINTLN();
    }
  }
  delay(10);
  Serial.println("$EIGPQ,RMC*3A");
  while (Serial.available())
  {
    int c = Serial.read();
    DEBUG_WRITE(c);
    DEBUG_SD_WRITE(c);
    if (gps.encode(c))
    {
      logchar('!');
      gps.get_position(&lat, &lon, &fix_age);
      gps.f_get_position(&f_lat, &f_lon, &fix_age);
      f_alt = gps.f_altitude();
      alt = gps.altitude();
      gps.crack_datetime(&years, &months, &days,
      &hour, &minutes, &second, &hundredths, &fix_age);
      DEBUG_PRINTLN();
    }
  }
}

void cutdown(boolean state)
{
  pinMode(P_CUTDOWN,OUTPUT);
  if (state) {
    if (cutdownon == 0) {
  digitalWrite(P_CUTDOWN,HIGH);
  cutdownon = millis();
    }
  } else {
  digitalWrite(P_CUTDOWN,LOW);
  }
  
  if ((millis() - cutdownon > 600000) && (digitalRead(P_CUTDOWN) == HIGH)) {
    digitalWrite(P_CUTDOWN,LOW);
    formatdebug(( msg_cutdown ),false);
    cutdownon = 0;
  }
  

}

void valvecontrol() 
{

  if (haslaunched) {
    if (fix_age < 1800000) { //If the last GPS lock was less than half an hour ago
      
      if ((lat < 5223) && (lon < 20) && (lon > 1) && (lat > 4000)) {
        if ((fix_age > 60000) && (speedavg() > 1)) // close the valve until a new gps position is gained
        {
          servopos(servoClosed);
          formatdebug(( msg_gpsfix ),true); 
        } 
        else {
          formatdebug(( msg_gpsfix ),false); 
          if (speedavg() > flightplan())
          {
            servopos(servoOpen);
            formatdebug(( msg_speedlimit ),true);
          } 
          else {
            servopos(servoClosed);
            formatdebug(( msg_speedlimit ),false);
          }
        }
      } 
      else {
        cutdown(true);
        formatdebug(( msg_cutdown | msg_geofence ),true);
        servopos(servoOpen);
      }
    } 
    else {
      cutdown(true);
      formatdebug(( msg_cutdown | msg_gpsfix ),true);
      servopos(servoOpen); //open the valve to dump helium
    }
  }
  else if(alt > 500) {
    haslaunched = true;
    launchtimer = millis();
    formatdebug(( msg_launched ),true);
  }
  else if(millis() - lastservomove > 100000) { //Make the servo close regularly until launch (as it is plugged in after filling)
    lastservopos = servoClosed - 1; //Make the system think the valve needs to be moved
    servopos(servoClosed);  
  }
  if (cutdownalt <= f_alt) {
    if (cutdowntimer == 0) {
      cutdowntimer = millis();
    } else if (millis() >= (cutdowntimer + floattime)) {
      cutdown(true);
      formatdebug(( msg_cutdown | msg_floatlimit ),true);
    }
  }
  if (launchtimer > 14400000) //If 4 hours since launch
    cutdown(true);
    formatdebug(( msg_cutdown | msg_timefence ),true);
}

void formatdebug(byte mask,boolean val) {
  if(val){ //We want to change to a 1
    debugmsg = debugmsg | mask;
  } else { //We want to change to a 0
    mask = ~mask;
    debugmsg = debugmsg & mask;
  }
}

void servopos(int pos) {
  //digitalWrite(P_SERVO_EN,HIGH); //Turn the MOSFET on
  int x = 0;
  if (pos != lastservopos) //If the desired servo position has changed
    if (pos >= lastservopos) //If the servo position has increased
    {
      for (x = lastservopos; x <= pos; x++) //Slowly increase the servo's position
      {
        //vservo.write(x);
        delay(10);
      }
    }
    else //Else the servo position has decreased
  {
    for (x = lastservopos; x >= pos; x--) //Slowly decrease the servo's position
    {
      //vservo.write(x);
      delay(10);
    }
  }
  delay(500);
  //digitalWrite(P_SERVO_EN,LOW); //Turn the MOSFET off
  lastservopos = pos; // write the new servo position to the register
  //if (pos == servoClosed) formatdebug(msg_valvestate,false);
  //if (pos == servoOpen) formatdebug(msg_valvestate,true);
}

int readTemperature()
{
  ADCSRA |= _BV(ADSC); // start the conversion
  while (bit_is_set(ADCSRA, ADSC)); // ADSC is cleared when the conversion finishes
  return (ADCL | (ADCH << 8)) - 342; // combine bytes & correct for temp offset (approximate)}
}

float averageTemperature()
{
  analogReference(INTERNAL); //Use internal 1.1V reference voltage
  ADMUX = 0xC8;
  readTemperature(); // discard first sample (never hurts to be safe)
  float averageTemp; // create a float to hold running average
  for (int i = 1; i < 100; i++) // start at 1 so we dont divide by 0
    averageTemp += ((readTemperature() - averageTemp)/(float)i); // get next sample, calculate running average
  //averageTemp *= 100;
  return averageTemp; // return average temperature reading
} 

void checkmem()
{
  DEBUG_PRINTLN( freeMemory() );
}

//from http://bildr.org/2011/01/tmp102-arduino/
float getextTemperature(){
  Wire.begin();
  /*delay(10);
  Wire.beginTransmission(tmp102Address);
  Wire.write(0x01);
  Wire.write(0x79);
  Wire.endTransmission();*/
  delay(20);
  Wire.requestFrom(tmp102Address,2); 

  byte MSB = Wire.read();
  byte LSB = Wire.read();

  //it's a 12bit int, using two's compliment for negative
  int TemperatureSum = ((MSB << 8) | LSB) >> 4; 
  if(TemperatureSum & (1<<11)) {
		TemperatureSum |= 0xF800; //Set bits 11 to 15 to 1s to get this reading into real twos compliment
  }
  float celsius = float(TemperatureSum)/16;
  return celsius;
}


//******************************************
//Program to calculate the speed based on a
//moving average array.
//******************************************
float speedavg(){
  float speedarray[20]; //Declare array to store speed values to create moving average
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
  int flightalt[] = {
  0,5000,10000,15000,20000,30000}; //Initialise variable array for altitude control points
  float flightspd[] = {
  30.0,5.0,4.0,2.0,0.0,0.0}; //Initialise variable array for controlled speeds in m/s
  for (int i = 0; i < arraysize; i++) //lookup which array element relates to the current altitude
  {
    if ( flightalt[i] <= f_alt) 
    {
      targetspeed = flightspd[i]; //looks up the target speed for this altitude
    }
  }
  return targetspeed; //returns the target speed to the calling function  
}

//void serialEvent(){
//  gps.encode(Serial.read());
//}

void transmit(){
  char packet[120];
  packetNum++;
  char slat[10], slon[10], salt[8], stemp[6],sint[6];
  //ftoa(slat,f_lat,8); 
  //ftoa(slon,f_lon,8);
  //f_lat *= 100;
  //f_lon *= 100;
  fmtDouble(f_lat,6,slat,10);
  fmtDouble(f_lon,6,slon,10);
  fmtDouble(f_alt,6,salt,8);
  fmtDouble(getextTemperature(),2,stemp,6);
  fmtDouble(averageTemperature(),2,sint,6);
  //int result = sprintf(packet,"$$ALTI,%u,%02u:%02u:%02u,%s,%s,%s,%d,%d,%d,%s,%s,%X,%u*",packetNum,hour,minutes,second,slat,slon,salt,pressure,v_in,vs_in,sint,stemp,debugmsg,freeMemory());
  int result = sprintf(packet,"$$ALTI,%u,%02u:%02u:%02u,%s,%s,%s,%d,%d,%d,%s,%s,%X*",packetNum,hour,minutes,second,slat,slon,salt,pressure,v_in,vs_in,sint,stemp,debugmsg);
  crc = (CRC16(&packet[3]));
  result = sprintf(&packet[result],"%04X\n",crc);
  //delay(1000);
  rtty_preamble(1);
  rtty_tx(packet,1);
  //set timers
  //return
}

void readpressure() {
  sensorValue = analogRead(A0);
  sensorValue = 0;
  for (int i = 0; i < 5; i++) {
    delay(1);
    sensorValue += analogRead(A0);
  }
  sensorValue /= 5;
  pressure = map(sensorValue, 4, 927, -2000, 2000);  //Map ADC to pressure
  pressure -= 45;
}

//******************************************
//Sends string of values to the openLog sys
//-tem. 
//******************************************
void logit() {
  //char temp[10];
  if (cardavailable) {
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      //dataFile.print(packet);
      dataFile.print(",Lat:,");
      dataFile.print(lat);
      dataFile.print(",Lon:, ");
      dataFile.print(lon);
      dataFile.print(",Alt:,");
      dataFile.println(alt);
      dataFile.print(",Pressure Raw:,");
      dataFile.print(sensorValue);
      dataFile.print(",Pressure:,");
      dataFile.print(pressure);
      dataFile.print(",Raw Main Battery Voltage,");
      dataFile.print(v_in);
      dataFile.print(",Raw Servo Battery Voltage,");
      dataFile.print(vs_in);
      dataFile.println();
      dataFile.close();
      // print to the serial port too:
    }  
    // if the file isn't open, pop up an error:
    else {
      DEBUG_PRINTLN("error opening datalog.txt");
    } 

    //int timenow = millis();
  }
}


void logchar(char data) {
  if (cardavailable) {
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.print(data);
      dataFile.close();
      // print to the serial port too:
    }  
    // if the file isn't open, pop up an error:
    else {
      DEBUG_PRINTLN("error opening datalog.txt");
    } 

    //int timenow = millis();
  }
}

void rtty_tx(char* sentence, int baud)
{
  // Disable interrupts
  //noInterrupts();

  int i=0;
  while(sentence[i] != 0)
  {
    rtty_tx_byte(sentence[i], baud);
    i++;
  }

  // Re-enable interrupts
  //interrupts();
}

void rtty_preamble(int baud)
{
  char sentence[] = "UUUUUUUU\r\n";

  // Disable interrupts
  //noInterrupts();

  int i=0;
  while(sentence[i] != 0)
  {
    rtty_tx_byte(sentence[i], baud);
    i++;
  }

  // Re-enable interrupts
  //interrupts();
}

void rtty_tx_byte(char c, int baud)
{
  // Start bit
  rtty_tx_bit(0, baud);

  // Send byte
  for(int b=0; b<8; b++)
  {
    if(c & 1)
    {
      rtty_tx_bit(1, baud);
    }
    else
    {
      rtty_tx_bit(0, baud);
    }

    c = c >> 1;
  }

  // 2 Stop bits
  rtty_tx_bit(1, baud);
  rtty_tx_bit(1, baud);
}

void rtty_tx_bit(int b, int baud)
{
  if(b)
  {
    // If HIGH
    digitalWrite(P_RADIO_TXD,HIGH);
  }
  else
  {
    // If LOW
    digitalWrite(P_RADIO_TXD,LOW);      
  }

  if(baud == 1)
  {
    // 300 baud
    delayMicroseconds(3370);
  }
  else if(baud == 0)
  {
    // 50 baud
    delayMicroseconds(10000);
    delayMicroseconds(10150);
  }
  else
  {
    // Otherwise default to 50 baud
    delayMicroseconds(10000);
    delayMicroseconds(10150);
  }
}

void getvtg() {
  v_in = analogRead(V_MAIN);
  vs_in = analogRead(V_SERVO);
}


void fmtDouble(double val, byte precision, char *buf, unsigned bufLen = 0xffff);
unsigned fmtUnsigned(unsigned long val, char *buf, unsigned bufLen = 0xffff, byte width = 0);

//
// Produce a formatted string in a buffer corresponding to the value provided.
// If the 'width' parameter is non-zero, the value will be padded with leading
// zeroes to achieve the specified width.  The number of characters added to
// the buffer (not including the null termination) is returned.
//
unsigned
fmtUnsigned(unsigned long val, char *buf, unsigned bufLen, byte width)
{
  if (!buf || !bufLen)
    return(0);

  // produce the digit string (backwards in the digit buffer)
  char dbuf[10];
  unsigned idx = 0;
  while (idx < sizeof(dbuf))
  {
    dbuf[idx++] = (val % 10) + '0';
    if ((val /= 10) == 0)
      break;
  }

  // copy the optional leading zeroes and digits to the target buffer
  unsigned len = 0;
  byte padding = (width > idx) ? width - idx : 0;
  char c = '0';
  while ((--bufLen > 0) && (idx || padding))
  {
    if (padding)
      padding--;
    else
      c = dbuf[--idx];
    *buf++ = c;
    len++;
  }

  // add the null termination
  *buf = '\0';
  return(len);
}

//
// Format a floating point value with number of decimal places.
// The 'precision' parameter is a number from 0 to 6 indicating the desired decimal places.
// The 'buf' parameter points to a buffer to receive the formatted string.  This must be
// sufficiently large to contain the resulting string.  The buffer's length may be
// optionally specified.  If it is given, the maximum length of the generated string
// will be one less than the specified value.
//
// example: fmtDouble(3.1415, 2, buf); // produces 3.14 (two decimal places)
//
void
fmtDouble(double val, byte precision, char *buf, unsigned bufLen)
{
  if (!buf || !bufLen)
    return;

  // limit the precision to the maximum allowed value
  const byte maxPrecision = 6;
  if (precision > maxPrecision)
    precision = maxPrecision;

  if (--bufLen > 0)
  {
    // check for a negative value
    if (val < 0.0)
    {
      val = -val;
      *buf = '-';
      bufLen--;
      buf++;
    }

    // compute the rounding factor and fractional multiplier
    double roundingFactor = 0.5;
    unsigned long mult = 1;
    for (byte i = 0; i < precision; i++)
    {
      roundingFactor /= 10.0;
      mult *= 10;
    }

    if (bufLen > 0)
    {
      // apply the rounding factor
      val += roundingFactor;

      // add the integral portion to the buffer
      unsigned len = fmtUnsigned((unsigned long)val, buf, bufLen);
      buf += len;
      bufLen -= len;
    }

    // handle the fractional portion
    if ((precision > 0) && (bufLen > 0))
    {
      *buf++ = '.';
      if (--bufLen > 0)
        buf += fmtUnsigned((unsigned long)((val - (unsigned long)val) * mult), buf, bufLen, precision);
    }
  }

  // null-terminate the string
  *buf = '\0';
}
