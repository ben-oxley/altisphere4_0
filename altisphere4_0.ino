// ___________________________
//|        Ben Oxley          |
//|AltiSphere Pre Release Code|
//|___________________________|
//Must initalise some variables on day of flight, may not work over midnight

//FIX: The release notes includes a handy pre-compiler directive to check of the arduino flavor you are using.

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

/*
 Output  0 - RXI                        Output  7 - SERVO DATA
 Output  1 - TXO                        Output  8 - DRIVE ENABLE COMMS
 Output  2 - N/C                        Output  9 - DRIVE COMMS
 Output  3 - N/C                        Output 10 - N/C
 Output  4 - MOSFET ON                  Output 11 - MOSI
 Output  5 - NOT RECEIEVE ENABLE COMMS  Output 12 - MISO
 Output  6 - RECIEVE COMMS              Output 13 - SCK
 
 Servo Goes between 18 and 90 Degrees
 
 Analog 0 - N/C
 Analog 1 - N/C
 Analog 2 - Pressure Sensor   Vout = VS × (0.2 × P(kPa)+0.5) ± 6.25% VFSS  Voltage is divided by two. Range 0.25-->2.25v Gives a resolution of 6 Pa (4000/(2/(3.3/1024)))
 Analog 3 - N/C
 Analog 4 - Temperature Sensors SDA
 Analog 5 - Temperature Sensors SCL
 Analog 6 - N/C
 Analog 7 - N/C 
 
 Ideal analog divider is 10K and 27K   730, 378, 3
 */
 
/*
  GPS Level Convertor Board Test Script
  03/05/2012 2E0UPU
 
  Initialise the GPS Module in Flight Mode and then echo's out the NMEA Data to the Software Serial.
 
  This example code is in the public domain.
  Additional Code by J Coxon (http://ukhas.org.uk/guides:falcom_fsa03)
 */
 
#include <SoftwareSerial.h>
 
SoftwareSerial mySerial(4, 5);
 
byte gps_set_sucess = 0 ;
 
void setup()
{
  mySerial.begin(9600);
  Serial.begin(9600);
  mySerial.println("Initialising....");
 
 // THIS COMMAND SETS FLIGHT MODE AND CONFIRMS IT 
  mySerial.println("Setting uBlox nav mode: ");
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                      };
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
  }
  gps_set_sucess=0;
 
  // THE FOLLOWING COMMANDS DO WHAT THE $PUBX ONES DO BUT WITH CONFIRMATION
   debug.println("Switching off NMEA GLL: ");
   uint8_t setGLL[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B                   };
   while(!gps_set_sucess)
   {		
   sendUBX(setGLL, sizeof(setGLL)/sizeof(uint8_t));
   gps_set_sucess=getUBX_ACK(setGLL);
   }
   gps_set_sucess=0;
 
   debug.println("Switching off NMEA GSA: ");
   uint8_t setGSA[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32                   };
   while(!gps_set_sucess)
   {	
   sendUBX(setGSA, sizeof(setGSA)/sizeof(uint8_t));
   gps_set_sucess=getUBX_ACK(setGSA);
   }
   gps_set_sucess=0;
   debug.println("Switching off NMEA GSV: ");
   uint8_t setGSV[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39                   };
   while(!gps_set_sucess)
   {
   sendUBX(setGSV, sizeof(setGSV)/sizeof(uint8_t));
   gps_set_sucess=getUBX_ACK(setGSV);
   }
   gps_set_sucess=0;
   debug.print("Switching off NMEA RMC: ");
   uint8_t setRMC[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40                   };
   while(!gps_set_sucess)
   {
   sendUBX(setRMC, sizeof(setRMC)/sizeof(uint8_t));
   gps_set_sucess=getUBX_ACK(setRMC);
   }
   gps_set_sucess=0;
   debug.print("Switching off NMEA VTG: ");
   uint8_t setVTG[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46                   };
   while(!gps_set_sucess)
   {
   sendUBX(setVTG, sizeof(setRMC)/sizeof(uint8_t));
   gps_set_sucess=getUBX_ACK(setVTG);
 
   }
   
 
}
 
void loop()
{
    Serial.println("$PUBX,00*33");
    if(Serial.available() > 0) 
    {
      char inByte = Serial.read();
      mySerial.write(inByte);
 
    }
 

}     
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
    mySerial.print(MSG[i], HEX);
  }
  Serial.println();
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
