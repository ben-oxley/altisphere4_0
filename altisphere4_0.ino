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
