/**
 * Uses the MCPDAC library from http://hacking.majenko.co.uk/MCPDAC
 *
 */

// MCPDAC relies on SPI.
#include <SPI.h>
#include <MCPDAC.h>

void setup()
{
  // CS on pin 10, no LDAC pin (tie it to ground).
  MCPDAC.begin(10);
  
  // Set the gain to "HIGH" mode - 0 to 4096mV.
  MCPDAC.setGain(CHANNEL_A,GAIN_HIGH);
  
  // Set the gain to "HIGH" mode - 0 to 4096mV.
  MCPDAC.setGain(CHANNEL_B,GAIN_HIGH);
  
  // Do not shut down channels
  MCPDAC.shutdown(CHANNEL_A,false);
  MCPDAC.shutdown(CHANNEL_B,false);
}

void loop()
{
  static unsigned int A_mv;
  static unsigned int B_mv;
  
  A_mv = 4095;
  
  B_mv = 1000;
  
  // Set the voltage of channel A.
  MCPDAC.setVoltage(CHANNEL_A, A_mv&0x0fff);
  
  // Set the voltage of channel B.
  MCPDAC.setVoltage(CHANNEL_B, B_mv&0x0fff);

}

