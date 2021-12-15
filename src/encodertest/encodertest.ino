//Robogaia.com
// the sensor communicates using SPI, so include the library:
#include <SPI.h>
#include "ls7366r.h"

/*
 MOSI: pin 11
 MISO: pin 12
 SCK: pin 13
*/

int chipSelectPin1=2;
//////////////////////////////////////////////


#define COUNTS_PER_2PIRAD       3800 
#define RESOLUTION              (360.0/COUNTS_PER_2PIRAD)       

//*****************************************************
void setup() 
//*****************************************************
{
  Serial.begin(9600);



  LS7366_Init();
  delay(100);
}

//*****************************************************
void loop() 
//*****************************************************
{
    long encoder1Value;
    double angle;
         
    
    encoder1Value = getEncoderValue(); 
    angle =  (encoder1Value % COUNTS_PER_2PIRAD) * RESOLUTION;
    Serial.print("Encoder value = ");
    Serial.print(encoder1Value);
    Serial.print(",angle = ");
    Serial.print(angle);
    Serial.print("\r\n");

   delay(100); 
 
}//end loop



  
//*****************************************************  
long getEncoderValue(void)
//*****************************************************
{
    unsigned int count1Value, count2Value, count3Value, count4Value;
    long result;
    
    digitalWrite(chipSelectPin1,LOW);


    SPI.transfer(READ_CNTR); // Request count
    //SPI.transfer(0x63); 
    count1Value = SPI.transfer(0x00); // Read highest order byte
    count2Value = SPI.transfer(0x00);
    count3Value = SPI.transfer(0x00);
    count4Value = SPI.transfer(0x00); // Read lowest order byte
    
    digitalWrite(chipSelectPin1,HIGH);
   
    result= ((long)count1Value<<24) + ((long)count2Value<<16) + ((long)count3Value<<8) + (long)count4Value;
    
    return result;
}//end func



// LS7366 Initialization and configuration
//*************************************************
void LS7366_Init(void)
//*************************************************
{
   
  pinMode(chipSelectPin1, OUTPUT);
  digitalWrite(chipSelectPin1, HIGH);

    
  // SPI initialization
  SPI.begin();
  //SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
  delay(10);
  
  digitalWrite(chipSelectPin1,LOW);
  SPI.transfer(WRITE_MDR0); 
  //SPI.transfer(0x03); 
  SPI.transfer(QUADRX4|FREE_RUN);//|INDX_LOADC|SYNCH_INDX|FILTER_2);
  digitalWrite(chipSelectPin1,HIGH); 
   
   
}//end func
