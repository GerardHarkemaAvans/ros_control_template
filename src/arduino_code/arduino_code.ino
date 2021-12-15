// the sensor communicates using SPI, so include the library:
#include <SPI.h>
#include "ls7366r.h"

#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
/*
 *         UNO     MEGA
 MOSI: pin 11       51
 MISO: pin 12       59
 SCK:  pin 13       52
*/

int chipSelectPin=2;
//////////////////////////////////////////////

#define M1              9                       // PWM outputs to motor driver module
#define M2              10

#define COUNTS_PER_PIRAD       1800 
#define RESOLUTION              (float)(180.0/COUNTS_PER_PIRAD) 


ros::NodeHandle  nh;

float pos = 0, vel= 0, output = 0, last_pos=0;
unsigned long lastTime,now;
std_msgs::Float64MultiArray joint_state;

void set_angle_cb( const std_msgs::Float64MultiArray& cmd_msg){
  output= cmd_msg.data[0]; 
}



ros::Subscriber<std_msgs::Float64MultiArray> sub("joints_to_aurdino", set_angle_cb);
ros::Publisher pub("joint_states_from_arduino", &joint_state);


long encoderValue;
float angle; 

float joint_state_data[2];


void setup(){

  pwmOut(0);
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  LS7366_Init();

  delay(100);

  // allocate memory for joints states
  joint_state.data = joint_state_data;


  encoderValue = getEncoderValue(); 
  last_pos =  (encoderValue % COUNTS_PER_PIRAD) * RESOLUTION;
  lastTime = millis();


}

 

void loop(){

#if 1
  pwmOut(output);
#endif

#if 1
  now = millis();
  if ((now - lastTime)> 100)
  {

    encoderValue = getEncoderValue(); 
    pos =  (encoderValue % COUNTS_PER_PIRAD) * RESOLUTION;
    vel = (pos - last_pos) / (now - lastTime);

    
    joint_state.data_length=2;
    joint_state.data[0]=(float)pos;
    joint_state.data[1]=vel;

    
    pub.publish(&joint_state);
    lastTime=now;
  }
#endif


  nh.spinOnce();
  //delay(10);
}


void pwmOut(float out) {                                
  if (out > 0) {
    analogWrite(M2, out);                             // drive motor CW
    analogWrite(M1, 0);
  }
  else {
    analogWrite(M2, 0);
    analogWrite(M1, abs(out));                        // drive motor CCW
  }
}

//*****************************************************  
long getEncoderValue(void)
//*****************************************************
{
    unsigned int count1Value, count2Value, count3Value, count4Value;
    long result;
    
    digitalWrite(chipSelectPin,LOW);


    SPI.transfer(READ_CNTR); // Request count
    //SPI.transfer(0x63); 
    count1Value = SPI.transfer(0x00); // Read highest order byte
    count2Value = SPI.transfer(0x00);
    count3Value = SPI.transfer(0x00);
    count4Value = SPI.transfer(0x00); // Read lowest order byte
    
    digitalWrite(chipSelectPin,HIGH);
   
    result= ((long)count1Value<<24) + ((long)count2Value<<16) + ((long)count3Value<<8) + (long)count4Value;
    
    return result;
}//end func


// LS7366 Initialization and configuration
//*************************************************
void LS7366_Init(void)
//*************************************************
{
   
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);

    
  // SPI initialization
  SPI.begin();
  //SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
  delay(10);
  
  digitalWrite(chipSelectPin,LOW);
  SPI.transfer(WRITE_MDR0); 
  //SPI.transfer(0x03); 
  SPI.transfer(QUADRX4|FREE_RUN);//|INDX_LOADC|SYNCH_INDX|FILTER_2);
  digitalWrite(chipSelectPin,HIGH); 
   
   
}//end func
