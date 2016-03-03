////////////////HEADER FILES FOR I2C AND MPU////////////////
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
////////////////////////////////////////////////////////////




/////////MPU OBJECT/////////
MPU6050 mpu;
////////////////////////////




///////////////////////////VARIABLES///////////////////////////////////////////////////////////

long int start,stopp,timer,value;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

///////////////////////////////////////////////////////////////////////////////////////////////////////





///////////////MPU INTERRUPT ROUTINE////////////////
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}
////////////////////////////////////////////////////





/////////////////////SETTING STUFF UP/////////////////////////////////////////////////////////////////////////////////////////
void setup() {


  //////////////INITIAL1ZING COMMUNICATIONS////////////
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
	#endif
	Serial.begin(9600);
  /////////////////////////////////////////////////////

	
	
	
	
	///////////MPU STUFF////////////////////
	mpu.initialize();
	if(!mpu.testConnection())
	exit(0);
	devStatus=mpu.dmpInitialize();
  mpu.setXGyroOffset(0);
	mpu.setYGyroOffset(0);
	mpu.setZGyroOffset(0);
	mpu.setXAccelOffset(0);
	mpu.setYAccelOffset(0);
	mpu.setZAccelOffset(0);
  if (devStatus == 0) 
	{  
		mpu.setDMPEnabled(true);
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();
		dmpReady = true;
		packetSize=mpu.dmpGetFIFOPacketSize();
	} 
  /////////////////////////////////////////
  
  
  
  
  
  ////////////////PORT INITIALIZATION//////////////////
	DDRD|=B11011000;
	DDRB|=B00110000;
	PORTB|=B00110000;
  ////////////////////////////////////////////////////
	
	
	
	
	
	////////////////ACTIVATING ESCS/////////////////
	value=2400;
	timer=millis();
	while((millis()-timer)<10000)
	{
		start=micros();
		PORTD|=B11011000;
		while((micros()-start)<value);
		PORTD&=B00000011;
		while(micros()-start<50000);
	} 
	timer=millis();
	value=600;
	while(millis()-timer<3000)
	{
		start=micros();
		PORTD|=B11011000;
		while((micros()-start)<value);
		PORTD&=B00000011;
		while(micros()-start<20000);
	} 
  ////////////////////////////////////////////////
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////FEEDBACK LOOP @50Hz//////////////////////////////////////////////////////////
void loop()
{
	if(Serial.available())
	{
	}

	value=2000;
	start=micros();
	PORTD|=B11011000;
	while((micros()-start)<value);
	PORTD&=B0000011;
	while(micros()-start<20000);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





//////////////////////////Subroutine to Calculate PID Controller Correction Values////////////////////////
void pid()
{

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
