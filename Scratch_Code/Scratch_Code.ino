////////////////HEADER FILES FOR I2C AND MPU////////////////1
#include<String.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
////////////////////////////////////////////////////////////




/////////MPU OBJECT////2
MPU6050 mpu;
////////////////////////////





//////////////////////////////PID gain and limit settings////////////////////////////////////////

float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.3;              //Gain setting for the roll I-controller (0.3)
float pid_d_gain_roll = 15;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 1;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.1;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

////////////////////////////////////////////////////////////////////////////////////////





///////////////////////////VARIABLES///////////////////////////////////////////////////////////3

long int start, stopp, timer, value;
String buffera;
char * chara;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifobuffer[64]; // FIFO storage buffera

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// PID variables
float pid_error_temp;
float pid_i_mem_roll=0, pid_roll_setpoint=0, dmp_roll_input=0, pid_output_roll=0, pid_last_roll_d_error=0;
float pid_i_mem_pitch=0, pid_pitch_setpoint=0, dmp_pitch_input=0, pid_output_pitch=0, pid_last_pitch_d_error=0;
float pid_i_mem_yaw=0, pid_yaw_setpoint=0, dmp_yaw_input=0, pid_output_yaw=0, pid_last_yaw_d_error=0;
///////////////////////////////////////////////////////////////////////////////////////////////////////




/////////////////////////////////MPU INTERRUPT ROUTINE//////////////////////////////////////////4
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////





/////////////////////SETTING STUFF UP/////////////////////////////////////////////////////////////////////////////////////////
void setup() {
	pid_roll_setpoint=0;
	pid_pitch_setpoint=0;
	pid_yaw_setpoint=0;


	//////////////INITIAL1ZING COMMUNICATIONS////////////5
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif
	Serial.begin(115200);
	/////////////////////////////////////////////////////





	///////////MPU STUFF////////////////////6
	mpu.initialize();
	if (!mpu.testConnection())
	exit(0);
	devStatus = mpu.dmpInitialize();
	if (devStatus == 0)
	{
		mpu.setDMPEnabled(true);
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();
		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	/////////////////////////////////////////





	////////////////PORT INITIALIZATION//////////////////7
	DDRD |= B11011000;
	DDRB |= B00110000;
	PORTB |= B00110000;
	////////////////////////////////////////////////////





	/*///////////////ACTIVATING ESCS/////////////////8
value = 2400;
timer = millis();
while ((millis() - timer) < 10000)
{
	start = micros();
	PORTD |= B11011000;
	while ((micros() - start) < value);
	PORTD &= B00000011;
	while (micros() - start < 50000);
}
timer = millis();
value = 600;
while (millis() - timer < 3000)
{
	start = micros();
	PORTD |= B11011000;
	while ((micros() - start) < value);
	PORTD &= B00000011;
	while (micros() - start < 20000);
}
*////////////////////////////////////////////////
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void pid();
///////////////////////////////////////FEEDBACK LOOP @50Hz//////////////////////////////////////////////////////////
void loop()
{




	if (!dmpReady) return;
	while (!mpuInterrupt && fifoCount < packetSize);
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));
	} else if (mpuIntStatus & 0x02) {
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
		mpu.getFIFOBytes(fifobuffer, packetSize);
		fifoCount -= packetSize;
	}
	/*////////////////////////Recieve Wireless Data//////////////////////9
if (Serial.available())
{
	buffera = Serial.read();
	buffera.toCharArray(chara,20);
	if (*chara == 'y')
	{
	chara++;
	buffera = chara;
	pid_yaw_setpoint = buffera.toFloat();
	}
	else if (*chara == 'p')
	{
	chara++;
	buffera = chara;
	pid_pitch_setpoint = buffera.toFloat();
	}
	else if (*chara == 'r')
	{
	chara++;
	buffera = chara;
	pid_roll_setpoint = buffera.toFloat();
	}
	else if (*chara == 'a')
	{
	chara++;
	buffera = chara;
	value = buffera.toFloat();
	}
}
else
{
	if (pid_pitch_setpoint > 10)
	pid_pitch_setpoint /= 2;
	else
	pid_pitch_setpoint = 0;
	if (pid_roll_setpoint > 10)
	pid_roll_setpoint /= 2;
	else
	pid_roll_setpoint = 0;
	if (pid_yaw_setpoint > 10)
	pid_yaw_setpoint /= 2;
	else
	pid_yaw_setpoint = 0;
}
*////////////////////////////////////////////////////////////////////////





	////////////////////////GET DMP DATA//////////////////////////10
	mpu.dmpGetQuaternion(&q, fifobuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	///////////////////////////////////////////////////////////////////





	//////////////////////CALL PID/////////////////////////////////////11
	pid();
	/////////////////////////////////////////////////////////////////////





	/*/////////////////////ESC BITBANGING//////////////////////////12
value = 2000;
start = micros();
PORTD |= B11011000;
while ((micros() - start) < value);
PORTD &= B0000011;
while (micros() - start < 20000);
*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}




//////////////////////////Subroutine to Calculate PID Controller Correction Values///////////////13
void pid()
{
	dmp_roll_input = ypr[2] ;
	dmp_pitch_input = ypr[1] ;
	dmp_yaw_input = ypr[0] ;

	//Roll calculations
	pid_error_temp = dmp_roll_input - pid_roll_setpoint;
	pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
	if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
	else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

	pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
	if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
	else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

	pid_last_roll_d_error = pid_error_temp;
	Serial.print(pid_output_roll,DEC);Serial.print('\t');Serial.print(ypr[2]*180/M_PI,DEC);Serial.print('\t');
	//Pitch calculations
	pid_error_temp = dmp_pitch_input - pid_pitch_setpoint;
	pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
	if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
	else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

	pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
	if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
	else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

	pid_last_pitch_d_error = pid_error_temp;
	Serial.print(pid_output_pitch,DEC);Serial.print('\t');Serial.print(ypr[1]*180/M_PI,DEC);Serial.print('\t');
	//Yaw calculations
	pid_error_temp = dmp_yaw_input - pid_yaw_setpoint;
	pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
	if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
	else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

	pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
	if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
	else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

	pid_last_yaw_d_error = pid_error_temp;
	Serial.print(pid_output_yaw,DEC);Serial.println('\t');Serial.print(ypr[0]*180/M_PI,DEC);Serial.print('\t');
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
