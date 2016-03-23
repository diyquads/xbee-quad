////////////////HEADER FILES FOR I2C AND MPU////////////////1
#include"I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
float THROTTLE_MAX =1800;
float THROTTLE_MIN =1000;
////////////////////////////////////////////////////////////

#include<Servo.h>
Servo m1,m2,m3,m4;
boolean blinkState;

/////////MPU OBJECT////2
MPU6050 mpu;
////////////////////////////





///////////////////////////VARIABLES///////////////////////////////////////////////////////////3

long int a,b,c,start,start1,stopp, timer,value=1000,valuefr=0,valuebr=0,valuebl=0,valuefl=0,count;
float change=0.01,smoothx,smoothy,smoothz;
char chara,set='p';
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffera
int16_t ax,ay,az,gx,gy,gz;
 
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

//PIDrate variables
float gyro_x_input,gyro_y_input,gyro_z_input;
float pidrate_error_temp;
float pidrate_i_mem_roll=0, pidrate_roll_setpoint=0, pidrate_output_roll=0, pidrate_last_roll_d_error=0;
float pidrate_i_mem_pitch=0, pidrate_pitch_setpoint=0, pidrate_output_pitch=0, pidrate_last_pitch_d_error=0;
float pidrate_i_mem_yaw=0, pidrate_yaw_setpoint=0, pidrate_output_yaw=0, pidrate_last_yaw_d_error=0;


//PID gain and limit settings
float pid_p_gain_roll = 15;               //Gain setting for the roll P-controller (150)
float pid_i_gain_roll =0;              //Gain setting for the roll I-controller (1)
float pid_d_gain_roll =0;                //Gain setting for the roll D-controller (750
int pid_max_roll = 10000;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = 15;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 0;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 10000;                     //Maximum output of the PID-controller (+/-)
///////////////////////////////////////////////////////////////////////////////////////////////////////

//PIDrate gain and limit settings
float pidrate_p_gain_roll =0.09;               //Gain setting for the roll P-controller (150)
float pidrate_i_gain_roll =0;              //Gain setting for the roll I-controller (1)
float pidrate_d_gain_roll =15;                //Gain setting for the roll D-controller (750
int pidrate_max_roll = 100;                    //Maximum output of the PID-controller (+/-)

float pidrate_p_gain_pitch =0.09;  //Gain setting for the pitch P-controller.
float pidrate_i_gain_pitch = 0;  //Gain setting for the pitch I-controller.
float pidrate_d_gain_pitch = 15;  //Gain setting for the pitch D-controller.
int pidrate_max_pitch = pidrate_max_roll;          //Maximum output of the PID-controller (+/-)

float pidrate_p_gain_yaw = .1;                //Gain setting for the pitch P-controller. //4.0
float pidrate_i_gain_yaw = 0;               //Gain setting for the pitch I-controller. //0.02
float pidrate_d_gain_yaw = 0;                //Gain setting for the pitch D-controller.
int pidrate_max_yaw = 100;
//////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////MPU INTERRUPT ROUTINE//////////////////////////////////////////4
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////





/////////////////////SETTING STUFF UP/////////////////////////////////////////////////////////////////////////////////////////
void setup() {
	
	m1.attach(3,1000,2000);
	m2.attach(4,1000,2000);
	m3.attach(6,1000,2000);
	m4.attach(7,1000,2000);
	
	
	
	
	//////////////INITIAL1ZING COMMUNICATIONS////////////5
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif
	//delay(9000);
	Serial.begin(57600);
	Serial.println(F("GET READY"));
	/////////////////////////////////////////////////////




	////////////////PORT INITIALIZATION//////////////////6
	DDRD |= B11011000;
	DDRB |= B00110001;
	PORTB |= B00110001;
	////////////////////////////////////////////////////

	////////////////ACTIVATING ESCS/////////////////8

	m1.write(180);
	m2.write(180);
	m3.write(180);
	m4.write(180);
	delay(2000);
	m1.write(0);
	m2.write(0);
	m3.write(0);
	m4.write(0);
	//delay(10000);

	///////////MPU STUFF////////////////////7
	mpu.initialize();
	Serial.println(F("Initializing MPU"));
	if (!mpu.testConnection())
	exit(0);
	Serial.println(F("MPU Connection Succesful"));
	devStatus = mpu.dmpInitialize();
	if (devStatus == 0)
	{
		mpu.setDMPEnabled(true);
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();
		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	//Your offsets: -1097 2131  2489  76  -82 9


	mpu.setXAccelOffset(-1072);
	mpu.setYAccelOffset(2170);
	mpu.setZAccelOffset(2602);
	mpu.setXGyroOffset(73);
	mpu.setYGyroOffset(-82);
	mpu.setZGyroOffset(14);
	/////////////////////////////////////////

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////





///////////////////////////////////////FEEDBACK LOOP @50Hz//////////////////////////////////////////////////////////
void loop()
{  

	if (!dmpReady) return;


	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize) {

		
		/////////////////////////Recieve Wireless Data//////////////////////9
		pid_roll_setpoint=0;
		pid_pitch_setpoint=0;
		pid_yaw_setpoint=0;

		if (Serial.available())
		{
			chara=Serial.read();
			if (chara == 'p' && set=='p')
			{
				pid_p_gain_roll+=change;
				pid_p_gain_pitch+=change;
				Serial.println(pid_p_gain_pitch,DEC);
			}
			else if (chara == 'p' && set=='r')
			{
				pidrate_p_gain_roll+=change;
				pidrate_p_gain_pitch+=change;
				Serial.println(pidrate_p_gain_pitch,DEC);
			}
			
			else if (chara == 'o'&&set=='p')
			{
				pid_p_gain_roll-=change;
				pid_p_gain_pitch-=change;
				Serial.println(pid_p_gain_pitch,DEC);
			}
			else if (chara == 'o'&& set=='r')
			{
				pidrate_p_gain_pitch-=change;
				pidrate_p_gain_roll-=change;
				Serial.println(pidrate_p_gain_pitch,DEC);
			}
			else if (chara == 'i'&&set=='p')
			{
				pid_i_gain_roll+=change;
				pid_i_gain_pitch+=change;
				Serial.println(pid_i_gain_pitch,DEC);
			}
			else if (chara == 'i'&&set=='r')
			{
				pidrate_i_gain_roll+=change;
				pidrate_i_gain_pitch+=change;
				Serial.println(pidrate_i_gain_pitch,DEC);
			}
			else if (chara == 'u'&&set=='p')
			{
				pid_i_gain_roll-=change;
				pid_i_gain_pitch-=change;
				Serial.println(pid_i_gain_pitch,DEC);
			}
			else if (chara == 'u'&&set=='r')
			{
				pidrate_i_gain_pitch-=change;
				pidrate_i_gain_roll-=change;
				Serial.println(pidrate_i_gain_pitch,DEC);
			}
			else if (chara == 'd'&&set=='p')
			{
				
				pid_d_gain_pitch+=change;
				pid_d_gain_roll+=change;
				Serial.println(pid_d_gain_pitch,DEC);
			}
			else if (chara == 'd'&&set=='r')
			{
				pidrate_d_gain_pitch+=change;
				pidrate_d_gain_roll+=change;
				Serial.println(pidrate_d_gain_pitch,DEC);
			}
			else if (chara == 's'&&set=='p')
			{
				pid_d_gain_roll-=change;
				pid_d_gain_pitch-=change;
				Serial.println(pid_d_gain_pitch,DEC);
			}
			else if (chara == 's'&&set=='r')
			{
				pidrate_d_gain_pitch-=change;
				pidrate_d_gain_roll-=change;
				Serial.println(pidrate_d_gain_pitch,DEC);
			}
			else if(chara=='a')
			{
				Serial.println(pid_p_gain_pitch,DEC);
				Serial.println(pid_i_gain_pitch,DEC);
				Serial.println(pid_d_gain_pitch,DEC);
				Serial.println(pidrate_p_gain_pitch,DEC);
				Serial.println(pidrate_i_gain_pitch,DEC);
				Serial.println(pidrate_d_gain_pitch,DEC);
			}
			else if(chara=='t')
			{
				value+=10;
				Serial.println(value,DEC);
			}
			else if(chara=='r')
			{
				value-=10;
				Serial.println(value,DEC);
			}
			else if(chara=='q')
			{
				value=1000;
				THROTTLE_MIN=1000;
				Serial.println("Emergency Shutdown Sequence Initiated");
				pid_p_gain_pitch=0;
				pid_i_gain_pitch=0;
				pid_d_gain_pitch=0;
				pidrate_p_gain_pitch=0;
				pidrate_i_gain_pitch=0;
				pidrate_d_gain_pitch=0;
			  pid_p_gain_roll=0;
        pid_i_gain_roll=0;
        pid_d_gain_roll=0;
        pidrate_p_gain_roll=0;
        pidrate_i_gain_roll=0;
        pidrate_d_gain_roll=0;
			  pid_pitch_setpoint=0;
        pid_roll_setpoint=0;
        pid_yaw_setpoint=0;
        
			}
			else if(chara=='z')
			{
				Serial.println(ypr[0]*180/PI);Serial.println(ypr[1]*180/PI);Serial.println(ypr[2]*180/PI);    
			}
			else if(chara=='y')
			{
				change+=(change/10);
				Serial.println(change,DEC);
			}
			else if(chara=='h')
			{
				change-=(change/10);
				Serial.println(change,DEC);
			}
			else if(chara=='m')
			{
				set='p';
				Serial.println("Setting STAB values");
			}
			else if(chara=='n')
			{
				set='r';
				Serial.println("Setting RATE values");
				
			}
			else if(chara=='w')
			{
				pid_i_mem_roll = 0;
				pid_last_roll_d_error = 0;
				pid_i_mem_pitch = 0;
				pid_last_pitch_d_error = 0;
				pid_i_mem_yaw = 0;
				pid_last_yaw_d_error = 0;
				pidrate_i_mem_roll = 0;
				pidrate_last_roll_d_error = 0;
				pidrate_i_mem_pitch = 0;
				pidrate_last_pitch_d_error = 0;
				pidrate_i_mem_yaw = 0;
				pidrate_last_yaw_d_error = 0;
				pid_pitch_setpoint=0;
				pid_roll_setpoint=0;
				pid_yaw_setpoint=0;
				
			}
     else if(chara=='b')
     {
      gyro_x_input+=100;
     }
     else if(chara=='v')
     {
      gyro_x_input-=100;
     }
     else if(chara=='f')
     {
      gyro_y_input+=100;
     }
     else if(chara=='c')
     {
      gyro_y_input-=100;
     }
		}
		/*else
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

		//////////////////////CALL PID/////////////////////////////////////11
		pid();
		pidrate();
		//Serial.print(pid_output_yaw);Serial.print('\t');Serial.print(pid_output_pitch);Serial.print('\t');Serial.println(pid_output_roll);
		/////////////////////////////////////////////////////////////////////


		//////////////////////ESC//////////////////////////12
		//3=bl
		//4=br
		//6=fl
		//7=fr
		//clockwise yaw +ve
		//right roll +ve
		//backwards pitch +ve
   //ccw gyroz +ve
   //right gyrox +ve
   //forward gyroy +ve
		valuebr=(     -pidrate_output_yaw    +pidrate_output_roll     -pidrate_output_pitch   ) + value;
		if(valuebr<THROTTLE_MIN)valuebr=THROTTLE_MIN;if(valuebr>THROTTLE_MAX)valuebr=THROTTLE_MAX;
		valuebl=(     +pidrate_output_yaw     -pidrate_output_roll     -pidrate_output_pitch   ) +value;
		if(valuebl<THROTTLE_MIN)valuebl=THROTTLE_MIN;if(valuebl>THROTTLE_MAX)valuebl=THROTTLE_MAX;
		valuefr=(     +pidrate_output_yaw     +pidrate_output_roll     +pidrate_output_pitch   ) +value;
		if(valuefr<THROTTLE_MIN)valuefr=THROTTLE_MIN;if(valuefr>THROTTLE_MAX)valuefr=THROTTLE_MAX;
		valuefl=(     -pidrate_output_yaw     -pidrate_output_roll     +pidrate_output_pitch   ) +value;
		if(valuefl<THROTTLE_MIN)valuefl=THROTTLE_MIN;if(valuefl>THROTTLE_MAX)valuefl=THROTTLE_MAX;
		//Serial.print(valuebr);Serial.print('\t');Serial.print(valuebl);Serial.print('\t');Serial.print(valuefr);Serial.print('\t');
		//Serial.println(valuefl-1000);

		if(value>1000)
		{
			m1.writeMicroseconds(valuebl);
			m2.writeMicroseconds(valuebr);
			m3.writeMicroseconds(valuefl);
			m4.writeMicroseconds(valuefr); 
		}
		else if(value<=1000)
		{
			m1.writeMicroseconds(1000);
			m2.writeMicroseconds(1000);
			m3.writeMicroseconds(1000);
			m4.writeMicroseconds(1000); 
		}


	}
	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)

		

		
	} else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		// display Euler angles in degrees
		mpu.getRotation(&gx,&gy,&gz);
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		//Serial.print(gx);Serial.print('\t');Serial.print(gy);Serial.print('\t');Serial.println(gz);
		
		// blink LED to indicate activity
		blinkState = !blinkState;
		digitalWrite(13, blinkState);
	}





	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
}





//////////////////////////Subroutine to Calculate PID Controller Correction Values///////////////13
void pid()
{
	dmp_roll_input  = ypr[2]*180/M_PI;
	dmp_pitch_input = ypr[1]*180/M_PI;
	dmp_yaw_input   = ypr[0]*180/M_PI;
	
	//Roll calculations
	pid_error_temp =dmp_roll_input;
	pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
	if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
	else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

	pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
	if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
	else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

	pid_last_roll_d_error = pid_error_temp;

	//Pitch calculations
	pid_error_temp = dmp_pitch_input;
	pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
	if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
	else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

	pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
	if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
	else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
	pid_last_pitch_d_error = pid_error_temp;

	//Yaw calculations
	pid_error_temp = dmp_yaw_input;
	pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
	if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
	else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

	pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
	if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
	else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

	pid_last_yaw_d_error = pid_error_temp;
	//Serial.print(pid_output_yaw);Serial.print('\t');Serial.print(pid_output_pitch);Serial.print('\t');Serial.println(pid_output_roll);
	
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void pidrate()
{

	gyro_x_input=(float)(gyro_x_input*.99)+(gx*0.01);
	gyro_y_input=(float)(gyro_y_input*.99)+(gy*0.01);
	gyro_z_input=(float)(gyro_z_input*.99)+(gz*0.01);
  smoothx=(smoothx*.8)+(pid_output_roll*.2);
  smoothy=(smoothy*.8)+(pid_output_pitch*.2);
  smoothz=(smoothz*.8)+(pid_output_yaw*.2);
  //Serial.print(mpu_x_input);Serial.print('\t');Serial.print(mpu_y_input);Serial.print('\t');Serial.println(mpu_z_input);
	
	pidrate_error_temp =gyro_x_input + smoothx;
	pidrate_i_mem_roll += pidrate_i_gain_roll * pidrate_error_temp;
	if(pidrate_i_mem_roll > pidrate_max_roll)pidrate_i_mem_roll = pidrate_max_roll;
	else if(pidrate_i_mem_roll < pidrate_max_roll * -1)pidrate_i_mem_roll = pidrate_max_roll * -1;

	pidrate_output_roll = pidrate_p_gain_roll * pidrate_error_temp + pidrate_i_mem_roll + pidrate_d_gain_roll * (pidrate_error_temp - pidrate_last_roll_d_error);
	if(pidrate_output_roll > pidrate_max_roll)pidrate_output_roll = pidrate_max_roll;
	else if(pidrate_output_roll < pidrate_max_roll * -1)pidrate_output_roll = pidrate_max_roll * -1;

	pidrate_last_roll_d_error = pidrate_error_temp;
 
	//Pitch calculations
	pidrate_error_temp = gyro_y_input - smoothy;
	pidrate_i_mem_pitch += pidrate_i_gain_pitch * pidrate_error_temp;
	if(pidrate_i_mem_pitch > pidrate_max_pitch)pidrate_i_mem_pitch = pidrate_max_pitch;
	else if(pidrate_i_mem_pitch < pidrate_max_pitch * -1)pidrate_i_mem_pitch = pidrate_max_pitch * -1;

	pidrate_output_pitch = pidrate_p_gain_pitch * pidrate_error_temp + pidrate_i_mem_pitch + pidrate_d_gain_pitch * (pidrate_error_temp - pidrate_last_pitch_d_error);
	if(pidrate_output_pitch > pidrate_max_pitch)pidrate_output_pitch = pidrate_max_pitch;
	else if(pidrate_output_pitch < pidrate_max_pitch * -1)pidrate_output_pitch = pidrate_max_pitch * -1;
	pidrate_last_pitch_d_error = pidrate_error_temp;

	//Yaw calculations
	pidrate_error_temp = gyro_z_input - smoothx;
	pidrate_i_mem_yaw += pidrate_i_gain_yaw * pidrate_error_temp;
	if(pidrate_i_mem_yaw > pidrate_max_yaw)pidrate_i_mem_yaw = pidrate_max_yaw;
	else if(pidrate_i_mem_yaw < pidrate_max_yaw * -1)pidrate_i_mem_yaw = pidrate_max_yaw * -1;

	pidrate_output_yaw = pidrate_p_gain_yaw * pidrate_error_temp + pidrate_i_mem_yaw + pidrate_d_gain_yaw * (pidrate_error_temp - pidrate_last_yaw_d_error);
	if(pidrate_output_yaw > pidrate_max_yaw)pidrate_output_yaw = pidrate_max_yaw;
	else if(pidrate_output_yaw < pidrate_max_yaw * -1)pidrate_output_yaw = pidrate_max_yaw * -1;

	pidrate_last_yaw_d_error = pidrate_error_temp;
  //Serial.print(pid_output_pitch);Serial.print('\t');Serial.println(-pidrate_output_pitch);
}

