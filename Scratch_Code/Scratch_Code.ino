////////////////HEADER FILES FOR I2C AND MPU////////////////1
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
////////////////////////////////////////////////////////////





/////////MPU OBJECT////2
MPU6050 mpu;
////////////////////////////





///////////////////////////VARIABLES///////////////////////////////////////////////////////////3

long int start, stopp, timer,value,valuefr=0,valuebr=0,valuebl=0,valuefl=0;
char chara;
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

//PID gain and limit settings
float pid_p_gain_roll = 0;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0;              //Gain setting for the roll I-controller (0.3)
float pid_d_gain_roll = 0;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 1;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)
///////////////////////////////////////////////////////////////////////////////////////////////////////




/////////////////////////////////MPU INTERRUPT ROUTINE//////////////////////////////////////////4
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////





/////////////////////SETTING STUFF UP/////////////////////////////////////////////////////////////////////////////////////////
void setup() {
	
	
	
	
	
	//////////////INITIAL1ZING COMMUNICATIONS////////////5
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif
delay(9000);
	Serial.begin(57600);
Serial.println("GET READY");
	/////////////////////////////////////////////////////




////////////////PORT INITIALIZATION//////////////////6
  DDRD |= B11011000;
  DDRB |= B00110000;
  PORTB |= B00110000;
  ////////////////////////////////////////////////////


	///////////MPU STUFF////////////////////7
	mpu.initialize();
 Serial.println("Initializing MPU");
	if (!mpu.testConnection())
	exit(0);
  Serial.println("MPU Connection Succesful");
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

 
  mpu.setXAccelOffset(-1097);
  mpu.setYAccelOffset(2131);
  mpu.setZAccelOffset(2489);
  mpu.setXGyroOffset(76);
  mpu.setYGyroOffset(-82);
  mpu.setZGyroOffset(9);
	/////////////////////////////////////////





	




	/*///////////////ACTIVATING ESCS/////////////////8
	value = 2400;
	timer = millis();
	while ((millis() - timer) < 5000)
	{
		start = micros();
		PORTD |= B11011000;
		while ((micros() - start) < value);
		PORTD &= B00000011;
		while (micros() - start < 20000);
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





///////////////////////////////////////FEEDBACK LOOP @50Hz//////////////////////////////////////////////////////////
void loop()
{
Serial.print("Sampling time\t\t");  
Serial.println(micros(),DEC);
Serial.print("begin\t\t\t");
Serial.println(micros(),DEC);
  
Serial.print("dmp data\t\t");  
Serial.println(micros(),DEC);


	////////////////////////GET DMP DATA//////////////////////////10
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
		mpu.dmpGetQuaternion(&q, fifobuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); 
	}
	///////////////////////////////////////////////////////////////////
Serial.print("Wireless Data\t\t");
Serial.println(micros(),DEC);
	/////////////////////////Recieve Wireless Data//////////////////////9
pid_roll_setpoint=0;
pid_pitch_setpoint=0;
pid_yaw_setpoint=0;

if (Serial.available())
{
  chara=Serial.read();
	if (chara == 'p')
	{
  pid_p_gain_roll+=.1;
  pid_p_gain_pitch+=.1;
  Serial.println(pid_p_gain_roll,DEC);
	}
	else if (chara == 'o')
	{
  pid_p_gain_roll-=.1;
  pid_p_gain_pitch-=.1;
  Serial.println(pid_p_gain_roll,DEC);
	}
	else if (chara == 'i')
	{
	pid_i_gain_roll+=.01;
  pid_i_gain_pitch+=.01;
  Serial.println(pid_i_gain_roll,DEC);
	}
	else if (chara == 'u')
	{
  pid_i_gain_roll-=.01;
  pid_i_gain_pitch-=.01;
	Serial.println(pid_i_gain_roll,DEC);
	}
  else if (chara == 'd')
  {
    pid_d_gain_roll+=.1;
    pid_d_gain_pitch+=.1;
    Serial.println(pid_d_gain_roll,DEC);
  }
  else if (chara == 's')
  {
    pid_d_gain_roll-=.1;
    pid_d_gain_pitch-=.1;
    Serial.println(pid_d_gain_roll,DEC);
  }
  else if(chara=='a')
  {
    Serial.println(pid_p_gain_roll,DEC);
    Serial.println(pid_i_gain_roll,DEC);
    Serial.println(pid_d_gain_roll,DEC);
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





	


Serial.print("pid loop\t\t");
Serial.println(micros(),DEC);
	//////////////////////CALL PID/////////////////////////////////////11
	pid();
	/////////////////////////////////////////////////////////////////////



Serial.print("ESC Bitbanging\t\t");
Serial.println(micros(),DEC);
	//////////////////////ESC BITBANGING//////////////////////////12
	//7=br
	//6=bl
	//4=fr
	//3=fl
  //clockwise yaw +ve
  //right roll +ve
  //backwards pitch +ve
  value=1600;
  valuebr=((    -   pid_output_pitch  -   pid_output_roll)  ) + value;
	if(valuebr<580)valuebr=580;if(valuebr>1200)valuebr=1200;
	valuebl=((   -   pid_output_pitch  +   pid_output_roll) ) +value;
  if(valuebl<600)valuebl=580;if(valuebl>1200)valuebl=1200;
  valuefr=((   +   pid_output_pitch  -   pid_output_roll)  ) +value;
  if(valuefr<600)valuefr=580;if(valuefr>1200)valuefr=1200;
  valuefl=((    +   pid_output_pitch  +   pid_output_roll) ) +value;
	if(valuefl<580)valuefl=580;if(valuefl>1200)valuefl=1200;
	start = micros();
	PORTD |= B11011000;
	while ((micros() - start) < 2500)
	{
		if(micros()-start>valuebr) PORTD &= B01011011;
		if(micros()-start>valuebl) PORTD &= B10011011;
		if(micros()-start>valuefr) PORTD &= B11001011;
		if(micros()-start>valuefl) PORTD &= B11010011;
	}
	while (micros() - start < 20000);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Serial.print("End\t\t\t");
Serial.println(micros(),DEC);
}




//////////////////////////Subroutine to Calculate PID Controller Correction Values///////////////13
void pid()
{
	dmp_roll_input = -ypr[2]*180/M_PI ;
	dmp_pitch_input = -ypr[1] *180/M_PI;
	dmp_yaw_input = -ypr[0] *180/M_PI;

	//Roll calculations
	pid_error_temp = dmp_roll_input - pid_roll_setpoint;
	pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
	if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
	else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

	pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
	if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
	else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

	pid_last_roll_d_error = pid_error_temp;
	
	//Pitch calculations
	pid_error_temp = dmp_pitch_input - pid_pitch_setpoint;
	pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
	if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
	else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

	pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
	if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
	else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

	pid_last_pitch_d_error = pid_error_temp;
	
	//Yaw calculations
	pid_error_temp = dmp_yaw_input - pid_yaw_setpoint;
	pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
	if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
	else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

	pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
	if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
	else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

	pid_last_yaw_d_error = pid_error_temp;
	
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
