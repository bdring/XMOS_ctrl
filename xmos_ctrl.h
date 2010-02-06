#ifndef XMOS_CTRL_H_
#define XMOS_CTRL_H_

// port assignments
out port led = XS1_PORT_1E;
out port motIO = XS1_PORT_8B;

in port uartRx = XS1_PORT_1C;  // Pin XD23
out port uartTx = XS1_PORT_1D; // Pin XD22 

in port p_button_0 = XS1_PORT_1K;
in port p_button_1 = XS1_PORT_1L;

#define TRUE				1
#define FALSE				0

//#define DEBUG_PRINT 1     // uncomment for debug info over JTAG...can mess up timing

// timer constants
#define TMR_SPEED  				100000000	// 100MHz XMOS System Timer
#define PWM_FREQ   				35000		// Hz
#define PWM_PERIOD				TMR_SPEED / PWM_FREQ
#define DIR_PIN_BEFORE_STEP 	30 // 300ns time after dir change before pulse 
#define THREE_SECS				TMR_SPEED * 3
#define COMM_TIMEOUT 			TMR_SPEED  // one second 

// for the UART
#define BIT_RATE 		57600
#define BIT_TIME 		TMR_SPEED / BIT_RATE

// for the pulsing engine
// make SPEED_OFFSET equally divisible by ENGINE_RATE to prevent integer operator data loss
#define ENGINE_RATE 	32768  // Hz - the speed of the pulsing engine
#define SPEED_OFFSET 	268435456  // 2^28

// Full scale power values
#define MAX_PIXEL_VALUE			255
#define MAX_REQUESTED_POWER		255
#define SCAN_FULL_SPEED_VAL 	255  // This indicates full speed on the scan...for power calc purposes

// stepper motor info
#define X_AXIS				0
#define Y_AXIS				1
#define Z_AXIS				2
#define AXIS_COUNT			3

// define the I/O bits for the axes
#define X_STEP_BIT			1
#define X_DIR_BIT			2
#define Y_STEP_BIT			4
#define Y_DIR_BIT			8
#define Z_STEP_BIT			16
#define Z_DIR_BIT			32

// protocol headers
#define SOH	0x01
#define ACK 0x06
#define NAK 0x15

// NAK reasons
#define NAK_E_STOP 0x01
#define NAK_AXIS_PARAMS 0x10

// define byte positions in protocol
#define CMD_LEN_MSB 	0x01  // position of len char 
#define CMD_LEN_LSB 	0x02
#define CMD_CMD			0x03
#define CMD_DATA 		0x04
#define BUFF_LEN		30
#define IMG_BUFF_SIZE 	3000

#define RSP_OK				1
#define RSP_ERR				2  // P1 = Error Code

// protocol commands
#define CMD_E_STOP			0 // emergency stop
#define CMD_E_STP_CLEAR		1 // clear emergency stop
#define CMD_SET_PWR_LVL		2 // P1 = Power Level (0-255)
#define CMD_SET_AXIS_PARAMS	4 // P1 = Accel, P2 = MaxSpeed, P3 = Axis
#define CMD_ZERO_AXIS		5 // P1 (0 = X, 1 = Y, 99 = All) 
#define CMD_GOTO_HOME		6 // P1 (0 = X, 1 = Y, 99 = All)
#define CMD_MOVE_TO			7 // P3 = X destination, P4 = Y destination	

#define CMD_DO_SCAN			10 

#define CMD_RCV_IMG_DATA	20  // revceive image data.  Length byte lets you know how much
#define CMD_SEND_IMG_DATA	21


#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE -1 

// this is a motion command struct.  P1..P4 are paramters to the command.  They are not given
// better names because they are used for different things for each command
typedef struct motVars
{
	char command;
	unsigned int P1;
	unsigned int P2;
	signed int P3;
	signed int P4;
	signed int P5;
}mot;


typedef struct axisInfo
{
		signed int currentLocation;
		unsigned int currentSpeed;
		unsigned int accumulator;
		unsigned int accelTicks;
		unsigned int maxSpeedTicks;
		signed int destination;
		int accelDist;			// how far it takes to accel/decel at max speed
		signed char motionDirection;
		unsigned int decelLocation;
		char bDecel;
}axisParameters;

/* 
This is used to pass information to another thread about the most recent step
it is used to determine what pixel to use and if any power changes due to speed 
need to be done
*/
typedef struct
{
	signed int xLocation;
	signed int yLocation;
	unsigned int currentSpeed;
	char newScanRow;
}stepInfo;

typedef union // this will be helpful to deserialize chars off the UART to ints
{
	char byte[4];
	unsigned int integer;

}uCharUnsignedInt;


#endif /*XMOS_CTRL_H_*/
