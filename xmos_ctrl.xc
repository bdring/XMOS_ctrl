/*
 ============================================================================
 Name        : xmos_ctrl.xc
 Description : Controls a Laser Engraver 
 
 The goal is to create a controller for laser engraving.  Engraving is done
 using stepper motors to scan across the work piece.  The stepper motor controller
 only requires pulses for steps and a signal for direction.  In order to get highest
 speed without loosing steps, the steppers must smoothly accelerate and decelerate.  
 The control process is something like this...
 
 1.  Clear the Emergency Stop condition
 2.  Set the acceleration and max speed parameters for both axes
 3.  Move the laser head to the desired start point
 4.  Zero the axes.
 5.  Specify the scan.  (X width in steps, steps per row in Y, number of rows, max speed.
 6.  Start the scan.
 
 A scan goes from X 0 to X max, moves up in Y the steps per row, then goes back to X 0.
 It continues the zig-zag pattern until done.  At each step it sets a PWM value to 
 control the laser power.  This is based on the gray value of the image and the speed of
 the laser head.  This keeps the value correct while accelerating.
 
 ============================================================================
 */

#include <xs1.h>
#include <print.h>  // for print statements to the JTAG
#include "xmos_ctrl.h"

// function prototypes
// Threads
void PWM_power(out port pwmPin, chanend pwm);
void pulseEngine(out port motionIO, chanend mx, chanend step);
void UART ( in port RX , int rxPeriod , out port TX , int txPeriod, chanend rxChar, streaming chanend txChar);
void commCenter(chanend rxChar, chanend mx, chanend step, chanend pwm, streaming chanend txChar);
// functions
void setAxisParams(unsigned int accelSteps, unsigned int maxSpeedSteps, axisParameters &theAxis);
void setMoveParams(signed int destination, axisParameters &theAxis);
void resetAxis(axisParameters &theAxis, char eStop);
int make32(char msB, char b1, char b2, char lsB);
unsigned int calcPower(unsigned int pixelVal, unsigned int currentSpeed, unsigned int powerRange, unsigned int minPower);
//unsigned int calculatePower(unsigned int currSpeed, unsigned int pixelVal, unsigned int powerRange, unsigned int minPower);

int main() {

	chan pwmVal;
	chan mx;
	chan rxChar;
	streaming chan txChar;
	chan step;
	
	printstr("\nXMOS Engaving Controller 1.2.0.0");
		
	// the following functions run on parallel threads
  	par
  	{
	  PWM_power(led, pwmVal);
	  pulseEngine(motIO, mx, step);
	  UART(uartRx, BIT_TIME, uartTx, BIT_TIME, rxChar, txChar);
	  commCenter(rxChar,mx, step, pwmVal, txChar);
	  
  	}
	
	while(1)
		;
	return 0;
}

/*
 * This is the hub of all threads.
 * 
 */
void commCenter(chanend rxChar,chanend mx, chanend step, chanend pwm, streaming chanend txChar)
{
	char cmdBuf[BUFF_LEN];	// char array to hold command from the UART 
	unsigned int bufPos;	// current position we are filling in the cmdBuf array
	int bGotHeader;			// have we receive the SOH character yet...we will ignore all charaters until we see it
	int bImageData;
	
	char imageLineBuf[IMG_BUFF_SIZE][2]; // two (Rows) of image information.  One current and one for the next line.
	unsigned int curBufImgRow;           // This is the array row currently used for pixel data
	unsigned int nextBufImgRow;			 // this is array row being filled for the next row
	
	int imageRes[2];			  //  X Y resolution of the image in pixels
	int engraveResSteps[2];		  //  X Y resplution of engraving in steps
	unsigned char maxPower;		  // this stores the max desired power (0-255).  We may only want to engrave at half power max
	unsigned char minPower;       // set a minimum desired power that we will never go below (due to image data) while scanning 
	unsigned char powerRange;	  // the working power range due to pixwl data - bascally maxPower - minPower
	int powerVal;				  // the on time for the PWM (0 - PWM_PERIOD)
	unsigned int currentXPixel;	  // the current X pixel we are on
	unsigned int currentYPixel;   // the current Y Pixel we are on
	
	unsigned int lastBufferedRow;
	
	int maxSpeed;				  // this is the maxX speed.  It is compared with current speed to set power
	unsigned char pixelVal;		  // the grayscale (0-255) of the current pixel
	
	timer commTimoutTmr;	// this is used to clear the buffer if nothing has been received for a while
	int commTimeout;		// the time at which the time out will occur
	
	mot motCmd;				// the motion command to be send via the channel to the pulse engine
	stepInfo currentStep;   // this will be received from the pulse engine to set power
	
	char rx;				// the character received.  It comes from the channel from the UART
	
	// initialize buffer
	bufPos = 0;
	bGotHeader = FALSE;
	bImageData = FALSE;
	
	maxPower = 255;
	
	curBufImgRow = 0;
	nextBufImgRow = 0;
	
	currentYPixel = 0;
	
	// this is used to clear out the commBuffer after a given time without new chars
	// this cleans up stray characters
	// each succesful rx will reset this 
	commTimoutTmr :> commTimeout;
	commTimeout += COMM_TIMEOUT;
	
	while(1){
	
		select {
			case rxChar :> rx :  // We got a character from the UART via the channel
				
				if (! bGotHeader) // check for the SOH character
				{
					bufPos = 0;
					if (rx == SOH)
					{
						cmdBuf[bufPos] = rx;
						bGotHeader = TRUE;
						bufPos++;
					}
				}
				else
				{
					
					// For reference a command looks like this  ...  SOH LEN LEN CMD DATA ... ...
				
					// determine if the data goes into the cmdBuf or imageLineBuf
					if ((bufPos < CMD_DATA) || (cmdBuf[CMD_CMD] < CMD_RCV_IMG_DATA))
					{
						cmdBuf[bufPos] = rx;
						bImageData = FALSE;
					}
					else
					{
						imageLineBuf[bufPos - 3][nextBufImgRow] = rx;  // data for image starts at postion 3
						bImageData = TRUE;
					}
					
					bufPos++;
					
					if ( (!bImageData && bufPos >= BUFF_LEN) || (bImageData && bufPos >= IMG_BUFF_SIZE)) // make sure we don't overflow the buffers
					{
						// problem...discard data and reset
						printstr("\nBuffer Overflow");
						// reset the buffer
						bufPos = 0;
						bGotHeader = FALSE;
					}
					else
					{
						if (bufPos >= CMD_LEN_LSB && bufPos == (cmdBuf[CMD_LEN_MSB]*256 + cmdBuf[CMD_LEN_LSB]) )  // if we are past the length character and the buffer position  = the length char 
						{
							
							// we may have a complete command....check it
							switch (cmdBuf[CMD_CMD])  // switch on the command byte
							{
							case CMD_E_STOP:
								txChar <: ACK;
								motCmd.command = CMD_E_STOP;								
								mx <: motCmd;				
								pwm <: 0;  // turn off the PWM
							break;
							
							
							case CMD_E_STP_CLEAR:
								txChar <: ACK;								
								motCmd.command = CMD_E_STP_CLEAR;
								mx <: motCmd;								
							break;
							
							case CMD_SET_AXIS_PARAMS:	
								txChar <: ACK;
								motCmd.command = CMD_SET_AXIS_PARAMS;						
								motCmd.P1 = make32(cmdBuf[CMD_DATA],cmdBuf[CMD_DATA+1],cmdBuf[CMD_DATA+2],cmdBuf[CMD_DATA+3]);
								motCmd.P2 = make32(cmdBuf[CMD_DATA+4],cmdBuf[CMD_DATA+5],cmdBuf[CMD_DATA+6],cmdBuf[CMD_DATA+7]);
								motCmd.P3 = make32(cmdBuf[CMD_DATA+8],cmdBuf[CMD_DATA+9],cmdBuf[CMD_DATA+10],cmdBuf[CMD_DATA+11]);
								
								// if this is the x axis we need to store the maxSpeed for use in pwm power calculations
								if (motCmd.P3 == X_AXIS)
								{
									// save max speed in engine steps/sec for power calcs during scans;
									maxSpeed = motCmd.P2;
								}
							
								mx <: motCmd;
							break;
							
							case CMD_ZERO_AXIS:
								txChar <: ACK;
								motCmd.command = CMD_ZERO_AXIS;						
								motCmd.P1 = make32(cmdBuf[CMD_DATA],cmdBuf[CMD_DATA+1],cmdBuf[CMD_DATA+2],cmdBuf[CMD_DATA+3]);
								//printintln(motCmd.P1);
								mx <: motCmd;
							break;
							
							case CMD_GOTO_HOME:						
								txChar <: ACK;
								motCmd.command = CMD_GOTO_HOME;
								motCmd.P1 = make32(cmdBuf[CMD_DATA],cmdBuf[CMD_DATA+1],cmdBuf[CMD_DATA+2],cmdBuf[CMD_DATA+3]);
								mx <: motCmd;
								
							break;
							
							case CMD_MOVE_TO:
								txChar <: ACK;
								motCmd.command = CMD_MOVE_TO;
								motCmd.P3 = make32(cmdBuf[CMD_DATA],cmdBuf[CMD_DATA+1],cmdBuf[CMD_DATA+2],cmdBuf[CMD_DATA+3]);
								motCmd.P4 = make32(cmdBuf[CMD_DATA+4],cmdBuf[CMD_DATA+5],cmdBuf[CMD_DATA+6],cmdBuf[CMD_DATA+7]);
								//printintln(motCmd.P3);							
								//printintln(motCmd.P4);
								mx <: motCmd;
							break;
							
							case CMD_DO_SCAN:
								txChar <: ACK;
								motCmd.command = CMD_DO_SCAN;
								motCmd.P1 = make32(cmdBuf[CMD_DATA],cmdBuf[CMD_DATA+1],cmdBuf[CMD_DATA+2],cmdBuf[CMD_DATA+3]);  // x steps								
								motCmd.P2 = make32(cmdBuf[CMD_DATA+4],cmdBuf[CMD_DATA+5],cmdBuf[CMD_DATA+6],cmdBuf[CMD_DATA+7]); // y rows
								motCmd.P3 = make32(cmdBuf[CMD_DATA+8],cmdBuf[CMD_DATA+9],cmdBuf[CMD_DATA+10],cmdBuf[CMD_DATA+11]); // Y stepover
								imageRes[X_AXIS] = cmdBuf[CMD_DATA+12]* 256 + (int)cmdBuf[CMD_DATA+13]; // X image res
								imageRes[Y_AXIS] = cmdBuf[CMD_DATA+14] * 256 + (int)cmdBuf[CMD_DATA+15]; // y image res						
								maxPower = cmdBuf[CMD_DATA+16]; // max power
								minPower = cmdBuf[CMD_DATA+17]; // max power
								
								powerRange = maxPower - minPower;
								
								// we need to save this to determine current pixel to use for PWM
								engraveResSteps[X_AXIS] = motCmd.P1;
								engraveResSteps[Y_AXIS] = motCmd.P2;
								
								// stream out the command to get the row information
								// we have row 0, we need to get row 1 
								
								txChar <: 1;
								txChar <: 0;
								txChar <: 6;
								txChar <: CMD_SEND_IMG_DATA;
								txChar <: 0;
								txChar <: 1;
								
								
								mx <: motCmd;
							break;
							
							case CMD_RCV_IMG_DATA:
								// if the row is 0 then reset the row counters
								
							break;
							
							default:
								printstr("\nUnknown Command");
								printint(cmdBuf[CMD_CMD]);
							break;
							
							}						
							bGotHeader = FALSE;
							bufPos = 0;
							
						}
						
					}
					
					
				}
				
				//printhex(rx);
			
				// reset the timeout timer
				commTimoutTmr :> commTimeout;
				commTimeout += COMM_TIMEOUT;
			
			break;
			
			case step :> currentStep:  // a new step has occered...we need to set the pwm
				
				// the newScanRow flag tells use we are done with the last row and need
				// to buffer the next row.  TODO: There may be more scan rows than image rows
				// such the it will scan the same image row twice.  No need to get the same image row twice
				if (currentStep.newScanRow)
				{
					
					// flip between what row is being used and what will be read in
					curBufImgRow ^= 1;
					nextBufImgRow ^= 1;
					
					currentYPixel = (currentStep.yLocation * imageRes[Y_AXIS]) / engraveResSteps[Y_AXIS];
					
					// stream out the command to get the row information
					txChar <: SOH;
					txChar <: 0;
					txChar <: 6;
					txChar <: CMD_SEND_IMG_DATA;
					txChar <: (currentYPixel & 0xFF00) / 256;
					txChar <: currentYPixel & 0xFF;
				}
			
			
				if (engraveResSteps[X_AXIS] != 0)  // prevent divide by zero error
				{	
					// determine what pixel we want
					currentXPixel = (currentStep.xLocation * imageRes[X_AXIS]) / engraveResSteps[X_AXIS];
					pixelVal = imageLineBuf[currentXPixel][curBufImgRow]; // 0-255 from pixel gray color
					
					// now we need to apply the range and min power			
					pixelVal = ((pixelVal * powerRange) / MAX_PIXEL_VALUE) + minPower;
					
				
					// calculate power based on current speed 
					powerVal = (PWM_PERIOD * currentStep.currentSpeed) / 255;
					
					//apply the pixel value
					
					powerVal = (powerVal * pixelVal) / 255;
						
					// make sure we are in a valid range
					if ((powerVal < 0) || (powerVal > PWM_PERIOD))					
						printintln(currentStep.currentSpeed);
					
					//powerVal = calcPower(pixelVal, currentStep.currentSpeed, powerRange, minPower);
					
					pwm <: powerVal;
							
				}
				
			break;
			
			// See if it has been too long since the last character..if so, reset the buffer
			case commTimoutTmr when timerafter(commTimeout) :> commTimeout:		
				bufPos = 0;
				bGotHeader = FALSE;
				commTimeout += COMM_TIMEOUT;
			break;
			
		}
	}
}

/* This outputs a PWM signal on a pin
 * 
 * pwmVal = The PWM value (0 - PWM_PERIOD) 
 * 
 */
void PWM_power(out port pwmPin, chanend pwm)
{
	timer tmr;
	unsigned time;
	int on_time, state_change;
	char on_off = 0;
	
	on_time = 0;
	
	while(1)
	{
		select
		{
			case pwm :> on_time :  // this case only occurs if the channel is passing us a value
					
			break;
		
			case tmr when timerafter(state_change) :> time :
				
				// if the period is full on or 0 then just set the led and wait for one period 
				if (on_time == PWM_PERIOD)
				{
					pwmPin <: 1;
					state_change = time + PWM_PERIOD;
				}
				else if (on_time == 0)
				{
					pwmPin <: 0;
					state_change = time + PWM_PERIOD;
				}
				else // PWM is somewhere in the middle toggle between on time and off time
				{				
					pwmPin <: on_off;
					on_off = ( 0x01 & (~on_off));  // flip value
					
					if (on_off == 1)
						state_change = time + (PWM_PERIOD - on_time); // calculate off time
					else    
						state_change = time + on_time; 		  // calculate on time
				}
			break;
		}
		
	}
}

void UART ( in port RX , int rxPeriod , out port TX , int txPeriod, chanend rxChar, streaming chanend txChar ) {
	int txByte , rxByte ;
	int txI , rxI;
	int rxTime , txTime ;
	int isTX = 0;
	int isRX = 0;
	
	char rxData;
	unsigned int txData;
	
	timer tmrTX , tmrRX ;
	
	// setup a FIFO buffer for the transmit buffer
	char txFifoBuf[BUFF_LEN];
	unsigned int txFifoRead = 0, txFifoWrite = 0;
	
	while (1) 
	{
		if (! isTX && (txFifoRead != txFifoWrite) ) {
			isTX = 1;
			txI = 0;
			
			txByte = txFifoBuf[txFifoRead];
			txFifoRead = (txFifoRead + 1) % BUFF_LEN;  // incr the read point or rollover to 0
			
			TX <: 0; // transmit start bit
			tmrTX :> txTime ; // set timeout for data bit
			txTime += txPeriod ;
		}
		
		
		select {
		
			case ! isRX => RX when pinseq (0) :> void :
				isRX = 1;
				tmrRX :> rxTime ;
				rxI = 0;
				rxTime += rxPeriod ;
			break ;
			
			case isRX => tmrRX when timerafter ( rxTime ) :> void :
				if ( rxI < 8)
					RX :> >> rxByte ;
				else { // receive stop bit
					RX :> void ;					
					//putByte ( rxByte >> 24);
					rxData = (rxByte >> 24);
					rxChar <: rxData;
					isRX = 0;
				}
				rxI ++;
				rxTime += rxPeriod ;
			break ;
			
			case isTX => tmrTX when timerafter ( txTime ) :> void :
				if ( txI < 8)
					TX <: >> txByte ;
				else if (txI == 8)
					TX <: 1; // stop bit
				else
					isTX = 0;
			
				txI ++;
				txTime += txPeriod ;
				
			break ;
			
			case txChar :> txData :  // We got a character from the UART via the channel
				
				txFifoBuf[txFifoWrite] = (unsigned char)txData;
				txFifoWrite = (txFifoWrite + 1) % BUFF_LEN; // incr or rollover to 0
			break;
		} 
	} 
}






int make32(char msB, char b1, char b2, char lsB)
{
	uCharUnsignedInt charInt;
	
	charInt.byte[3] = msB;
	charInt.byte[2] = b1;
	charInt.byte[1] = b2;
	charInt.byte[0] = lsB;	
	
	return charInt.integer;
}


/*	 ================== PULSING ENGINE ======================================
 * 
 * This is a very basic pulsing engine for controlling stepper motors.
 *  The engine runs at a regular frequency of ENGINE_PERIOD.  Each iteration
 *  is called an engine "tick" in this routine.
 * 
 *  The engine runs at a frequency higher than any step rate that will ever 
 *  occur.  An accumulator is used to count up to the next step.  To keep away
 *  from float math the accumulator counts up from 0 to SPEED_OFFSET.
 * 
 *  The commands are presented in steps, so before each move, calculations
 *  are done to convert everything into accumulator units.  Also the point
 *  at which the move must start to decelerate is precaqlculated.
 * 
 *  This does nothing to coordinate motion.  This should be done before the
 *  move by specifing the correct accel/and maxSpeed values to coordinate the
 *  motion.
 *  
 *  
 * At each tick, the speed is incremented by the acceleration rate and the 
 * 	
 */
void pulseEngine(out port motionIO, chanend mx, chanend step)
{
	
	#define ENGINE_PERIOD TMR_SPEED / ENGINE_RATE;
	#define PULSE_ON_TIME 200 // (min 1uS per allegro data sheet)
	
	// timers
	timer engTmr;
	timer pulseTmr;
	unsigned int engTime;
	unsigned pulseTime;
	
	// used for scanning
	int yStepover;
	int xScanMax;
	int yScanMax;
	char bScanning = FALSE;
	char currentScanDirection;
	char bNewScanRow = FALSE;
	
	unsigned char motPort; // This stores the value of the I/O port 
	
	char eStop = TRUE;
	
	int axis;
	
	axisParameters axisParams[2];  // 
	
	stepInfo currentStep;
		
	mot myCommand;
	
	for (axis=0; axis<2; axis++)
	{
		axisParams[axis].currentLocation = 0;
		axisParams[axis].currentSpeed = 0;
		axisParams[axis].accumulator = 0;
		axisParams[axis].destination = 0;
		axisParams[axis].bDecel = FALSE;
	}
	
	motPort = 0;
	motionIO <: motPort;
	
	engTmr :> engTime;
	
	while(1)
	{
		select
		{
			case mx :> myCommand :  // this case only occurs if the channel is passing us a value
				
				
				switch (myCommand.command)
				{
					case CMD_E_STOP:
						eStop = TRUE;
						bScanning = FALSE;
						resetAxis(axisParams[X_AXIS], TRUE);
						resetAxis(axisParams[Y_AXIS], TRUE );
					break;
					
					case CMD_E_STP_CLEAR:
						eStop = FALSE;
						resetAxis(axisParams[X_AXIS], FALSE);
						resetAxis(axisParams[Y_AXIS], FALSE);
					break;
					
					case CMD_SET_AXIS_PARAMS:
						if (myCommand.P3 < 2)  // make sure it is a valid axis
						{							
							setAxisParams(myCommand.P1, myCommand.P2, axisParams[myCommand.P3]);	
						}
					break;
					
					case CMD_ZERO_AXIS:
						switch (myCommand.P1)
						{
						case X_AXIS:
						case Y_AXIS:
							axisParams[myCommand.P1].currentLocation = 0;
							resetAxis(axisParams[myCommand.P1], TRUE);
						break;
						case 99:
							axisParams[X_AXIS].currentLocation = 0;
							resetAxis(axisParams[X_AXIS], TRUE);
							axisParams[Y_AXIS].currentLocation = 0;
							resetAxis(axisParams[Y_AXIS], TRUE);
							
						break;
						}
					break;
					
					case CMD_GOTO_HOME:
						
						switch (myCommand.P1)
						{
						case X_AXIS:							
						case Y_AXIS:
							resetAxis(axisParams[myCommand.P1], FALSE);
							setMoveParams(0, axisParams[myCommand.P1]);
						break;
						case 99:
							resetAxis(axisParams[X_AXIS], FALSE);
							setMoveParams(0, axisParams[X_AXIS]);
							
							resetAxis(axisParams[Y_AXIS], FALSE);
							setMoveParams(0, axisParams[Y_AXIS]);
						break;
						}
						// setup the direction pins
						if (axisParams[X_AXIS].motionDirection == DIRECTION_FORWARD)
							motPort |= X_DIR_BIT; // 0010 x dir pin fwd
						else
							motPort &= ~(X_DIR_BIT); //0xd; // 1101 x pin direction rev
						
						if (axisParams[Y_AXIS].motionDirection == DIRECTION_FORWARD)
							motPort |= Y_DIR_BIT;
						else
							motPort &= ~(Y_DIR_BIT);
						
						motionIO <:  motPort;
						// the direction pin needs to be on for 200nS before stepping per data sheet so move out the engTime timer	
						engTmr :> engTime;
						engTime += DIR_PIN_BEFORE_STEP; 
					break;
					
					case CMD_MOVE_TO:
						
							
						if (eStop)
						{							
							//mx <: myCommand;
							continue;
						}
						
						resetAxis(axisParams[X_AXIS], FALSE);
						resetAxis(axisParams[Y_AXIS], FALSE);
						
						setMoveParams(myCommand.P3, axisParams[X_AXIS]);						
						setMoveParams(myCommand.P4, axisParams[Y_AXIS]);
						
						
						
						// setup the direction pins
						if (axisParams[X_AXIS].motionDirection == DIRECTION_FORWARD)
							motPort |= X_DIR_BIT; // 0010 x dir pin fwd
						else
							motPort &= ~(X_DIR_BIT); //0xd; // 1101 x pin direction rev
						
						if (axisParams[Y_AXIS].motionDirection == DIRECTION_FORWARD)
							motPort |= Y_DIR_BIT;  //8; // 1000 y dir pin forward
						else
							motPort &= ~(Y_DIR_BIT); //0x7; // 0111 y pin dir ref
						
						
						
						motionIO <:  motPort;
						// the direction pin needs to be on for 200nS before stepping per data sheet so move out the engTime timer	
						engTmr :> engTime;
						engTime += DIR_PIN_BEFORE_STEP;
						
						#ifdef DEBUG_PRINT
							printstr("\nGoto X:");
							printint(axisParams[X_AXIS].destination);
							printstr("\nGoto Y:");
							printint(axisParams[Y_AXIS].destination);
						#endif
							
					break;
					
					case CMD_DO_SCAN:
						// set the variables
						
						
						
						xScanMax = myCommand.P1;
						yScanMax = myCommand.P2;
						yStepover = myCommand.P3;
						bScanning = TRUE;
						
						// reset the axes
						resetAxis(axisParams[X_AXIS], FALSE);
						resetAxis(axisParams[Y_AXIS], FALSE);
						
						// seup the first move
						setMoveParams(xScanMax, axisParams[X_AXIS]);
						currentScanDirection = X_AXIS;
						
						
						//printstr("Starting Scan");
						
						// set direction signal
						motPort |= X_DIR_BIT; // Move is forward
						motionIO <:  motPort;
						// the direction pin needs to be on for 200nS before stepping per data sheet so move out the engTime timer	
						engTmr :> engTime;
						engTime += DIR_PIN_BEFORE_STEP;
						
					break;	
					
					default:
						// unknown command
					break;
				}
				
			break;
		
			// ================= Stepper Engine Starts Here ==========================
			case engTmr when timerafter(engTime) :> engTime:
				
				engTime += ENGINE_PERIOD;
			
				axis = X_AXIS;
				
				if (eStop)
				{
					resetAxis(axisParams[X_AXIS], TRUE);
					resetAxis(axisParams[Y_AXIS], TRUE);
					continue;
				}
					
					for (axis = X_AXIS; axis < 2; axis++)					
					{	
						
						if (axisParams[axis].currentLocation != axisParams[axis].destination)
						{
						
							if (axisParams[axis].currentLocation == axisParams[axis].decelLocation)
								axisParams[axis].bDecel = TRUE;
							
							// do we accel or decel
							if (! axisParams[axis].bDecel)
							{						
								if (axisParams[axis].currentSpeed < axisParams[axis].maxSpeedTicks)
									axisParams[axis].currentSpeed += axisParams[axis].accelTicks;
							}
							else
							{
								axisParams[axis].currentSpeed -= axisParams[axis].accelTicks;
								
						         if (axisParams[axis].currentSpeed <= 0) // never let it get to zero otherwise we could get stuck
						        	 axisParams[axis].currentSpeed = axisParams[axis].accelTicks;
							}
							
							axisParams[axis].accumulator += axisParams[axis].currentSpeed;
							
							if (axisParams[axis].accumulator > SPEED_OFFSET) // this basically tests the 24th bit...(keeps it 8-bit)
							{
								// Turn step pin on
								if (axis == X_AXIS)
									motPort |= X_STEP_BIT; // turn on the X step pin
								else
									motPort |= Y_STEP_BIT; // turn on the X step pin
								
								// update the current location and reset the accululator
								axisParams[axis].currentLocation += axisParams[axis].motionDirection;
								axisParams[axis].accumulator -= SPEED_OFFSET;	
							}
						}
					}
						
							
							if (bScanning)
							{
								// if both axes are at destination we need to determine the next move
								if (axisParams[X_AXIS].currentLocation == axisParams[X_AXIS].destination &&   axisParams[Y_AXIS].currentLocation == axisParams[Y_AXIS].destination)
								{
									if (axisParams[Y_AXIS].currentLocation >= yScanMax) // If we here we are done
									{
										bScanning = FALSE;
										// TO DO post scan cleanup???
									}
									else
									{
										// move up or scan next line
										if (currentScanDirection == X_AXIS)
										{
											
											currentScanDirection = Y_AXIS;
											// move up in Y																		
											resetAxis(axisParams[Y_AXIS], FALSE);
											setMoveParams(axisParams[Y_AXIS].currentLocation + yStepover, axisParams[Y_AXIS]);
											// set direction signal
											motPort |= Y_DIR_BIT; // Y move is always forward in scan mode											
										}
										else
										{
											bNewScanRow = TRUE;
											currentScanDirection = X_AXIS;
											// we zig zag in X so we need to determine which way to we go into X?
											if (axisParams[X_AXIS].currentLocation == 0)
											{
												// move forward to x max
												resetAxis(axisParams[X_AXIS], FALSE);
												setMoveParams(xScanMax, axisParams[X_AXIS]);
												motPort |= X_DIR_BIT; // Move is forward
											}
											else
											{												
												resetAxis(axisParams[X_AXIS], FALSE);
												setMoveParams(0, axisParams[X_AXIS]);
												motPort &= ~(X_DIR_BIT); // x pin direction rev												
											}
										}
										motionIO <:  motPort;
										// the direction pin needs to be on for 200nS before stepping per data sheet so move out the engTime timer	
										engTmr :> engTime;
										engTime += DIR_PIN_BEFORE_STEP;
									}
								}
							}
					// If either step pin is on in motPort then we need to sent that to the I/O port
					// then wait the PULSE_ON_TIME, then turn the step pins off
					if ((motPort && 0x5) != 0)
					{
						motionIO <:  motPort; // send it to the port
						pulseTmr :> pulseTime;
						pulseTime += PULSE_ON_TIME;
						pulseTmr when timerafter(pulseTime) :> void;
						motPort &= ~(X_STEP_BIT + Y_STEP_BIT); //0xA; // 1010 // turn off x & y step pins
						motionIO <:  motPort; // send it to the port	
						
						if (bScanning)
						{
							// setup step information to send out to the commCenter for power calcs and row buffering
							currentStep.xLocation = axisParams[X_AXIS].currentLocation;
							currentStep.yLocation = axisParams[Y_AXIS].currentLocation;
							
							currentStep.currentSpeed = (axisParams[X_AXIS].currentSpeed) / (axisParams[X_AXIS].maxSpeedTicks / SCAN_FULL_SPEED_VAL);
							
							currentStep.newScanRow = bNewScanRow;
							bNewScanRow = FALSE; // so this only happens once after setting it.
							
							step <: currentStep;	
						}	
					}	
			break;
		}
	}
}



/* 
 * 
 * This converts acceleration and max speed from units in steps/sec to
 * pulse engine accumulator units per engine ticks.  It also
 * determines the deceleration distance in steps from the maximum speed
 * 
 * The accelDist equation is manipulated so that it can be done with
 * integers math and minimal loss of precision.
 * 
 * Parameters
 * 		accelSteps		= The acceleration in steps/sec^2
 * 		maxSpeedSteps	= The maximum speed in steps/sec
 * 		theAxis			= This is the structure that contains the axis information.
 * 						  it is passed by reference.
 * 	
 */
void setAxisParams(unsigned int accelSteps, unsigned int maxSpeedSteps, axisParameters &theAxis)
{
	// convert accel steps to accel ticks
	theAxis.accelTicks = SPEED_OFFSET / ENGINE_RATE;
	theAxis.accelTicks *= accelSteps;
	theAxis.accelTicks /= ENGINE_RATE;
	
	theAxis.maxSpeedTicks = SPEED_OFFSET / ENGINE_RATE;
	theAxis.maxSpeedTicks *= maxSpeedSteps;
	
	
	//determine Accel distance in steps
	//     d = 1/2 a t^2
	//     t = maxSpeed/accel .... (time to max speed)
	//		To work with integers do this
	//    	d = speed^2 * a / accel^2 / 2
	//   or d = speed^2 / accel / 2
	theAxis.accelDist = maxSpeedSteps * maxSpeedSteps / accelSteps / 2;

	#ifdef DEBUG_PRINT
		printstr("\nAccel Ticks:");
		printint(theAxis.accelTicks);
		
		printstr("\nMax Speed Ticks:");
		printint(theAxis.maxSpeedTicks);
		
		printstr("\nAccel dist:");
		printint(theAxis.accelDist);
	#endif

}



/*  This gives the destination, movement direction, and point to start decelerating
 *  to the axis structure.
 * 
 * 
 * 	Parameters
 * 		destination = The location to go to in steps
 * 		theAxis		= This is the structure that contains the axis information.
 * 					  it is passed by reference.
 * 
 */
void setMoveParams(signed int destination, axisParameters &theAxis)
{
	int distToDestination;
	
	theAxis.destination = destination; // set the destination
	
	distToDestination = (theAxis.destination - theAxis.currentLocation); // Distance we will travel on this move
							
	// the distance should be positive
	if (distToDestination < 0)
		distToDestination *= -1;
	
	if (theAxis.destination > theAxis.currentLocation)  // If the destination is larger than the current location we go forward
	{
		theAxis.motionDirection = DIRECTION_FORWARD;
				      	
		// now determine decel location
						      
		if (theAxis.accelDist * 2 >= distToDestination)  // Will we get to full speed?
			theAxis.decelLocation = theAxis.destination - distToDestination / 2; // decel at the half way point
		else
			theAxis.decelLocation = theAxis.destination - theAxis.accelDist;  // decel the decel distance from destination 
	
	}
	else
	{
		theAxis.motionDirection = DIRECTION_REVERSE;
							      
		  // now determine the decel location
		  if (theAxis.accelDist * 2 >= distToDestination) // if we never get to full speed
			  theAxis.decelLocation = theAxis.destination + distToDestination / 2; // decel at the half way point
		  else
			  theAxis.decelLocation = theAxis.destination + theAxis.accelDist;  // decel the decel distance from destination
		      
	}
		
			#ifdef DEBUG_PRINT
				printstr("\nDest X:");
				printint(theAxis.destination);
			
				printstr("\nDecel:");
				printint(theAxis.decelLocation);
			
				printstr("\nDirection:");
				printint(theAxis.motionDirection);
	
			#endif
}

void resetAxis(axisParameters &theAxis, char eStop)
{
	theAxis.currentSpeed = 0;
	theAxis.accumulator = 0;
	theAxis.bDecel = FALSE;
	
	if (eStop == TRUE)
	{
		theAxis.destination = theAxis.currentLocation;
	}
}

unsigned int calcPower(unsigned int pixelVal, unsigned int currentSpeed, unsigned int powerRange, unsigned int minPower)
{
		
	unsigned int powerVal;
	
	// now we need to apply the range and min power			
	pixelVal = ((pixelVal * powerRange) / MAX_PIXEL_VALUE) + minPower;
	

	// calculate power based on current speed 
	currentSpeed = (PWM_PERIOD * currentSpeed) / 255;
	
	//apply the pixel value
	
	powerVal = (powerVal * pixelVal) / 255;
		
	// make sure we are in a valid range
	if (powerVal > PWM_PERIOD)					
		powerVal = PWM_PERIOD;
						
	return powerVal;
}
