//----------------------------------------------------------------------------80
// filename: SnackAttack.ino
// date: 7/1/12
// author: Charles C. Shortlidge
// description: Arduino sketch associated with the Snack Attack mobile vending
//  machine robot application
//
//  motor 1 = left motor
//  motor 2 = right motor
//----------------------------------------------------------------------------80

//#define BENCH_ENV 0x01

#include <avr/interrupt.h>  
#include <avr/io.h>

// macros
#define BTF(x,b)	((x) ^= (1 << (b)))		// Bit toggle
#define BCF(x,b)	((x) &= ~(1 << (b)))		// Bit clear
#define BSF(x,b)	((x) |= (1 << (b)))		// Bit set
#define BTEST(x,b)	(!!((x) & (1 << (b))))  	// Bit test

#define	max(a,b)	(((a) > (b)) ? (a) : (b))
#define	min(a,b)	(((a) < (b)) ? (a) : (b))

// function prototypes
int respHandler(void);

volatile int status_config = 0;
volatile int status_init = 0;
volatile int status_exec = 0;
volatile int status_rt = 0;
volatile int status_bkgnd = 0;
volatile unsigned int statusind = 0;

unsigned int uiTimer = 0;
unsigned int uiTimerMin = 0xFFFF;
unsigned int uiTimerMax = 0;
float fTime = 0.;

int iLedPinNum = 13;
int iLedState = 0;

int iAdPinNum = 0;
int iAdPinVal = 0;

int iMtrCtrlEnablePinNum = 12;
int iMtrCtrlEnablePinVal = 0;

int iMtrCmdModeSel = 0;

float fMtrCmdAmp = 750.;
float fMtrCmdFreq = 0.25;

int iMtr1Cmd = 0;
int iMtr2Cmd = 0;
char cMtrCmds[16] = "";

char cDataBuf[128] = "";

int iSerCmd = 0;
//char cSerCmdResp[32] = "";

char cRespBuf[32] = "";
char cRespByte = 0x00;
int iRespBufLen = 32;
int iRespComplete = 0;
int iRespLen = 0;

int iVdr = 0;
int iVmot = 0;
int iV5out = 0;
int iMtrCtrlFaultWord = 0;

void setup() {                
  // initialization

  // pin 13 has an LED connected on most Arduino boards
  // initialize the digital pin as an output.
  pinMode(iLedPinNum, OUTPUT);
  pinMode(iMtrCtrlEnablePinNum, OUTPUT);

  // configure serial port 0
  // shared with USB connector. accordingly, do not use it unless we have to
  Serial.begin(9600);

  // configure serial port 1
  // use for communication with motor controller
  Serial1.begin(115200);

  // configure serial port 2
  // use for communication with laptop, e.g. monitoring, data logging
  Serial2.begin(38400);

  // configure and start timers
  // perform this task just before exiting initialization logic, i.e. about to enter real-time mode

  // configure timer 1 for normal mode
  // TOV1 flag will be cleared by the associated ISR
  TCCR1A = 0x00;
  TCCR1C = 0x00;
  TCCR1B = TCCR1B | 0x03;       // prescaler = 64
  TCNT1H = 0xCF;                // reset timer
  TCNT1L = 0x2C;
//  TIMSK1 = TIMSK1 & 0xFE;       // disable inerrupt
  TIMSK1 = TIMSK1 | 0x01;       // enable interrupt

  // configure timer 2 for normal mode
  // TOV2 flag will be cleared by the associated ISR
  // timer 2 overflow interrupt vector handler called (16x10^6/256)/1024 = 61.035 times per second
  TCCR2A = 0x00;
  TCCR2B = TCCR2B | 0x07;       // prescaler = 1024
  TCNT2 = 0x00;                 // reset timer
  TIMSK2 = TIMSK2 & 0xFE;       // disable interrupt
//  TIMSK2 = TIMSK2 | 0x01;       // enable interrupt
}

int realtime(){
  // real-time logic. executes once every 0.05 s

  int i = 0;
  int iTmp = 0;

  if(statusind > 25){
    // Arduino has been powered-up for several minor frames
    // the rest of the system, e.g. motor controller should be powered up as well
    // set flag to indicate to the downstream logic that system is now initialized
    status_init = status_init || 0x01;
  }

  // keep track of time elapsed since last MCU reset
  fTime += 0.05;

// to indicate that the MCU is alive, toggle the state of the LED attached to pin 13 once every 1 s
//  if((statusind % 61) == 0){    // if using timer 2
  if((statusind % 20) == 0){      // if using timer 1
//    statusind = 0;
    if(iLedState == 0){
      iLedState = 1;
    }else{
      iLedState = 0;
    }
  }

  // input signal management:
  // read inputs, e.g. A/Ds, digital inputs, commands/messages received over serial port
  // validate inputs
  // detect, declare, and accomodate any signal faults
  // e.g. read AD ch 0 with pin connected to on-board 3.3 V
  iAdPinVal = analogRead(iAdPinNum);

  // read data from the serial port
  // temporarily deactivate the following so that can test logic for reading data from motor controller
  // usinf serial port 0
  if(Serial.available() > 0) {
    iSerCmd = Serial.read();
    // the following is to support debugging
//    sprintf(cSerCmdResp, "iSerCmd = %i\r", iSerCmd);
//    Serial.print(cSerCmdResp);
  }

  // read responses from motor controller
  // process complete responses, i.e. series of characters terminated by '\r'
//  while(Serial.available() > 0){
  while(Serial1.available() > 0){
//    cRespByte = Serial.read();
    cRespByte = Serial1.read();
    if(cRespByte == '\r'){
      iRespComplete = 1;
    }
    if((cRespByte != -1) & (iRespComplete == 0)){
      // response is not yet completely received and
      // new byte has been received. append the new byte to the array of bytes previously accumulated
      cRespBuf[iRespLen] = cRespByte;
      iRespLen++;
    }
    if(!(iRespLen < iRespBufLen)){
      // buffer is too small to handle the incoming response
      // abort
      for(i = 0; i < iRespBufLen; i++){
        cRespBuf[i] = '\0';
      }
      iRespLen = 0;
    }
    if(iRespComplete != 0){
      // call command response handler
      // for now, just send the data back to the sender
//      Serial.write(cRespBuf);
      respHandler();
//      Serial.print(iVmot, DEC);
//      Serial.print(iMtrCtrlFaultWord, DEC);
      // prepare for receiving and processing a new response
      // clear buffer amd reset meta data
      for(i = 0; i < iRespBufLen; i++){
        cRespBuf[i] = '\0';
      }
      iRespLen = 0;
      iRespComplete = 0;
    }
  }

  // if a user command was received over the serial port, then set corresponding mode for calculating the
  // commands to send to the motor controller
  switch(iSerCmd){
    case 83:
      // 'S': sinusoidal
      iMtrCmdModeSel = 1;
      break;
    case 97:
      // 'a': autonomous
      iMtrCmdModeSel = 0;
      break;
    case 98:
      // 'b': backwards
      iMtrCmdModeSel = 11;
      break;
    case 102:
      // 'f': forward
      iMtrCmdModeSel = 10;
      break;
    case 108:
      // 'l': left
      iMtrCmdModeSel = 12;
      break;
    case 114:
      // 'r': right
      iMtrCmdModeSel = 13;
      break;
    case 115:
      // 's': stop
      iMtrCmdModeSel = 14;
      break;
    default:
      break;
  }

  // operational logic: 
  // line-following,
  // collision avoidance using distance sensors,
  // emergency stop activation based on state of bumpers and buttons that may be pressed by a human
  // detect, declare, and accomodate any operational faults, e.g. motor wheel leaving the ground

  switch(iMtrCmdModeSel){
    case 0:
      // autonomous mode
      // compute commands to implement line-following
      // for now, set commands consistent with zero speed
      iMtr1Cmd = 0;
      iMtr2Cmd = 0;
      break;
    case 1:
      // set sinusoidal values
      iMtr1Cmd = (int)(fMtrCmdAmp*sin(2.*3.1416*fMtrCmdFreq*fTime));
      iMtr2Cmd = (int)(fMtrCmdAmp*sin(2.*3.1416*fMtrCmdFreq*fTime));
      break;
    case 10:
      // forward
      iMtr1Cmd = 750;
      iMtr2Cmd = 750;
      break;
    case 11:
      // backwards
      iMtr1Cmd = -750;
      iMtr2Cmd = -750;
      break;
    case 12:
      // left
      iMtr1Cmd = -750;
      iMtr2Cmd = 750;
      break;
    case 13:
      // right
      iMtr1Cmd = 750;
      iMtr2Cmd = -750;
      break;
    case 14:
      // stop
      iMtr1Cmd = 0;
      iMtr2Cmd = 0;
      break;
    default:
      // error
      // set commands consistent with zero speed
      iMtr1Cmd = 0;
      iMtr2Cmd = 0;
      break;
  }
  // construct command to be sent to the motor controller
  // command value associated with motor 1 must be multiplied by -1 due to encoder polarity issue
  iTmp = -1*iMtr1Cmd;
  sprintf(cMtrCmds, "!M %i %i\r", iTmp, iMtr2Cmd);
      
  // decide if motor control board should be disabled/enabled
  // for now, set to enabled
  iMtrCtrlEnablePinVal = 1;

  // output signal management:
  // write messages to the motor controller
  // write status data to external computer/data logger
  // turns out that the Arduino buffers data to be transmitted through each serial port. accordingly,
  // do not need to have the background task do this
  // etc.

  if(iLedState == 0){
    digitalWrite(iLedPinNum, LOW);    // turn the LED off
  }else{
    digitalWrite(iLedPinNum, HIGH);   // turn the LED on
  }

  // disable/enable motor control board
  if(iMtrCtrlEnablePinVal == 0){
    digitalWrite(iMtrCtrlEnablePinNum, LOW);
  }else{
    digitalWrite(iMtrCtrlEnablePinNum, HIGH);
  }

  if((BTEST(status_init, 0)) & (!(BTEST(status_config, 0)))){
    // send configuration commands to motor controller

    // disable echoing of characters sent to the motor controller
    // this will reduce load on logic for processing messages received from the motor controller
    Serial1.print("^ECHOF 1\r");

    // set flag to indicate that configuration commands have been sent to the motor controller
    // accordingly, the configuration commands will not be sent again
    status_config = status_config | 0x01;
  }

  if((BTEST(status_init, 0)) & ((statusind % 2) == 0)){
    // send motor commands to motor controller

    // based on test completed so far, for some reason, the motor controller does not respond to
    // commands if they are sent faster than once every other minor frame, i.e. 0.100 s. would like
    // to figure this out, but for now, proceed to send commands only every other minor frame
    Serial1.print(cMtrCmds);
  }

  if((BTEST(status_init, 0)) & ((statusind % 20) == 0)){
    // send queries to motor controller
    Serial1.print("?FF 1\r");
//    Serial1.print("?FF 2\r");
    Serial1.print("?V\r");
  }

  if((statusind % 5) == 0){
    // send data to laptop
    // size of cDataBuf is 128 bytes. do not attempt to load with more than 127 bytes. otherwise, null termination
    // byte will be overwritten such that library functions do not work properly. also, data written beyond 128
    // bytes will corrupt RAM

    // load the buffer and transmit the buffer
    // note that the Arduino library implementation of sprintf does not support floats
    sprintf(cDataBuf, "%u,%u,%u,%u,%i,%i,%i,%i,%i\r", 
      statusind, uiTimer, uiTimerMin, uiTimerMax, 
      iAdPinVal, 
      iMtr1Cmd, iMtr2Cmd,
      iVmot, iMtrCtrlFaultWord);
    Serial.print(cDataBuf);
  }

  // capture the current state of the real-time timer
  noInterrupts();
  uiTimer = TCNT1;
  interrupts();

  // keep track of the min and max timer values
  if(uiTimer < uiTimerMin){
    uiTimerMin = uiTimer;
  }
  if(uiTimer > uiTimerMax){
    uiTimerMax = uiTimer;
  }

  // clear flag for executive to execute real-time task
  BCF(status_rt, 0);
  // set flag indicating that real-time task execution is complete
  BSF(status_rt, 1);

  return 0;
}

int background(){

  // execute any tasks that are not time sensitive and might take a long time to complete
  // transmission of large amount of status data
  // handling input commands from an external computer
  // etc.

  // clear flag for executive to execute background task
  BCF(status_bkgnd, 0);
  // set flag indicating that background task execution is complete
  BSF(status_bkgnd, 1);

  return 0;
}

int respHandler(){
  // determine the type of motor controller response
  // and parse the data based on the expected format

  if(cRespBuf[0] == 'V'){
//    Serial.println("Handling V resp type ...");
    sscanf(cRespBuf, "V = %i : %i : %i", &iVdr, &iVmot, &iV5out);
  }
  if(cRespBuf[0] == 'F'){
//    Serial.println("Handling FF resp type ...");
    sscanf(cRespBuf, "FF = %i", &iMtrCtrlFaultWord);
  }

  return 0;
}

void loop() {
  int itmp = 0;
  // executive loop
  // execute real-time and background tasks sequentially
  // when ISR resets status_rt<0> and status_bkgnd<0>, exit tight loop to
  // beginning of executive loop

  // test real-time logic status and execute, as required, real-time logic
  if(BTEST(status_rt, 0)) itmp = realtime();
  //if(BTEST(uctmp, 0)) BSF(faultind[0], 6);

  // test background logic status and execute, as required, background logic
  if(BTEST(status_bkgnd, 0)) itmp = background();
  //if(BTEST(uctmp, 0)) BSF(faultind[0], 6);

  // clear flag for executive to execute main loop
  BCF(status_exec, 0);
  // set flag to indicate that executive task execution is complete
  BSF(status_exec, 1);

  while(!BTEST(status_exec, 0)){
    //delay(1000);
  }
}

ISR(TIMER1_OVF_vect){
  // clear interrupt flag. specs indicate that the flag is cleared by hardware as part of call to the ISR.
  // accordingly, probably do not need to do this
//  TIFR1 = TIFR1 & 0xFE;

  //reset timer
  TCNT1H = 0xCF;
  TCNT1L = 0x2C;

  // increment statusind to measure the progrssion of time
  statusind++;

  // clear flags associated with indication that the various tasks have completed execution
  BCF(status_exec, 1);
  BCF(status_rt, 1);
  BCF(status_bkgnd, 1);

  // set flags associated with indication that various tasks should be executed
  BSF(status_exec, 0);
  BSF(status_rt, 0);
  BSF(status_bkgnd, 0);
}

ISR(TIMER2_OVF_vect){
  // clear interrupt flag. specs indicate that the flag is cleared by hardware as part of call to the ISR.
  // accordingly, probably do not need to do this
//  TIFR2 = TIFR2 & 0xFE;

  // reset timer
  // since timer rolled over to 0, do not need to reset the timer
  // in fact, resetting the timer now, after several instructions/MCU cycles have executed since the rollover
  // event will tend to skew the timing logic
//  TCNT2 = 0;

  // increment statusind to measure the progrssion of time
//  statusind++;

  // clear flags associated with indication that the various tasks have completed execution
  BCF(status_exec, 1);
  BCF(status_rt, 1);
  BCF(status_bkgnd, 1);

  // set flags associated with indication that various tasks should be executed
  BSF(status_exec, 0);
  BSF(status_rt, 0);
  BSF(status_bkgnd, 0);
}

//----------------------------------------------------------------------------80
// end of file
