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

#include <avr/interrupt.h>
#include <avr/io.h>

// It is necessary to put user-defined types in their own header file, because
// the Arduino IDE handles user-defined types incorrectly, see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1264694180
#include "task.h"

#define PENDING 0x01
#define CLEAR_PENDING(t) ((t).status &= ~PENDING)
#define SET_PENDING(t) ((t).status |= PENDING)
#define IS_PENDING(t) (!!((t).status & PENDING))

int executive(void);
int realtime(void);
int background(void);

Task executiveTask = {
  0, &executive
};

Task realtimeTask = {
  0, &realtime
};

Task backgroundTask = {
  0, &background
};

/**
  If Task t is marked pending, execute it.
  Checks and clears pending status atomically.
*/
void runIfPending(Task *t) {
  cli();

  if (IS_PENDING(*t)) {
    CLEAR_PENDING(*t);
    sei();
    t->taskMethod();
  }
  else {
    sei();
  }
}

#define TICKS_PER_SECOND 20

volatile bool isConfigured = false;
volatile bool isInitialized = false;
volatile unsigned int ticks = 0;

boolean bLedState = false;

const int iLedPinNum = 13;
const int iMtrCtrlEnablePinNum = 12;
const float pi = 3.141592654;

/**
  Do Arduino initialization here
*/
void setup() {

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

  // disable all interrupts before messing with timer registers
  cli();

  // Clear the Timer Counter/Control Registers for Timer 1:
  TCCR1A = 0;
  TCCR1B = 0;

  // Set bits CS10 and CS11 of timer control register TCCR1B.
  // This requests one count per 64 clock cycles.
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS11);

  // There are 16,000,000 / 64 counts per second
  // Divide by the number of timer interrupts per second to get the number of
  // counts per timer interrupt.
  // Set the compare-match register to that value, minus 1 to account for the 1
  // clock cycle used to clear the counter.
  OCR1A = (16000000 / 64) / TICKS_PER_SECOND - 1;

  // Turn on Clear Timer on Compare Match mode.
  // This causes a timer interrupt to be triggered whenever the Timer 1 counter equals the
  // value in OCR1A. (One subsequent cycle is used to reset the Timer 1 counter)
  TCCR1B |= (1 << WGM12);

  // Finally, enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);

  // re-enable interrupts
  sei();
}


void loop() {
  // Note that Arduino's main() busy waits: it sets up an infinite loop which calls our
  // loop(). Therefore, there is little to be gained by not returning from loop()
  // immediately after checking for tasks.
  runIfPending(&executiveTask);
  runIfPending(&backgroundTask);
  runIfPending(&realtimeTask);
}


ISR(TIMER1_COMPA_vect){
  // Increment `ticks` to measure the progression of time
  ticks++;

  // For now, we'll execute every task per tick.
  SET_PENDING(executiveTask);
  SET_PENDING(realtimeTask);
  SET_PENDING(backgroundTask);
}






// macros
#define BTF(x,b)  ((x) ^= (1 << (b)))   // Bit toggle
#define BCF(x,b)  ((x) &= ~(1 << (b)))    // Bit clear
#define BSF(x,b)  ((x) |= (1 << (b)))   // Bit set
#define BTEST(x,b)  (!!((x) & (1 << (b))))    // Bit test

#define max(a,b)  (((a) > (b)) ? (a) : (b))
#define min(a,b)  (((a) < (b)) ? (a) : (b))

// function prototypes
int respHandler(void);

unsigned int uiTimer = 0;
unsigned int uiTimerMin = 0xFFFF;
unsigned int uiTimerMax = 0;

int iAdPinNum = 0;
int iAdPinVal = 0;


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


int realtime() {
  // real-time logic. Executes TICKS_PER_SECOND times per second.

  int i = 0;
  int iTmp = 0;

  if (millis() > 1250) {
    // Arduino has been powered-up for 1.25 seconds.
    // the rest of the system, e.g. motor controller should be powered up as well
    // set flag to indicate to the downstream logic that system is now initialized
    isInitialized = true;
  }

  // Blink the LED with a 1s period to indicate that the MCU is alive.
  if (ticks % TICKS_PER_SECOND == 0) {
    bLedState = !bLedState;
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
      iMtr1Cmd = (int)(fMtrCmdAmp*sin(2.*pi*fMtrCmdFreq * (0.001 * (float)millis())));
      iMtr2Cmd = (int)(fMtrCmdAmp*sin(2.*pi*fMtrCmdFreq * (0.001 * (float)millis())));
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
  digitalWrite(iLedPinNum, bLedState ? HIGH : LOW);

  // disable/enable motor control board
  if(iMtrCtrlEnablePinVal == 0){
    digitalWrite(iMtrCtrlEnablePinNum, LOW);
  }else{
    digitalWrite(iMtrCtrlEnablePinNum, HIGH);
  }

  if (isInitialized && !isConfigured) {
    // send configuration commands to motor controller

    // disable echoing of characters sent to the motor controller
    // this will reduce load on logic for processing messages received from the motor controller
    Serial1.print("^ECHOF 1\r");

    // set flag to indicate that configuration commands have been sent to the motor controller
    // accordingly, the configuration commands will not be sent again
    isConfigured = true;
  }

  if (isInitialized && (ticks % 2) == 0) {
    // send motor commands to motor controller

    // based on test completed so far, for some reason, the motor controller does not respond to
    // commands if they are sent faster than once every other minor frame, i.e. 0.100 s. would like
    // to figure this out, but for now, proceed to send commands only every other minor frame
    Serial1.print(cMtrCmds);
  }

  if (isInitialized && (ticks % TICKS_PER_SECOND) == 0) {
    // send queries to motor controller
    Serial1.print("?FF 1\r");
//    Serial1.print("?FF 2\r");
    Serial1.print("?V\r");
  }

  if((ticks % 5) == 0){
    // send data to laptop
    // size of cDataBuf is 128 bytes. do not attempt to load with more than 127 bytes. otherwise, null termination
    // byte will be overwritten such that library functions do not work properly. also, data written beyond 128
    // bytes will corrupt RAM

    // load the buffer and transmit the buffer
    // note that the Arduino library implementation of sprintf does not support floats
    sprintf(cDataBuf, "%u,%u,%u,%u,%i,%i,%i,%i,%i\r",
      ticks, uiTimer, uiTimerMin, uiTimerMax,
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

  return 0;
}

int background() {

  // execute any tasks that are not time sensitive and might take a long time to complete
  // transmission of large amount of status data
  // handling input commands from an external computer
  // etc.

  return 0;
}

int executive() {
  return 0;
}

int respHandler() {
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


//----------------------------------------------------------------------------80
// end of file
