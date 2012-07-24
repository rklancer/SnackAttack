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

/*
  Header file with some user-defined types. Certain user-defined types
  (particularly ones with function pointers) must go in a header file because
  the Arduino IDE attempts to extract function prototypes incorrectly.
  See http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1264694180
*/
#include "SnackAttack.h"

const float pi = 3.141592654;

/*
  Configuration constants
*/
const int ticksPerSecond = 20;
const float fMtrCmdAmp = 750.;
const float fMtrCmdFreq = 0.25;

const int iAdPinNum = 0;
const int iLedPinNum = 13;
const int iMtrCtrlEnablePinNum = 12;

/*
  System State
*/
volatile bool isConfigured = false;
volatile bool isInitialized = false;
volatile unsigned int ticks = 0;

/*
  Values read in from elsewhere
*/
ResponseBuffer motorControllerResponse = {
  "",
  0,
  false
};

unsigned int uiTimer = 0;
unsigned int uiTimerMin = 0xFFFF;
unsigned int uiTimerMax = 0;

int iAdPinVal = 0;
int iVdr = 0;
int iVmot = 0;
int iV5out = 0;
int iMtrCtrlFaultWord = 0;

/*
  Values and system state to be output
*/
boolean bLedState = false;

MotorCommand motorCommand;
bool mtrCtrlEnablePinVal = 0;
int iMtr1Cmd = 0;
int iMtr2Cmd = 0;
char cMtrCmds[16] = "";

/*
  Functions and their associated Tasks
*/
void setup(void);
void loop(void);
void startTimers(unsigned int ticksPerSecond);
void runIfPending(Task *t);

void readMotorControllerResponse(void);
void handleMotorControllerResponse(void);

void constructMotorCommandString(MotorCommand command);

int realtime(void);

Task realtimeTask = {
  0, &realtime
};

int background(void);

Task backgroundTask = {
  0, &background
};

int executive(void);

Task executiveTask = {
  0, &executive
};


/**
  Standard Arduino initialization callback
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
  startTimers(ticksPerSecond);
}


/**
  Setup timer 1 to call trigger the TIMER1_COMPA interrupt ticksPerSecond
  times per second.
*/
void startTimers(unsigned int ticksPerSecond) {
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
  OCR1A = (16000000 / 64) / ticksPerSecond - 1;

  // Turn on Clear Timer on Compare Match mode.
  // This causes a timer interrupt to be triggered whenever the Timer 1 counter equals the
  // value in OCR1A. (One subsequent cycle is used to reset the Timer 1 counter)
  TCCR1B |= (1 << WGM12);

  // Finally, enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);

  // re-enable interrupts
  sei();
}


/**
  This interrupt service routine is called by the microprocessor when
  Timer 1, as initialized by startTimer, "fires".
*/
ISR(TIMER1_COMPA_vect) {
  // Increment `ticks` to measure the progression of time
  ticks++;

  // For now, we'll execute every task once per tick.
  SET_STATUS(executiveTask,  pending);
  SET_STATUS(realtimeTask,   pending);
  SET_STATUS(backgroundTask, pending);
}


/**
  The main loop just executes any tasks that need to be run and returns.
*/
void loop() {
  // Note that Arduino's main() busy waits: it sets up an infinite loop which calls our
  // loop(). Therefore, there is little to be gained by not returning from loop()
  // immediately after checking for tasks.
  runIfPending(&executiveTask);
  runIfPending(&backgroundTask);
  runIfPending(&realtimeTask);
}


/**
  If Task t is marked pending, execute it.
  Checks and clears pending status atomically.
*/
void runIfPending(Task *t) {
  cli();

  if (HAS_STATUS(*t, pending)) {
    CLEAR_STATUS(*t, pending);
    sei();
    t->taskMethod();
  }
  else {
    sei();
  }
}


/**
  Append motor controller response (on Serial1) to buffer, callback to
  handleMotorControllerResponse if complete
*/
void readMotorControllerResponse(void) {
  // process complete responses, i.e. series of characters terminated by '\r'

  char responseByte;

  while (Serial1.available() > 0) {

    responseByte = Serial1.read();

    if (responseByte == '\r') {
      motorControllerResponse.buffer[motorControllerResponse.length++] = '\0';
      motorControllerResponse.isComplete = true;
      handleMotorControllerResponse();
      return;
    }
    else if (motorControllerResponse.length < (sizeof(motorControllerResponse.buffer) - 1)) {
      motorControllerResponse.buffer[motorControllerResponse.length++] = responseByte;
    }
    // Skip (truncate) characters that don't fit in the buffer.
  }
}

/**
  Handle completed motor controller response.
*/
void handleMotorControllerResponse(void) {
  // determine the type of motor controller response
  // and parse the data based on the expected format

  if (motorControllerResponse.buffer[0] == 'V') {
    // Serial.println("Handling V resp type ...");
    sscanf(motorControllerResponse.buffer, "V = %i : %i : %i", &iVdr, &iVmot, &iV5out);
  }

  if (motorControllerResponse.buffer[0] == 'F') {
    // Serial.println("Handling FF resp type ...");
    sscanf(motorControllerResponse.buffer, "FF = %i", &iMtrCtrlFaultWord);
  }

  motorControllerResponse.buffer[0] = '\0';
  motorControllerResponse.length = 0;
  motorControllerResponse.isComplete = false;
}


/**
  Set the string 'cMtrCmds' to the appropriate motor command string.
*/
void constructMotorCommandString(MotorCommand command) {
  // operational logic:
  // line-following,
  // collision avoidance using distance sensors,
  // emergency stop activation based on state of bumpers and buttons that may be pressed by a human
  // detect, declare, and accomodate any operational faults, e.g. motor wheel leaving the ground

  switch (command) {
    case autonomous:
      // autonomous mode
      // compute commands to implement line-following
      // for now, set commands consistent with zero speed
      iMtr1Cmd = 0;
      iMtr2Cmd = 0;
      break;
    case sinusoidal:
      // set sinusoidal values
      iMtr1Cmd = (int)(fMtrCmdAmp*sin(2.*pi*fMtrCmdFreq * (0.001 * (float)millis())));
      iMtr2Cmd = (int)(fMtrCmdAmp*sin(2.*pi*fMtrCmdFreq * (0.001 * (float)millis())));
      break;
    case forward:
      iMtr1Cmd = 750;
      iMtr2Cmd = 750;
      break;
    case backward:
      iMtr1Cmd = -750;
      iMtr2Cmd = -750;
      break;
    case left:
      iMtr1Cmd = -750;
      iMtr2Cmd = 750;
      break;
    case right:
      iMtr1Cmd = 750;
      iMtr2Cmd = -750;
      break;
    case stop:
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
  sprintf(cMtrCmds, "!M %i %i\r", -1*iMtr1Cmd, iMtr2Cmd);
}


/**
  Real-time logic. Executes once per timer tick.
*/
int realtime() {
  int i = 0;
  int iTmp = 0;

  if (millis() > 1250) {
    // Arduino has been powered-up for 1.25 seconds.
    // By now, the rest of the system, e.g. motor controller should be powered up as well.
    // set flag to indicate to the downstream logic that system is now initialized.
    isInitialized = true;
  }

  // Blink the LED with a 1s period to indicate that the MCU is alive.
  if (ticks % ticksPerSecond == 0) {
    bLedState = !bLedState;
  }

  readMotorControllerResponse();

  // if a user command was received over the serial port, then set corresponding mode for calculating the
  // commands to send to the motor controller
  switch ((char)Serial.read()) {
    case 'S':
      motorCommand = sinusoidal;
      break;
    case 'a':
      motorCommand = autonomous;
      break;
    case 'b':
      motorCommand = backward;
      break;
    case 'f':
      motorCommand = forward;
      break;
    case 'l':
      motorCommand = left;
      break;
    case 'r':
      motorCommand = right;
      break;
    case 's':
      motorCommand = stop;
      break;
    default:
      break;
  }

  constructMotorCommandString(motorCommand);

  // decide if motor control board should be disabled/enabled
  // for now, set to enabled
  mtrCtrlEnablePinVal = true;

  // output signal management:
  // write messages to the motor controller
  // write status data to external computer/data logger
  // turns out that the Arduino buffers data to be transmitted through each serial port. accordingly,
  // do not need to have the background task do this
  // etc.
  digitalWrite(iLedPinNum, bLedState ? HIGH : LOW);

  // disable/enable motor control board
  digitalWrite(iMtrCtrlEnablePinNum, mtrCtrlEnablePinVal ? HIGH : LOW);

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

  if (isInitialized && (ticks % ticksPerSecond) == 0) {
    // send queries to motor controller
    Serial1.print("?FF 1\r");
    // Serial1.print("?FF 2\r");
    Serial1.print("?V\r");
  }

  if ((ticks % 5) == 0) {
    char cDataBuf[128];
    // send data to laptop
    // size of cDataBuf is 128 bytes. do not attempt to load with more than 127 bytes. otherwise, null termination
    // byte will be overwritten such that library functions do not work properly. also, data written beyond 128
    // bytes will corrupt RAM

    int iAdPinVal = analogRead(iAdPinNum);

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
  if (uiTimer < uiTimerMin ){
    uiTimerMin = uiTimer;
  }
  if (uiTimer > uiTimerMax) {
    uiTimerMax = uiTimer;
  }

  return 0;
}

/*
  Background task.
  Execute any tasks that are not time sensitive and might take a long time to complete
  transmission of large amount of status data
  handling input commands from an external computer
  etc.
*/
int background() {
  return 0;
}


int executive() {
  return 0;
}

//----------------------------------------------------------------------------80
// end of file
