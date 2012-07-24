typedef struct _Task {
  volatile uint8_t status;
  int (* taskMethod)();
} Task;


typedef enum _TaskStatus {
  pending = 0x01
} TaskStatus;


#define CLEAR_STATUS(t, statusBit)  ( (t).status &= ~(statusBit) )
#define SET_STATUS(t, statusBit)    ( (t).status |=  (statusBit) )
#define HAS_STATUS(t, statusBit) (!!( (t).status &   (statusBit) ))


typedef struct _ResponseBuffer {
  char buffer[32];
  unsigned int length;
  bool isComplete;
} ResponseBuffer;


typedef enum _MotorCommand {
  sinusoidal,
  autonomous,
  backward,
  forward,
  right,
  left,
  stop
} MotorCommand;


