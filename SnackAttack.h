typedef struct _Task {
  volatile uint8_t status;
  int (* taskMethod)();
} Task;


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
