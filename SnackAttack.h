typedef struct _Task {
  volatile uint8_t status;
  int (* taskMethod)();
} Task;

typedef enum _MotorCommand {
  sinusoidal,
  autonomous,
  backward,
  forward,
  right,
  left,
  stop
} MotorCommand;
