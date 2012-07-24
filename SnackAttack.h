typedef struct _Task {
  volatile uint8_t status;
  int (* taskMethod)();
} Task;
