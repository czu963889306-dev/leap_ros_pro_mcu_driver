#include "microros_runtime.h"

void microros_task(void *p) {
    (void)p;

    microros_runtime_loop();
}
