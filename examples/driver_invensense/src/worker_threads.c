#include <modules/worker_thread/worker_thread.h>

WORKER_THREAD_SPAWN(lpwork_thread, LOWPRIO, 1024)
WORKER_THREAD_SPAWN(can_thread, LOWPRIO, 1024)
