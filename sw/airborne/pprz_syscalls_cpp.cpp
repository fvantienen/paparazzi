#include <stdio.h>
#include <errno.h>

#pragma GCC diagnostic ignored "-Wmissing-declarations"

#ifndef USE_CHIBIOS_RTOS // already defined by chibios syscalls

#include "pprz_syscalls_cpp.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void _open_r(void){
  return;
}

void __cxa_pure_virtual() {
  return;
}

void _fini(void) {
  return;
}

#ifdef __cplusplus
}
#endif

#endif

#pragma GCC diagnostic pop
