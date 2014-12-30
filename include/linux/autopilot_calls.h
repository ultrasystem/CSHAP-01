#ifndef __AUTOPILOT_CALLS_H__
#define __AUTOPILOT_CALLS_H__

#ifdef __NR_syscalls
#define AUTOPILOT_CALL_BASE     __NR_syscalls
#else
#define AUTOPILOT_CALL_BASE     (__NR_kcmp + 1)
#endif

#define __NR_i2c_transfer       AUTOPILOT_CALL_BASE + 0
__SYSCALL(__NR_i2c_transfer, sys_i2c_transfer)
#define __NR_i2c_pack_transfer  AUTOPILOT_CALL_BASE + 1
__SYSCALL(__NR_i2c_pack_transfer, sys_i2c_pack_transfer)

#define AUTOPILOT_CALL_MAX      __NR_i2c_pack_transfer
#undef __NR_syscalls
#define __NR_syscalls           AUTOPILOT_CALL_MAX

#endif // __AUTOPILOT_CALLS_H__
