#ifndef AUTOPILOT_H
#define AUTOPILOT_H

typedef void *                      ORBHANDLE;

#define ORBERROR                    NULL
#define ORBVIAFS                    0

#ifdef ORBVIAFS
#include <linux/objmem.h>
#define sys_orb_create(x, y, z)     (ORBHANDLE) object_memory_create(x, y, z)
#define sys_orb_publish(x, y, z)    object_memory_publish(x, y, z)
#define sys_orb_close(x)            object_memory_close(x)
#endif

#endif // AUTOPILOT_H
