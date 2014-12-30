#ifndef ETSAIRSPEED_H
#define ETSAIRSPEED_H


/* I2C bus */
#define I2C_TRIES	3
#define ETSAIRSPEED_DRV_NAME "ets_airspeed"
#define ETS_PATH	"/dev/" ETSAIRSPEED_DRV_NAME

/* Register address */
#define READ_CMD	0x07	/* Read the data */

/**
 * The Eagle Tree Airspeed V3 cannot provide accurate reading below speeds of 15km/h.
 * You can set this value to 12 if you want a zero reading below 15km/h.
 */
#define MIN_ACCURATE_DIFF_PRES_PA 0

/* Measurement rate is 100Hz */
#define CONVERSION_INTERVAL	(1000000 / 100)	/* microseconds */

#define __ETSAIRSPEEDIOC		0x90

#define ETS_AIRSPEED_IOCTL_PREREAD		_IO(__ETSAIRSPEEDIOC, 1)

#endif // ETSAIRSPEED_H
