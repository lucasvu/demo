#ifndef __TEST_CONFIG_H
#define __TEST_CONFIG_H

#define MODE_SPI_BYPASS		0
#define MODE_NORMAL				1
#define MODE_SLEEP				2
#define MODE_CRC					3

typedef struct
{
	volatile unsigned char nOftrim;
	volatile unsigned char mode;
	volatile unsigned short unlock;
} TestConfig;

typedef struct
{
	volatile unsigned char value;
	volatile unsigned char address;
	volatile unsigned char rfu0;
	volatile unsigned char rfu1;
} TrimParams;

#endif	// __TEST_CONFIG_H

// bbb_comment
// bbb_comment
