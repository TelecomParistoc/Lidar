#ifndef SLAM_H
#define SLAM_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint16_t distance;
	bool valid;
} slam_measure_t;

#endif
