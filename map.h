#ifndef MAP_H
#define MAP_H

#include "slam.h"

#define MAP_SIZE 360
#define DISCONTINUITY_THRESHOLD 50 /* in mm */
#define CONTINUITY_CHECK_WINDOW 3 /* in nb of samples */
#define MIN_VALID_DATA_NB 220 /* in nb of samples */

int findNextValidIndex(slam_measure_t map[], int cur_index);

#endif /* MAP_H */
