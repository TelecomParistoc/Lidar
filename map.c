#include "map.h"
#include "utils.h"
#include <math.h>
#ifdef TEST
#include <stdio.h>
#endif

#define ROBOT_LENGTH 200 // in mm
#define ROBOT_WIDTH 120 // in mm

int corner_angle;

int findNextValidIndex(slam_measure_t map[], int cur_index) {
  int i = 1;
  while (!map[(cur_index + i) % MAP_SIZE].valid)
    i++;

  return (cur_index + i) % MAP_SIZE;
}

void clean_data(slam_measure_t map[]) {
  bool invalid_data = false;
  int nextIndex;
  int curIndex = 0;
  int check_curIndex;
  int check_nextIndex;

  while (true) {
    invalid_data = false;
    nextIndex = findNextValidIndex(map, curIndex);
    if (ABS(map[nextIndex].distance - map[curIndex].distance) > DISCONTINUITY_THRESHOLD) {
#ifdef TEST
      printf("Discontinuity between %d and %d\n", curIndex, nextIndex);
#endif
      check_nextIndex = nextIndex;
      for (int j = 0; j < CONTINUITY_CHECK_WINDOW; j++) {
        check_curIndex = check_nextIndex;
        check_nextIndex = findNextValidIndex(map, check_curIndex);
        if (ABS(map[check_curIndex].distance - map[check_nextIndex].distance) > DISCONTINUITY_THRESHOLD) {
          invalid_data = true;
          break;
        }
      }

      if (invalid_data) {
        map[nextIndex].valid = false;
      }
    }

    if (!invalid_data) {
      nextIndex = findNextValidIndex(map, curIndex);
      if (curIndex < nextIndex)
        curIndex = nextIndex;
      else
        break;
    }
  }
}

void init_robot(void) {
  corner_angle = atan(ROBOT_LENGTH/ROBOT_WIDTH);
}

int get_robot_border(int angle) {
  if (angle > 360) {
    return -1;
  }

  // Robot is a rectangle

  if ((angle < corner_angle) || (angle > (360 - corner_angle))) {
    return (ROBOT_LENGTH / 2 * cos(angle));
  } else if (angle < (180 - corner_angle)) {
    return (ROBOT_WIDTH / 2 * sin(angle));
  } else if (angle < (180 + corner_angle)) {
    return -(ROBOT_LENGTH / 2 * cos(angle));
  } else if (angle < (360 - corner_angle)) {
    return -(ROBOT_WIDTH / 2 * sin(angle));
  }

  return -1;
}

int detect_collision(slam_measure_t *map) {
  if (map == NULL) {
    return -1;
  }

  for (int i = 0; i < MAP_SIZE; i++) {
    if (map[i].valid) {
      if (get_robot_border(i) + MARGIN < map[i].distance) {
          return 1; // Imminent collision
      }
    }
  }

  return 0; // No collision
}
