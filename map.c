#include "map.h"
#include "utils.h"
#if TEST
#include <stdio.h>
#endif

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
#if TEST
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
