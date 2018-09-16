#include "map.h"

int findNextValidIndex(slam_measure_t map[], int cur_index) {
  int i = 1;
  while (!map[(cur_index + i) % MAP_SIZE].valid)
    i++;

  return (cur_index + i) % MAP_SIZE;
}
