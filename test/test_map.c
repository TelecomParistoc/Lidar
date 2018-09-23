#include "test_map.h"
#include "map.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static slam_measure_t *read_map(const char *file_name) {
  FILE *fp;
  int index, distance;
  slam_measure_t *map = (slam_measure_t*)malloc(sizeof(slam_measure_t) * MAP_SIZE);
  if (map == NULL) {
    printf("Fail to allocate map into memory\n");
    return NULL;
  }

  // Initialize map
  for (int i = 0; i < MAP_SIZE; i++) {
    map[i].distance = 0;
    map[i].valid = false;
  }

  // Parse map file
  fp = fopen(file_name, "r");
  while (fscanf(fp, "%d %d\n", &index, &distance) != EOF) {
    map[index].distance = distance;
    map[index].valid = true;
  }

  fclose(fp);

  return map;
}

static void print_map(slam_measure_t *map, const char *file_name) {
  FILE *fp;
  if (map == NULL) {
    printf("Invalid map provided\n");
    return;
  }
  if (file_name == NULL) {
    printf("Invalid file name provided\n");
    return;
  }

  fp = fopen(file_name, "w");

  for (int i = 0; i < MAP_SIZE; i++) {
    if (map[i].valid)
      fprintf(fp, "%d %d\n", i, map[i].distance);
  }

  fclose(fp);
}

int test_map(const char *file_name) {
  slam_measure_t *map;
  char *output_file_name;

  if (file_name == NULL) {
    printf("Invalid file name\n");
    return 1;
  }

  output_file_name = (char*)malloc(strlen(file_name) + 10);
  strcpy(output_file_name, file_name);
  strcat(output_file_name, ".out");

  map = read_map(file_name);
  clean_data(map);
  print_map(map, output_file_name);

  free(map);
  free(output_file_name);

  return 0;
}
