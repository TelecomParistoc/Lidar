#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define XV11_INVALID_DATA 0x80
#define XV11_STRENGTH_WARNING 0x40
#define XV11_DISTANCE_MASK 0x3F
#define ABS(x) (((x) > 0) ? (x) : (-(x)))

typedef struct __attribute__ ((__packed__)) {
	uint16_t distance;
	bool invalid_data;
	bool strength_warning;
	uint16_t signal_strength;
} xv11_data_t;

typedef struct __attribute__ ((__packed__)) {
	uint8_t index;
	uint16_t speed;
	xv11_data_t data[4];
	uint16_t checksum;
} xv11_packet_t;

typedef struct {
	uint16_t distance;
	bool valid;
} slam_measure_t;

#define MAP_SIZE 360
#define XV11_DATA_FRAME_START 0xFA
#define XV11_INDEX_OFFSET 0xA0
#define DISCONTINUITY_THRESHOLD 50 /* in mm */
#define CONTINUITY_CHECK_WINDOW 3 /* in nb of samples */
#define MIN_VALID_DATA_NB 110 /* in nb of samples */

char file_name[7] = {0};

/*
 * @brief Parse the raw data received into a C structure.
 *
 * @param[in] raw_data The binary data received from Lidar
 * @param[out] data Pointer to the C structure to fill
 */
void xv11_parse_data(uint8_t raw_data[4], xv11_data_t *data) {
  memset(data, 0, sizeof(xv11_data_t));

  if (raw_data[1] & XV11_INVALID_DATA)
  data->invalid_data = true;

  if (raw_data[1] & XV11_STRENGTH_WARNING)
  data->strength_warning = true;

  data->distance = raw_data[0] + ((raw_data[1] & XV11_DISTANCE_MASK) << 8);
  data->signal_strength = raw_data[3] + (raw_data[4] << 8);
}

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

int main(int argc, char* argv[]) {
  xv11_packet_t packet;
	uint8_t raw_data[21];
  slam_measure_t map[MAP_SIZE];
  uint8_t byte;
	uint8_t last_index = 0;
	FILE *output_file;
	uint8_t file_index = 0;
	FILE *plot = NULL;
	int valid_cnt = 0;

  if (argc != 2) {
    printf("Usage: %s <file name>", argv[0]);
    return 1;
  }

  FILE *fd = fopen(argv[1], "r");
  if (fd == NULL) {
    printf("Fail to open file %s", argv[1]);
    return 1;
  }

  while (1) {
    byte = 0;
		memset(raw_data, 0, sizeof(raw_data));
    fread(&byte, 1, 1, fd);
    if (byte == XV11_DATA_FRAME_START) {
      fread(raw_data, 1, sizeof(raw_data), fd);
			packet.index = (raw_data[0] - XV11_INDEX_OFFSET) * 4;
			packet.speed = (raw_data[1] + (raw_data[2] << 8)) >> 6;
			for (int data_index = 0; data_index < 4; data_index++) {
				xv11_parse_data(&raw_data[3 + (data_index * 4)], &packet.data[data_index]);
				if (!packet.data[data_index].invalid_data && !map[packet.index + data_index].valid) {
					map[packet.index + data_index].distance = packet.data[data_index].distance;
					map[packet.index + data_index].valid = true;
					valid_cnt++;
					printf("valid %d\n", valid_cnt);
				}
			}
    }

		if (valid_cnt > 120) {
			clean_data(map);
			sprintf(file_name, "%d.map", file_index);
			printf("Opening %s\n", file_name);
			output_file = fopen(file_name, "w");
			for (int i = 0; i < MAP_SIZE; i++) {
				if (map[i].valid)
					fprintf(output_file, "%d %d\n", i, map[i].distance);
			}
			fclose(output_file);
			file_index++;
			valid_cnt = 0;
			memset(map, 0, sizeof(map));
		}
		last_index = packet.index;
  }
  return 0;
}
