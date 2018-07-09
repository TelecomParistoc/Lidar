#ifndef XV11_CONSTANTS
#define XV11_CONSTANTS

#define XV11_DATA_FRAME_START 0xFA

#define XV11_PACKET_LENGTH 22

#define XV11_INDEX_OFFSET 0xA0
#define XV11_INDEX_MIN 0xA0
#define XV11_INDEX_MAX 0xF9

#define XV11_INVALID_DATA 0x80
#define XV11_STRENGTH_WARNING 0x40
#define XV11_DISTANCE_MASK 0x3F

#define XV11_TARGET_SPEED_RPM 300

typedef struct __attribute__ ((__packed__)) {
	uint16_t distance;
	bool invalid_data;
	bool strength_warning;
	uint16_t signal_strength;
} xv11_data_t;

typedef struct __attribute__ ((__packed__)) {
	uint8_t start;
	uint8_t index;
	uint16_t speed;
	xv11_data_t data[4];
	uint16_t checksum;
} xv11_packet_t;

#endif // XV11_CONSTANTS
