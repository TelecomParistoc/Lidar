/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "xv11.h"
#include "slam.h"
#include <string.h>
#define PWM_FREQUENCY_KHZ 200000
#define PWM_MAX 100

static PWMConfig pwm_config_tim3 = {
    PWM_FREQUENCY_KHZ * PWM_MAX,
    PWM_MAX,
    NULL,
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_DISABLED, NULL},
        {PWM_OUTPUT_DISABLED, NULL},
        {PWM_OUTPUT_DISABLED, NULL}
    },
    0,
    0
};

static SerialConfig uartCfg2 = {115200, 0, 0, 0};
#define MAX_PENDING_PACKETS 10
static msg_t free_packets_msg[MAX_PENDING_PACKETS];
static msg_t packets_msg[MAX_PENDING_PACKETS];
static xv11_packet_t buffers[MAX_PENDING_PACKETS];
MAILBOX_DECL(free_packets_mb, free_packets_msg, MAX_PENDING_PACKETS);
MAILBOX_DECL(packets_mb, packets_msg, MAX_PENDING_PACKETS);

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

/*
 * @brief Compute the checksum of the packet.
 *
 * @param[in] packet Pointer to the packet to work on
 *
 * @return The computed checksum
 */
uint16_t compute_checksum(xv11_packet_t *packet) {
  uint32_t checksum = 0;
  uint8_t *dataPtr = (uint8_t*)packet;
  uint8_t index;

  for (index = 0; index < 10; index++) {
    checksum = (checksum << 1);
    checksum += ((uint32_t)(dataPtr[2*index]) + (((uint32_t)(dataPtr[2*index+1])) << 8));
  }

  checksum = ((checksum & 0x7FFF) + (checksum >> 15)) & 0x7FFF;

  return checksum;
}

static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
  (void)arg;
	uint8_t index;
	xv11_packet_t *packet;
	msg_t retCode = MSG_OK;
  uint8_t raw_data[4];

  chRegSetThreadName("lidar");

	// Acquire data
  while (true) {
    if (sdGet(&SD2) == XV11_DATA_FRAME_START) { // New data frame
      retCode = chMBFetch(&free_packets_mb, (msg_t*)&packet, TIME_IMMEDIATE);
			if (retCode == MSG_OK) {
				packet->index = sdGet(&SD2);

        if ((packet->index < XV11_INDEX_MIN) || (packet->index > XV11_INDEX_MAX)) { // Invalid index
          chMBPost(&free_packets_mb, (msg_t)packet, TIME_INFINITE);
          continue; // don't waste time trying to read an invalid packet
        }

				packet->speed = sdGet(&SD2);
				packet->speed |= sdGet(&SD2) << 8;

				for (index = 0; index < 4; index++) {
					sdRead(&SD2, raw_data, 4);
          xv11_parse_data(raw_data, &packet->data[index]);
				}

				packet->checksum = sdGet(&SD2);
				packet->checksum |= sdGet(&SD2) << 8;

        if((packet->index == XV11_INDEX_MIN) && !packet->data[0].invalid_data) {
          palTogglePad(GPIOA, GPIOA_LED_GREEN);
        }

        if (!packet->data[0].invalid_data) {
          sdPut(&SD2, packet->index - XV11_INDEX_OFFSET);
          sdPut(&SD2, packet->data[0].distance & 0xFF);
          sdPut(&SD2, (packet->data[0].distance & 0xFF00) >> 8);
      }

        // Check packet integrity
        if (packet->checksum != compute_checksum(packet)) {
          chMBPost(&free_packets_mb, (msg_t)packet, TIME_INFINITE);
          continue; // drop invalid packet
        }

				retCode = chMBPost(&packets_mb, (msg_t)packet, TIME_IMMEDIATE);
				if (retCode != MSG_OK) { // Fail to post packet
					memset(packet, 0, sizeof(xv11_packet_t));
					// Return it back to "free" pool
					chMBPost(&free_packets_mb, (msg_t)packet, TIME_INFINITE);
				}
			}
		}
  }
}

slam_measure_t map[360];

static THD_WORKING_AREA(waThread2, 128);
static THD_FUNCTION(Thread2, arg) {
	xv11_packet_t *packet;
	uint8_t dataIndex;
	msg_t retCode;
	bool mapComplete = false;

  (void)arg; // unused

	while(true) {
		retCode = chMBFetch(&packets_mb, (msg_t*)&packet, TIME_INFINITE);
		if (retCode == MSG_OK) {
			// Fill distance map
			for (dataIndex = 0; dataIndex < 4; dataIndex++) {
				if (!packet->data[dataIndex].invalid_data && !packet->data[dataIndex].strength_warning) {
					map[(packet->index - XV11_INDEX_OFFSET) * 4 + dataIndex].distance = packet->data[dataIndex].distance;
					map[(packet->index - XV11_INDEX_OFFSET) * 4 + dataIndex].valid = true;
				}
			}

			if (packet->index == XV11_INDEX_MAX) {
				mapComplete = true;
			}

			// Clear packet and send it back to "free" pool
			memset(packet, 0, sizeof(xv11_packet_t));
			chMBPost(&free_packets_mb, (msg_t)packet, TIME_INFINITE);
		}

		// Run analysis on full map
		if (mapComplete) {
			mapComplete = false;
			// TODO: extract connex components
		}
	}
}

/*
 * Application entry point.
 */
int main(void) {

  msg_t retCode;
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  palClearPad(GPIOA, GPIOA_LED_GREEN);
  palSetPad(GPIOB, GPIOB_PHA);

  /*
   * Activates the serial driver 2 using the driver default configuration.
   */
  sdStart(&SD2, &uartCfg2);
  pwmStart(&PWMD3, &pwm_config_tim3);
  pwmEnableChannel(&PWMD3, 0, 90);

  // Init free packet mailbox
  for (int i = 0; i < MAX_PENDING_PACKETS; i++) {
    retCode = chMBPost(&free_packets_mb, (msg_t)&buffers[i], TIME_INFINITE);
    if (retCode != MSG_OK) {
      palSetPad(GPIOA, GPIOA_LED_GREEN);
    }
  }

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
	chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

  while (true) {
  //  palTogglePad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(200);
  }
}
