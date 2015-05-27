/*
 * Copyright (C) ROland
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/read_matrix_serial/read_matrix_serial.c"
 * @author ROland
 * reads from the serial
 */

#include "modules/read_matrix_serial/read_matrix_serial.h"
#include "subsystems/datalink/telemetry.h"
#include <stdio.h>
#include <sys/fcntl.h>
#include <math.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <inttypes.h>
#include <serial_port.h>
#define SIZE_OF_ONE_IMAGE 152



char psResponse[2*SIZE_OF_ONE_IMAGE];
double n;
uint8_t MATRIX_WIDTH = 24;

struct termios tty;
int spot=0;

uint8_t lineBuffer[96];
struct SerialPort *port;
uint8_t response[SIZE_OF_ONE_IMAGE*2];

static void send_distance_matrix(void) {
	DOWNLINK_SEND_DISTANCE_MATRIX(DefaultChannel, DefaultDevice, sizeof lineBuffer, lineBuffer);
 }

int search_start_position(int startPosition, int size_of_one_image, uint8_t* raw){
    int sync=0;
    // Search for the startposition
    for (int i=startPosition; i < size_of_one_image-1; i++){
    	printf("Now ckecing startpos: %d , has: %d\n", i, (uint8_t)raw[i]);
        if ((raw[i] == 255) && (raw[i + 1] == 0) && (raw[i + 2] == 0)){
        	printf("Possible end of image: %d", i);
            if (raw[i + 3] == 171){
                sync = i;
                break;
            }
        }
    }
    return sync;
}

void serial_init(void) {
	printf("Init serial\n");
	memset(response, '\0', sizeof response);
	port = serial_port_new();
	speed_t speed = B1000000;
	int result=serial_port_open_raw(port,"/dev/ttyUSB0",speed);
	printf("Result open: %d", port->fd);

	register_periodic_telemetry(DefaultPeriodic, "DISTANCE_MATRIX", send_distance_matrix);

//	close(USB_INPUT);
	printf("\nEnd init serial");
}
void serial_update(void) {
	printf("---Reading read distance matrix-----\n");
	int n=0;
	int fd = port->fd;
	char buf = '\0';
	int skippedStuff =0;
	int tried=0;
	// Read everything
	do {
	   n = read( fd, &buf, 1 );
	   tried++;

	   if (n > 0){
		   spot++;
		   sprintf( &response[spot], "%c", buf );
	   }
	} while((tried<SIZE_OF_ONE_IMAGE) && (spot < sizeof response-2));


	if(spot>(2*SIZE_OF_ONE_IMAGE-2))
	{
		spot=0;
		int startPos = search_start_position(0,SIZE_OF_ONE_IMAGE,response);
		printf("Found startpos: %d ", startPos);
		int arrayIndex = 0;
		for (int x = 0; x < sizeof lineBuffer; x++)
		{
		  lineBuffer[x] = 0;
		}


		for (int i = startPos; i < SIZE_OF_ONE_IMAGE + startPos;i++){
			if ((response[i] == 255) && (response[i + 1] == 0) && (response[i + 2] == 0)){
				if (response[i + 3] == 128){
				// Start Of Line
					int startOfBuf = i + 4;
					int endOfBuf = (i + 4 + MATRIX_WIDTH);
					for(int indexInBuffer = startOfBuf; indexInBuffer < endOfBuf; indexInBuffer++){
						lineBuffer[arrayIndex] = response[indexInBuffer];
						arrayIndex++;
					}
				}
			}
		}

		for (int x = 0; x < sizeof lineBuffer; x++)
		{
		  printf("%d\n",lineBuffer[x]);
		}
	}

}
void serial_start(void)
{
	//printf("serial start\n");
}
