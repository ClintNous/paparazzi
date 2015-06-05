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
#include "read_matrix_serial.h"

uint8_t MATRIX_ROWS=6;
uint8_t SINGLE_MATRIX_COLUMNS=6;
uint8_t CAMERAS_COUNT=6;
uint8_t COMPLETE_MATRIX_WIDTH;

int SIZE_OF_ONE_IMAGE;
int SIZE_OF_BUFFER_TO_READ;


struct termios tty;

uint8_t *imageBuffer;
uint8_t *response;

uint8_t *lastReadStack;
int spot=0;

struct SerialPort *port;

static void send_distance_matrix(void) {
	for(int x=0; x < MATRIX_ROWS; x++)
	{
		DOWNLINK_SEND_DISTANCE_MATRIX(DefaultChannel, DefaultDevice, &x,COMPLETE_MATRIX_WIDTH, imageBuffer);
	}
	/*
	if(SIZE_OF_ONE_IMAGE>50){

		DOWNLINK_SEND_DISTANCE_MATRIX(DefaultChannel, DefaultDevice, &MATRIX_ROWS,50, imageBuffer);
	}
	else
	{
		DOWNLINK_SEND_DISTANCE_MATRIX(DefaultChannel, DefaultDevice, &MATRIX_ROWS,SIZE_OF_ONE_IMAGE, imageBuffer);
	}*/
 }

typedef struct ImageProperties{
	int positionImageStart;
	int lineLength;
	int lineCount;
} ImageProperties;


void allocateSerialBuffer(int widthOfImage, int heightOfImage)
{
	MATRIX_ROWS=heightOfImage;
	COMPLETE_MATRIX_WIDTH=widthOfImage;

	SIZE_OF_BUFFER_TO_READ=2*(widthOfImage+8)*heightOfImage+8; // Length of the complete image, including indicator bytes, two times (to make sure we see it)
	response=malloc(SIZE_OF_BUFFER_TO_READ * sizeof(uint8_t));
	memset(response, '\0', SIZE_OF_BUFFER_TO_READ);


	SIZE_OF_ONE_IMAGE=COMPLETE_MATRIX_WIDTH*MATRIX_ROWS;
	imageBuffer=malloc(SIZE_OF_ONE_IMAGE*sizeof(uint8_t));
	memset(imageBuffer, '\0', SIZE_OF_ONE_IMAGE);

}

int isEndOfImage(uint8_t *stack){
	if (stack[0] == 255 && (stack[1] == 0) && (stack[2] == 0) && stack[3]==171){
		return 1;
	}
	return 0;
}

ImageProperties search_start_position(uint8_t *raw, int size){
    int sync=0;
    ImageProperties imageProperties={-1,-1,-1,-1};
    int boolStartCounting=0;
    int startOfLine=0;
    // Search for the startposition of the image, the end of the image, and the width and height of the image
    for (int i=0; i < size-1; i++){
    	//printf("Now checking: %d \n",raw[i]);
    	// Check the first 3 bytes for the pattern 255-0-0, then check what special byte is encoded next
        if ((raw[i] == 255) && (raw[i + 1] == 0) && (raw[i + 2] == 0)){
        	//printf("Possible thing: %d\n", raw[i+3]);
            if (raw[i + 3] == 171 && imageProperties.positionImageStart >= 0){ // End of image
                sync = i;
                break;
            }
            if (raw[i + 3] == 175){ // Start of image
            	imageProperties.positionImageStart = i;
            	boolStartCounting=1;
            	imageProperties.lineCount=0;
            }
            if (raw[i + 3] == 128){ // Start of line
            	startOfLine = i;
			}
            if (raw[i + 3] == 218 && boolStartCounting==1){ // End of line
            	imageProperties.lineLength = i-startOfLine-4; // removed 4 for the indication bit
            	imageProperties.lineCount+=1;
			}
        }
    }

    return imageProperties;
}

void serial_init(void) {
	printf("Init serial\n");
	COMPLETE_MATRIX_WIDTH=SINGLE_MATRIX_COLUMNS*CAMERAS_COUNT;
	SIZE_OF_ONE_IMAGE=COMPLETE_MATRIX_WIDTH*MATRIX_ROWS;//camerasAmount*matrixColumns*matrixRows+4+matrixRows*8
	printf("Size of image is now: %d\n", SIZE_OF_ONE_IMAGE);

	allocateSerialBuffer(COMPLETE_MATRIX_WIDTH,MATRIX_ROWS);
	lastReadStack=malloc(4 * sizeof(uint8_t));
	memset(lastReadStack, 0, 4);

	port = serial_port_new();
	speed_t speed = B1000000;
	int result=serial_port_open_raw(port,"/dev/ttyUSB0",speed);
	printf("Result open: %d", port->fd);

	register_periodic_telemetry(DefaultPeriodic, "DISTANCE_MATRIX", send_distance_matrix);

	printf("\nEnd init serial");
}
void serial_update(void) {
	printf("---Reading read distance matrix-----\n");
	int n=0;

	char buf = '\0';
	int skippedStuff =0;
	int tried=0;
	// Read everything
	while((tried<SIZE_OF_ONE_IMAGE) && (spot < SIZE_OF_BUFFER_TO_READ)){
	   n = read(  port->fd, &buf, 1 );
	   tried++;
	   if (n > 0){
		   spot++;
		   sprintf( &response[spot], "%c", buf );
		   printf("Reading: %d \n",buf);

		   int locationInStack;
		   for (locationInStack=0; locationInStack<3; locationInStack++){
			   lastReadStack[locationInStack]=lastReadStack[locationInStack+1];
		   }
		   sprintf( &lastReadStack[3], "%c", buf );
		   if(isEndOfImage(lastReadStack)){
			   break;
		   }
	   }
	//} while((tried<SIZE_OF_ONE_IMAGE) && (spot < sizeof response-2));
	}
	printf("Now exceeded, as tried: %d < SIZE_OF_IMAGE %d and spot %d < SIZE_OF_BUFFER_TO_READ %d \n",tried,SIZE_OF_ONE_IMAGE, spot,SIZE_OF_BUFFER_TO_READ);

	//if(spot>(sizeof response-2))
	//if(spot>500)
	//if(spot>=SIZE_OF_BUFFER_TO_READ)
	if(isEndOfImage(lastReadStack))
	{
		spot=0;
		//printf("Now checking length of response: %d \n", sizeof response);
		ImageProperties imageProperties = search_start_position(response, SIZE_OF_BUFFER_TO_READ);
		printf("Found image properties, startpos: %d , width: %d, height: %d \n", imageProperties.positionImageStart, imageProperties.lineLength, imageProperties.lineCount);
		//536
		if(imageProperties.lineCount!=MATRIX_ROWS || imageProperties.lineLength != COMPLETE_MATRIX_WIDTH)
		{
			allocateSerialBuffer(imageProperties.lineLength,imageProperties.lineCount);
		}

		for (int x = 0; x < SIZE_OF_ONE_IMAGE; x++)
		{
			imageBuffer[x] = 0;
		}
		//lineBuffer=(uint8_t*)realloc(lineBuffer,4*(imageProperties.lineCount*imageProperties.lineLength));
		//lineBuffer=realloc(lineBuffer,20);

		int arrayIndex=0;
		int lineNumber=0;
		for (int i = imageProperties.positionImageStart; i < imageProperties.positionImageStart+(imageProperties.lineLength+8)*imageProperties.lineCount+8;i++){

			if ((response[i] == 255) && (response[i + 1] == 0) && (response[i + 2] == 0)){
				if (response[i + 3] == 128){
				// Start Of Line
					int startOfBuf = i + 4;
					int endOfBuf = (i + 4 + imageProperties.lineLength);
					for(int indexInBuffer = startOfBuf; indexInBuffer < endOfBuf; indexInBuffer++){

						imageBuffer[arrayIndex] = response[indexInBuffer];
						arrayIndex++;
					}
				}
			}
		}

		for (int x = 0; x < imageProperties.lineCount*imageProperties.lineLength; x++)
		{
		  printf(" ,%d ",imageBuffer[x]);
		  if (x%imageProperties.lineLength==0 && x>0){
			  printf("line end\n");
		  }
		}
	}

}
void serial_start(void)
{
	//printf("serial start\n");
}
