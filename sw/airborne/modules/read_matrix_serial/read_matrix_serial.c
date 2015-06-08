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

speed_t speed = B1000000;

uint8_t MATRIX_ROWS=6;
uint8_t SINGLE_MATRIX_COLUMNS=6;
uint8_t CAMERAS_COUNT=6;
uint8_t COMPLETE_MATRIX_WIDTH;

int lengthBytesImage;
int lengthBytesInputArray;


typedef struct ImageProperties{
	int positionImageStart;
	int lineLength;
	int height;
} ImageProperties;

struct termios tty;

uint8_t *READimageBuffer;
uint8_t *serialResponse;

uint8_t *lastReadStack;
int writeLocationInput=0;

struct SerialPort *READING_port;
int messageArrayLocation=0;
static void READsend_distance_matrix(void) {
	DOWNLINK_SEND_DISTANCE_MATRIX(DefaultChannel, DefaultDevice, &messageArrayLocation,COMPLETE_MATRIX_WIDTH, READimageBuffer+messageArrayLocation*COMPLETE_MATRIX_WIDTH);
	messageArrayLocation= (messageArrayLocation+1)%MATRIX_ROWS;
 }



void READallocateSerialBuffer(int widthOfImage, int heightOfImage)
{
	MATRIX_ROWS=heightOfImage;
	COMPLETE_MATRIX_WIDTH=widthOfImage;

	lengthBytesInputArray=2*(widthOfImage+8)*heightOfImage+8; // Length of the complete image, including indicator bytes, two times (to make sure we see it)
	serialResponse=malloc(lengthBytesInputArray * sizeof(uint8_t));
	memset(serialResponse, '\0', lengthBytesInputArray);


	lengthBytesImage=COMPLETE_MATRIX_WIDTH*MATRIX_ROWS;
	READimageBuffer=malloc(lengthBytesImage*sizeof(uint8_t));
	memset(READimageBuffer, '\0', lengthBytesImage);

}

/**
 * Checks if the sequence in the array is equal to 255-0-0-171
 */
int READisEndOfImage(uint8_t *stack){
	if (stack[0] == 255 && (stack[1] == 0) && (stack[2] == 0) && stack[3]==171){
		return 1;
	}
	return 0;
}


ImageProperties READget_image_properties(uint8_t *raw, int size){
    int sync=0;
    ImageProperties imageProperties={-1,-1,-1,-1};
    int boolStartCounting=0;
    int startOfLine=0;
    // Search for the startposition of the image, the end of the image,
    // and the width and height of the image
    for (int i=0; i < size-1; i++){
    	// Check the first 3 bytes for the pattern 255-0-0, then check what special byte is encoded next
        if ((raw[i] == 255) && (raw[i + 1] == 0) && (raw[i + 2] == 0)){
            if (raw[i + 3] == 171 && imageProperties.positionImageStart >= 0){ // End of image
                sync = i;
                break;
            }
            if (raw[i + 3] == 175){ // Start of image
            	imageProperties.positionImageStart = i;
            	boolStartCounting=1;
            	imageProperties.height=0;
            }
            if (raw[i + 3] == 128){ // Start of line
            	startOfLine = i;
			}
            if (raw[i + 3] == 218 && boolStartCounting==1){ // End of line
            	imageProperties.lineLength = i-startOfLine-4; // removed 4 for the indication bit
            	imageProperties.height+=1;
			}
        }
    }

    return imageProperties;
}

void resetLastReadStack()
{
	for(int x=0; x < 4; x++){
		lastReadStack[x]=0;
	}
}
void serial_init(void) {
	printf("Init serial\n");
	COMPLETE_MATRIX_WIDTH=SINGLE_MATRIX_COLUMNS*CAMERAS_COUNT;
	lengthBytesImage=COMPLETE_MATRIX_WIDTH*MATRIX_ROWS;//camerasAmount*matrixColumns*matrixRows+4+matrixRows*8
	READallocateSerialBuffer(COMPLETE_MATRIX_WIDTH,MATRIX_ROWS);

	// Allocate the stack with zeros, to prevent random initialisation
	lastReadStack=malloc(4 * sizeof(uint8_t));
	resetLastReadStack();

	// Open the serial port
	READING_port = serial_port_new();
	int result=serial_port_open_raw(READING_port,"/dev/ttyUSB0",speed);
	printf("Result open: %d", READING_port->fd);

	register_periodic_telemetry(DefaultPeriodic, "DISTANCE_MATRIX", READsend_distance_matrix);

	printf("\nEnd init serial");
}

void READprintArray(uint8_t *toPrintArray, int totalLength, int width)
{
	for (int x = 0; x < totalLength; x++)
	{

	//  printf(" ,%d ",toPrintArray[x]);
	  if (x%width==0 && x>0){
		//  printf("line end\n");
	  }
	}
	for (int x = 0; x < totalLength; x++)
	{
		if(toPrintArray[x]>10){
			//printf(" ,%d ",toPrintArray[x]);
			 printf("Danger!");
			 // Set to -5 degrees

		}



	}
	printf("\n");
}

/**
 * Add a byte at the end of the stack, and move the other bytes forward (hereby losing the first byte)
 */
void READaddLastReadByteToStack(uint8_t* lastReadStack,char buf)
{
   int locationInStack;
   for (locationInStack=0; locationInStack<3; locationInStack++){
	   lastReadStack[locationInStack]=lastReadStack[locationInStack+1];
   }
   sprintf( &lastReadStack[3], "%c", buf );
}

void serial_update(void) {
	int n=0;
//	int fd = port->fd;
	char singleCharBuffer = '\0';

	int timesTriedToRead=0;

	// We want to read a complete image
	// Unfortunately the drone falls down if we wait too long in this function,
	// We therefore only try to read a certain amount of bytes
	while((timesTriedToRead<lengthBytesImage) && (writeLocationInput < lengthBytesInputArray) && !READisEndOfImage(lastReadStack)){
		//printf("Trying to read: ");
	   n = read(  READING_port->fd, &singleCharBuffer, 1 );

	   timesTriedToRead++;
	   if (n > 0){
		 //  printf("Read byte: %d \n", singleCharBuffer);
		   sprintf( &serialResponse[writeLocationInput++], "%c", singleCharBuffer );
		   READaddLastReadByteToStack(lastReadStack,singleCharBuffer);
	   }
	}

	// TODO: it is possible that the image is bigger than expected...
	// ... In that case we will exceed lengthBytesInputArray, and need to resize our buffers.
	if(READisEndOfImage(lastReadStack))
	{
		writeLocationInput=0;
		resetLastReadStack();
		//serial_port_flush(READING_port);


		// As we found a complete image we will now start writing at the start of the buffer again

		// Find the properties of the image by iterating over the complete image
		ImageProperties imageProperties = READget_image_properties(serialResponse, lengthBytesInputArray);
		printf("Found image properties, start position: %d , width: %d, height: %d \n", imageProperties.positionImageStart, imageProperties.lineLength, imageProperties.height);
		if(imageProperties.positionImageStart<0){
			return;
		}
		// Because image properties might change (when uploading new code to the multigaze), we need to resize arrays
		// and set the width and height variables
		if(imageProperties.height!=MATRIX_ROWS || imageProperties.lineLength != COMPLETE_MATRIX_WIDTH)
		{
			//READallocateSerialBuffer(imageProperties.lineLength,imageProperties.height);
			printf("Watch out, image not as expected");
		}

		// Remove all bytes that are indications of start and stop lines
		int imagePixelIndex=0;
		for (int i = imageProperties.positionImageStart; i < imageProperties.positionImageStart+(imageProperties.lineLength+8)*imageProperties.height+8;i++){

			if ((serialResponse[i] == 255) && (serialResponse[i + 1] == 0) && (serialResponse[i + 2] == 0)){
				if (serialResponse[i + 3] == 128){ // Start Of Line

					// Add four to the start and end positions to skip the first four bytes (255-0-0-128)
					int startOfBuf = i + 4;
					int endOfBuf = (i + 4 + imageProperties.lineLength);

					// Copy from the serialResponse in the imagebuffer
					// Hereby removing all bytes that indicate the start of images and lines
					for(int indexInBuffer = startOfBuf; indexInBuffer < endOfBuf; indexInBuffer++){
						READimageBuffer[imagePixelIndex] = serialResponse[indexInBuffer];
						imagePixelIndex++;
					}
				}
			}
		}

		READprintArray(READimageBuffer,imageProperties.height*imageProperties.lineLength,imageProperties.lineLength);

	}

}
void serial_start(void)
{
	//printf("serial start\n");
}
