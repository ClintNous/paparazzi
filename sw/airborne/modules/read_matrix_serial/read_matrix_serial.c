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

speed_t usbInputSpeed = B1000000;

#define PRINT_STUFF 0

uint8_t singleImageColumnCount=6;
uint8_t camerasCount=6;
uint8_t imageHeight=6;
uint8_t imageWidth;

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
int writeLocationInput=0;

struct SerialPort *READING_port;
int messageArrayLocation=0;
int widthToSend;
static void send_distance_matrix(void) {
	widthToSend=imageWidth;

	DOWNLINK_SEND_DISTANCE_MATRIX(DefaultChannel, DefaultDevice, &messageArrayLocation,36, READimageBuffer);
}



void allocateSerialBuffer(int widthOfImage, int heightOfImage)
{
	imageHeight=heightOfImage;
	imageWidth=widthOfImage;

//	lengthBytesInputArray=2*((widthOfImage+8)*heightOfImage+8); // Length of the complete image, including indicator bytes, two times (to make sure we see it)
	lengthBytesInputArray=50000;
	size_t sizeArrays=50000;
	serialResponse=malloc(sizeArrays * sizeof(uint8_t));
	memset(serialResponse, '0', sizeArrays);


	lengthBytesImage=50000;//COMPLETE_MATRIX_WIDTH*MATRIX_ROWS;
	READimageBuffer=malloc(sizeArrays*sizeof(uint8_t));
	memset(READimageBuffer, '0', sizeArrays);

}

/**
 * Checks if the sequence in the array is equal to 255-0-0-171,
 * as this means that this is the end of an image
 */
int isEndOfImage(uint8_t *stack){
	if (stack[0] == 255 && (stack[1] == 0) && (stack[2] == 0) && stack[3]==171){
		return 1;
	}
	return 0;
}

/**
 * Checks if the sequence in the array is equal to 255-0-0-171,
 * as this means a new image is starting from here
 */
int isStartOfImage(uint8_t *stack){
	if (stack[0] == 255 && (stack[1] == 0) && (stack[2] == 0) && stack[3]==175){
		return 1;
	}
	return 0;
}

ImageProperties get_image_properties(uint8_t *raw, int size, int startLocation){
    ImageProperties imageProperties={-1,-1,-1};
    int boolStartCounting=0;
    int startOfLine=0;
    // Search for the startposition of the image, the end of the image,
    // and the width and height of the image
    for (int i=startLocation; i < startLocation+size+10; i++){
    	// Check the first 3 bytes for the pattern 255-0-0, then check what special byte is encoded next
        if ((raw[i] == 255) && (raw[i + 1] == 0) && (raw[i + 2] == 0)){
            if (raw[i + 3] == 171 && imageProperties.positionImageStart >= 0){ // End of image
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
#if PRINT_STUFF
            	printf("Line length: %d \n", imageProperties.lineLength);
#endif
            	imageProperties.height+=1;
			}
        }
    }

    return imageProperties;
}

void serial_init(void) {
	imageWidth=singleImageColumnCount*camerasCount;
	lengthBytesImage=imageWidth*imageHeight;//camerasAmount*matrixColumns*matrixRows+4+matrixRows*8
	allocateSerialBuffer(imageWidth,imageHeight);

  
	// Open the serial port
	READING_port = serial_port_new();
	int result=serial_port_open_raw(READING_port,"/dev/ttyUSB0",usbInputSpeed);
	register_periodic_telemetry(DefaultPeriodic, "DISTANCE_MATRIX", send_distance_matrix);
}		

void printArray(uint8_t *toPrintArray, int totalLength, int width)
{
#if PRINT_STUFF
	for (int x = 0; x < totalLength; x++)
	{

	  printf(" ,%2d ",toPrintArray[x]);
	  if ((x+1)%width==0){
		  printf("line end\n");
	  }
	}
#endif
	for (int x = 0; x < totalLength; x++)
	{
		if(toPrintArray[x]>10){
			//printf(" ,%d ",toPrintArray[x]);
			 //printf("Danger!");
			 // Set to -5 degrees

		}



	}
}	

int isImageReady(int end, int start, int prevStart)
{
   if(start <0)
   {
	   return 0;
   }
   if(end <0)
   {
	   return 0;
   }
   if(start-end > 0)
   {
	   if(prevStart >=0)
	   {
		   return 1;
	   }
   }
   if(end-start>0)
   {
	   return 1;
   }

   return 0;
}


void serial_update(void) {
	int n=0;
	int timesTriedToRead=0;

	n = read(  READING_port->fd, &serialResponse[writeLocationInput], 10000 );
#if PRINT_STUFF
	printf("Read %d bytes\n",n);
#endif
	timesTriedToRead++;
	int lastRecordedStart=-1;
	int lastRecordedEnd=-1;
	int previousStart =-1;
	if (n > 0){
		// Check if we found the end of the image
		for(int startLocationToSearch=0; startLocationToSearch<writeLocationInput+n;startLocationToSearch++)
		{

				if(isStartOfImage(&serialResponse[startLocationToSearch]))
				{

					previousStart=lastRecordedStart;
					lastRecordedStart=startLocationToSearch;
				}
				if(isEndOfImage(&serialResponse[startLocationToSearch]))
				{
					lastRecordedEnd=startLocationToSearch;
				}
		}
#if PRINT_STUFF
		printf("Last recorded end: %d , last Recorded start: %d previous start %d !\n",lastRecordedEnd, lastRecordedStart, previousStart);
#endif
		if(isImageReady(lastRecordedEnd, lastRecordedStart, previousStart)>0)
		{
#if PRINT_STUFF
			printf("Searhing properties. LastRecorded start: %d lastRecordedEnd: %d previousStart: %d \n",lastRecordedStart,lastRecordedEnd, previousStart);
#endif
			// Find the properties of the image by iterating over the complete image
			ImageProperties imageProperties;
			if (lastRecordedEnd > lastRecordedStart){
				imageProperties= get_image_properties(serialResponse, lastRecordedEnd-lastRecordedStart, lastRecordedStart);
			}
			else
			{
				imageProperties = get_image_properties(serialResponse, lastRecordedEnd-previousStart, previousStart);
			}
#if PRINT_STUFF
			printf("Found image properties, start position: %d , width: %d, height: %d \n", imageProperties.positionImageStart, imageProperties.lineLength, imageProperties.height);
#endif
			if(imageProperties.positionImageStart<0 || imageProperties.height<0||imageProperties.lineLength<0){
				printf("[Read_matrix_serial] serious problem with the image properties");
				return;
			}
			if(imageProperties.height!=imageHeight || imageProperties.lineLength != imageWidth)
			{

				// Because image properties might change (when uploading new code to the multigaze), we need to resize arrays
				// and set the width and height variables
				printf("[Read_matrix_serial] image properties were not as expected");
				imageHeight=imageProperties.height;
				imageWidth=imageProperties.lineLength;
			}
			else
			{

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

				printArray(READimageBuffer,imageProperties.height*imageProperties.lineLength,imageProperties.lineLength);
			}

			// Now move everything after the end of the buffer to the start of the buffer
			int locationNewImage=0;
			for(int toProcess=lastRecordedEnd;toProcess<writeLocationInput+n;toProcess++)
			{
				serialResponse[locationNewImage++]=serialResponse[toProcess];
			}
			writeLocationInput=locationNewImage; // As we found a complete image we will now start writing at the start of the buffer again
		}
		else
		{
			writeLocationInput+=n;
		}
	}
}
void serial_start(void)
{
	//printf("serial start\n");
}
