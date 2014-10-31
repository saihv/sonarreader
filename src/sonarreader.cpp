#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

using namespace std;

void writeBytes(int descriptor, int count);
void readBytes(int descriptor, int count);

void setModeIIC(int fd, char mode); 
void setModeIICatod(int fd, char mode); 
void setModeIICultraSound(int fd, char mode);
int  aatoint(char hi,char lo);

char serialBuffer[100];					// Serial buffer sto store data for I/O

ros::Publisher vis_pub;

void showVPArrow(string name, geometry_msgs::Point pt1, geometry_msgs::Point pt2, int idx, float r = 1, float g = 0, float b = 1, float duration = 0.1)
{
	visualization_msgs::Marker marker;
	
	geometry_msgs::Point begin;
	begin.x = pt1.x;
	begin.y = pt1.y;
	begin.z = pt1.z;
	
	geometry_msgs::Point end;
	end.x = pt2.x;
	end.y = pt2.y;
	end.z = pt2.z;
	
	marker.points.push_back(begin);
	marker.points.push_back(end);
		    
    //marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = name;
    marker.id = idx;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
	
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.a = 0.5;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.lifetime = ros::Duration(duration); 
    vis_pub.publish( marker );
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "readSonar");
	int fd;						// File descriptor of port we will talk to 
	
	char *portName = "/dev/ttyUSB0";		// Name of the IIC module
	struct termios options;				// Port options
	
    printf(" sizeof: %d aatoInt: %d\n", sizeof( fd ), aatoint( '1','2' ) );

	fd = open(portName, O_RDWR | O_NOCTTY);		// Open port for read and write not making it a controlling terminal
	if (fd == -1)
	{
   		perror("openPort: Unable to open port ");		// If open() returns an error
	} else {
	  printf("\n usb iic usb open \n");
	} 

	tcgetattr(fd, &options);
	cfsetispeed(&options, B19200);						// Set baud rate
	cfsetospeed(&options, B19200);					
	cfmakeraw(&options);
	tcflush(fd, TCIFLUSH);

	options.c_cc[VMIN]  = 6; // wait for atleast these CHARs
	options.c_cc[VTIME] = 3; // wait deci seconds.

	tcsetattr(fd, TCSANOW, &options);
	usleep(1000);							// Sleep for UART to power up and set options
	tcflush(fd, TCIFLUSH);
	cfsetospeed(&options, B19200);					
	cfmakeraw(&options);
	tcflush(fd, TCIFLUSH);
	
	usleep(1000);							// Sleep for UART to power up and set options
	
	printf("\n ==================== \n");
	
	setModeIICultraSound( fd, 1 );
	usleep(100000);						// Sleep for UART to power up and set options

	setModeIICultraSound( fd, 1 );

	while(1){
	  setModeIICultraSound( fd, 1 );
	}
	usleep(100000);						// Sleep for UART to power up and set options
	
	close(fd);							// Close port
	
	return 0;
}

void writeBytes(int descriptor, int count) {
   int cnt;
	if (( cnt = write(descriptor, serialBuffer, count)) == -1) {	// Send data out	
		perror("Error writing");
		close(descriptor);					// Close port if there is an error
		exit(1);
	}
}

void readBytes(int descriptor, int count) {

	int count_rd;
	if (count_rd = read(descriptor, serialBuffer, count) == -1) {		// Read back data into buf[]
		perror("Error reading ");
		close(descriptor);					// Close port if there is an error
		exit(1);
	}
	
}

void setModeIIC(int fd, char mode) {
	int count,cp3;
	
	serialBuffer[0] = 0x5A;		
	serialBuffer[1] = 0x10;						// Mode we wish to set
	serialBuffer[2] = mode ;					// Mode we wish to set
	serialBuffer[3] = 0x00;						// Mode we wish to set
	writeBytes(fd, 4);

	readBytes( fd,1);
	printf("%02X \n", serialBuffer[0] );
        
}

void setModeIICatod(int fd, char mode) {

	setModeIIC( fd, 0x0C );

	
	usleep(100000);						// Sleep for UART to power up and set options
	serialBuffer[0] = 0x5A;		
	serialBuffer[1] = 0x12;						// Mode we wish to set
	serialBuffer[2] = 0x00;						// Mode we wish to set
	serialBuffer[3] = 0x00;						// Mode we wish to set

	writeBytes(fd, 4);
        usleep(  600000  );

	readBytes( fd,8);
	printf(  "%02X %02X   %02X %02X \n", serialBuffer[0] , serialBuffer[1],        serialBuffer[2], serialBuffer[3] );
}

void setModeIICultraSound(int fd, char mode) 
{	
	int aai_left, aai_right, aaat_left, aaat_right ;	
	geometry_msgs::Point pOrigin, pTarget;
	pOrigin.x = pOrigin.y = pOrigin.z = 0;

	
	serialBuffer[0] = 0x55;	// iic command to do iic 	
	serialBuffer[1] = 0xE0;	// iic address
	serialBuffer[2] = 0x00;	// device internal reg
	serialBuffer[3] = 0x01;	// number of bytes to write
	serialBuffer[4] = 0x51;	// 0x52	 Real Ranging Mode - Result in micro-seconds

	writeBytes(fd, 5);
	readBytes( fd,1);
	usleep(  100000  );

	// Set up the iic for a read operation.
	serialBuffer[0] = 0x55;	// iic command to do iic 	
	serialBuffer[1] = 0xE1;	// iic address
	serialBuffer[2] = 0x00;	// device internal reg
	serialBuffer[3] = 0x06;	// number of bytes to read
	writeBytes(fd, 4);
	readBytes( fd,6);

 	aai_left  = aatoint( serialBuffer[3], serialBuffer[2]);
    aaat_left = aatoint( serialBuffer[5], serialBuffer[4]);
	


	// Done with the first sonar, moving on to the second - Sai
	usleep(70000); 	// This needs to be at least 65 ms to prevent echoing


	serialBuffer[0] = 0x55;	// iic command to do iic 	
	serialBuffer[1] = 0xF2;	// iic address
	serialBuffer[2] = 0x00;	// device internal reg
	serialBuffer[3] = 0x01;	// number of bytes to write
	serialBuffer[4] = 0x51;	// 0x52	 Real Ranging Mode - Result in micro-seconds

	writeBytes(fd, 5);
	readBytes( fd,1);
	usleep(  100000  );

	// Set up the iic for a read operation.
	serialBuffer[0] = 0x55;	// iic command to do iic 	
	serialBuffer[1] = 0xF3;	// iic address
	serialBuffer[2] = 0x00;	// device internal reg
	serialBuffer[3] = 0x06;	// number of bytes to read
	writeBytes(fd, 4);
	readBytes( fd,6);

 	// au.b1[0] = serialBuffer[3];
 	// au.b1[1] = serialBuffer[2];

    aai_right  = aatoint( serialBuffer[3], serialBuffer[2]);
    aaat_right = aatoint( serialBuffer[5], serialBuffer[4]);

    if(aai_left < 100 && aai_right < 100)
    {
    	printf("Too close to an obstacle on both LEFT and RIGHT <<------------>>>\n");
		pTarget.x = 0; //turn right
		pTarget.y = 0;
		pTarget.z = -1000;
		showVPArrow("sonar_signal", pOrigin, pTarget, 103);
    }

    else if(aai_left < 100)
    {
    	printf("Too close to an obstacle on the LEFT <<<---------------\n");
		pTarget.x = 1000; //turn right
		pTarget.y = 0;
		pTarget.z = 0;
		showVPArrow("sonar_signal", pOrigin, pTarget, 101);
    }

    else if(aai_right < 100)
    {
    	printf("Too close to an obstacle on the RIGHT --------------->>>\n");
		pTarget.x = -1000; //turn left
		pTarget.y = 0;
		pTarget.z = 0;
		showVPArrow("sonar_signal", pOrigin, pTarget, 102);
    }

    else
    {
    	printf("Clear pathway!\n");
    	pTarget.x = 0; //clear
		pTarget.y = 0;
		pTarget.z = 0;
		showVPArrow("sonar_signal", pOrigin, pTarget, 100);				
    }
}


int aatoint(char hi,char lo){

  union ua{
    int i;
    char a[2];  // lo-hi
  } u;
  
  if ( sizeof( u.i ) == 4 ) {
    u.a[0]=hi;
    u.a[1]=lo;
    u.a[2]=0;
    u.a[3]=0; 
  } else {
    printf( "sizeof( int ) != 4 - needs little enden numbers\n" );
    exit(1);
  }
  return u.i; 
}

/* END */
