#include <linux/i2c-dev.h>
//#include <i2c/smbus.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h> 

#include <iostream>
#include <string>


#include <opencv2/opencv.hpp>

// largely based on  arduino code at https://wiki.dfrobot.com/Positioning_ir_camera__SKU_SEN0158



bool write2bytes(int file, char d1, char d2)
{
	char buffer[2];
	buffer[0] = d1;
	buffer[1] = d2;
	if (write(file, buffer, 2) != 2) {
		return false;
	}
	return true;
}

int main(int argc, char* argv[] ) {

	// Setup I2C device
	std::string deviceName = "/dev/i2c-1";
	auto file = open(deviceName.c_str(), O_RDWR);
	if (file < 0) {
		std::cerr << "Failed to open device: " << deviceName << std::endl;
		exit(1);
	}

	int addr = 0x58; // i2c slave address
	if (ioctl(file, I2C_SLAVE, addr) < 0) {
		std::cerr << "Failed to access slave: " << std::hex << addr << std::endl;
		exit(1);
	}


	// IR sensor initialize (sequence from DFRobot arduino example)
	struct timespec delay;
	struct timespec remains;
	delay.tv_sec = 0;
	delay.tv_nsec = 10000000; // 10 miliseconds
	write2bytes(file,0x30,0x01); nanosleep(&delay, &remains);
	write2bytes(file,0x30,0x08); nanosleep(&delay, &remains);
	write2bytes(file,0x06,0x90); nanosleep(&delay, &remains);
	write2bytes(file,0x08,0xC0); nanosleep(&delay, &remains);
	write2bytes(file,0x1A,0x40); nanosleep(&delay, &remains);
	write2bytes(file,0x33,0x33); nanosleep(&delay, &remains);
	delay.tv_nsec = 5000000; // 5 miliseconds
	nanosleep(&delay, &remains);


	// Read continuously
	char buffer[255] = {0};
	int Ix[4];
	int Iy[4];
	int s;
	cv::namedWindow("IR", cv::WINDOW_AUTOSIZE);
	cv::Mat blankImg( 768, 1024, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat displayImg( 768, 1024, CV_8UC3, cv::Scalar(0,0,0));
	bool persist = false;
	while(true) {
		// send IR sensor read command
		buffer[0] = 0x36;
		size_t len = 1;
		if (write(file, buffer, len) != len)		
		{
			std::cerr << "Failed to write to the i2c bus.\n";
		}	

		// read 16 bytes over i2C
		len = 16;
		if (read(file, buffer, len) != len)	
		{
			std::cerr << "Failed to read from the i2c bus.\n";
		}

		Ix[0] = buffer[1];
		Iy[0] = buffer[2];
		s   = buffer[3];
		Ix[0] += (s & 0x30) <<4;
		Iy[0] += (s & 0xC0) <<2;

		Ix[1] = buffer[4];
		Iy[1] = buffer[5];
		s   = buffer[6];
		Ix[1] += (s & 0x30) <<4;
		Iy[1] += (s & 0xC0) <<2;

		Ix[2] = buffer[7];
		Iy[2] = buffer[8];
		s   = buffer[9];
		Ix[2] += (s & 0x30) <<4;
		Iy[2] += (s & 0xC0) <<2;

		Ix[3] = buffer[10];
		Iy[3] = buffer[11];
		s   = buffer[12];
		Ix[3] += (s & 0x30) <<4;
		Iy[3] += (s & 0xC0) <<2;

		if( !persist ) {
			displayImg = blankImg.clone();
		}
		std::cout << "----" << std::endl;
		for(int i=0; i<4; i++)
		{
			std::cout << i << ": " <<  int(Ix[i])  << "," << int(Iy[i]) << std::endl;
			cv::Point2f pt( Ix[i], Iy[i] );
			cv::Scalar color( 0,0,255);
			if( i == 1 ) {
			 	color = cv::Scalar( 0,255, 0);
			} else if (i == 2 ) {
			 	color = cv::Scalar(255,0, 0);
			} else if( i == 3 ) {
			 	color = cv::Scalar(255,255, 0);
			}
			cv::circle( displayImg, pt, 1.0, color, 2.0f );
		}
		// slight delay
		cv::imshow("IR", displayImg);
		auto key = cv::waitKey(5);
		if( key == 'q' ) {
			break;
		} else if ( key == 'p' ) {
			persist = !persist;
		}

	}	
	cv::destroyWindow("IR");
	return 0;
}
