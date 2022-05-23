#include "serialib.h"
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include <fstream>


#include <stdio.h>
#include <cstdLib>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#if defined (_WIN32) || defined(_WIN64)
//for serial ports above "COM9", we must use this extended syntax of "\\.\COMx".
//also works for COM0 to COM9.
//https://docs.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-createfilea?redirectedfrom=MSDN#communications-resources
#define SERIAL_PORT "\\\\.\\COM4"
#endif
#if defined (__linux__) || defined(__APPLE__)
#define SERIAL_PORT "/dev/ttyUSB0"
#endif

#include <sstream>
using namespace std;
#include <bitset>


using namespace std;

#include <stdint.h>


#include "ins.h"


uint16_t Crc16_CCITT(int wLength, char* pData)
{
	int i;
	uint16_t wCrc = 0xffff;

	while (wLength--) {
		wCrc ^= *(unsigned char*)pData++ << 8;
		for (i = 0; i < 8; i++)
			wCrc = wCrc & 0x8000 ? (wCrc << 1) ^ 0x1021 : wCrc << 1;
	}
	return wCrc & 0xffff;
}

// function definition
int convertb(signed val) {
	bitset<32> b(val);
	int result = 0;
	for (int k = 23; k < 31; k++) {
		result += ((int)b[k]) * pow(2, k - 23);
	}

	return result;
}

float mantissa(signed val) {
	float result = 0;
	bitset<32> b(val);
	for (int k = 0; k < 23; k++) {
		result += (int)b[22 - k] * pow(2, -1 * (k + 1));
	}
	result += 1;
	return result;
}

float converter(signed val) {
	bitset<32> b(val);

	float result;

	result = pow((-1), (int)b[31]) * pow(2, (convertb(val) - 127)) * mantissa(val);
	return result;
}


int insCall(float &rollOut, float& pitchOut, float& yawOut, float& altitudeOut, float& latGT, float& longGT, float& timeStamp)
{
	//open file for writing
	//ofstream fw("C:/Users/emrec/Desktop/CPlusPlusSampleFile.txt", std::ofstream::out);

	// Serial object

	serialib serial;


	signed val = 0x3d9d220d;

	// Connection to serial port
	char errorOpening = serial.openDevice(SERIAL_PORT, 115200);

	int crc_flag = 0;
	float result1, result2, result3;
	uint16_t resultos;

	// If connection fails, return the error code otherwise, display a success message
	if (errorOpening != 1) return errorOpening;
	//printf("Successful connection to %s\n", SERIAL_PORT);
	//cout << "errorOpening: " << (int)errorOpening << endl;
	// Set DTR
	serial.DTR(true);
	// Clear RTS
	serial.RTS(false);

	unsigned char* byte = new unsigned char[300];
	unsigned char* data = new unsigned char[82];

	char* temporary = new char[82];
	char* bizimcrc = new char[8];
	int combined;
	int i = 0;
	// Loop forever
	while (1)
	{
		serial.readBytes(byte, 300);
		int found = -1;
		for (int i = 0; i < 300; i++)
		{
			if (0x3d == (int)byte[i]) {
				found = i;
				break;
			}
		}
		if (i == -1) {
			serial.flushReceiver();
			continue;
		}
		// Original
		for (int i = found; i < found + 82; i++) {
			//cout << hex << (int) byte[i] << " ";
			temporary[i - found] = byte[i];
			data[i - found] = byte[i];
		}

		resultos = Crc16_CCITT(80, temporary);
		combined = data[81] << 8 | (data[80]);

		if (combined != resultos) {
			continue;
		}

		unsigned char* accX = new unsigned char[4];
		// This is for Accelerometer
		for (int i = found + 4; i < found + 8; i++) {
			accX[i - found - 4] = byte[i];
		}

		unsigned char* accY = new unsigned char[4];
		// This is for Accelerometer
		for (int i = found + 8; i < found + 12; i++) {
			accY[i - found - 8] = byte[i];
		}

		unsigned char* accZ = new unsigned char[4];
		// This is for Accelerometer
		for (int i = found + 12; i < found + 16; i++) {
			accZ[i - found - 12] = byte[i];
		}


		unsigned char* gyroX = new unsigned char[4];
		// This is for Gyro
		for (int i = found + 16; i < found + 20; i++) {
			gyroX[i - found - 16] = byte[i];
		}

		unsigned char* gyroY = new unsigned char[4];
		// This is for Gyro
		for (int i = found + 20; i < found + 24; i++) {
			gyroY[i - found - 20] = byte[i];
		}

		unsigned char* gyroZ = new unsigned char[4];
		// This is for Gyro
		for (int i = found + 24; i < found + 28; i++) {
			gyroZ[i - found - 24] = byte[i];
		}

		unsigned char* magX = new unsigned char[4];
		// This is for Magnetometer
		for (int i = found + 28; i < found + 32; i++) {
			magX[i - found - 28] = byte[i];
		}

		unsigned char* magY = new unsigned char[4];
		// This is for Magnetometer
		for (int i = found + 32; i < found + 36; i++) {
			magY[i - found - 32] = byte[i];
		}

		unsigned char* magZ = new unsigned char[4];
		// This is for Magnetometer
		for (int i = found + 36; i < found + 40; i++) {
			magZ[i - found - 36] = byte[i];
		}

		unsigned char* lat = new unsigned char[4];
		// This is for Time
		for (int i = found + 40; i < found + 44; i++) {
			lat[i - found - 40] = byte[i];
		}

		unsigned char* lon = new unsigned char[4];
		// This is for Time
		for (int i = found + 44; i < found + 48; i++) {
			lon[i - found - 44] = byte[i];
		}

		unsigned char* alt = new unsigned char[4];
		// This is for Time
		for (int i = found + 48; i < found + 52; i++) {
			alt[i - found - 48] = byte[i];
		}

		unsigned char* roll = new unsigned char[4];
		// This is for Euler
		for (int i = found + 52; i < found + 56; i++) {
			roll[i - found - 52] = byte[i];
		}

		unsigned char* pitch = new unsigned char[4];
		// This is for Euler
		for (int i = found + 56; i < found + 60; i++) {
			pitch[i - found - 56] = byte[i];
		}

		unsigned char* yall = new unsigned char[4];
		// This is for Euler
		for (int i = found + 60; i < found + 64; i++) {
			yall[i - found - 60] = byte[i];
		}

		unsigned char* barr_height = new unsigned char[4];
		// This is for Pressure
		for (int i = found + 72; i < found + 76; i++) {
			barr_height[i - found - 72] = byte[i];
		}

		unsigned char* time = new unsigned char[4];
		// This is for Time
		for (int i = found + 76; i < found + 80; i++) {
			time[i - found - 76] = byte[i];
		}

		

		if (crc_flag == 0)
			//continue;

			serial.flushReceiver();


		cout << endl;
		cout << endl;

		float lat_val, lon_val, alt_val;
		float time_val;

		time_val = converter(time[0] | (time[1] << 8) | (time[2] << 16) | (time[3] << 24));

		lat_val = (converter(lat[0] | (lat[1] << 8) | (lat[2] << 16) | (lat[3] << 24))) * 57.2957795;
		lon_val = (converter(lon[0] | (lon[1] << 8) | (lon[2] << 16) | (lon[3] << 24))) * 57.2957795;
		alt_val = (converter(alt[0] | (alt[1] << 8) | (alt[2] << 16) | (alt[3] << 24)));


		result1 = 57.2957795 * converter(roll[0] | (roll[1] << 8) | (roll[2] << 16) | (roll[3] << 24));
		result2 = 57.2957795 * converter(pitch[0] | (pitch[1] << 8) | (pitch[2] << 16) | (pitch[3] << 24));
		result3 = 57.2957795 * converter(yall[0] | (yall[1] << 8) | (yall[2] << 16) | (yall[3] << 24));

		float height = converter(barr_height[0] | (barr_height[1] << 8) | (barr_height[2] << 16) | (barr_height[3] << 24));

		/*

		cout << "TIME: " << time_val << endl;

		cout << "ROLL	" << result1 << endl;
		cout << "PITCH	" << result2 << endl;
		cout << "YAW	" << result3 << endl;
		cout << endl;
		cout << endl;
		cout << "height	" << converter(barr_height[0] | (barr_height[1] << 8) | (barr_height[2] << 16) | (barr_height[3] << 24)) << endl;
		cout << endl;
		cout << endl;
		cout << endl;
		cout << "llh1	" << lat_val << endl;
		cout << "llh2	" << lon_val << endl;
		cout << "llh3	" << alt_val << endl;
		cout << "***************************************************************************************************" << endl;

		*/

		//fw << "Time: " << time_val << " Lat: " << lat_val << " Lon: " << lon_val << " Altitude: " << alt_val << "\n";

		if(lat_val > 1 || lon_val > 1)
		{
			delete[] accX;
			delete[] accY;
			delete[] accZ;
			delete[] magX;
			delete[] magY;
			delete[] magZ;
			delete[] lat;
			delete[] lon;
			delete[] alt;
			delete[] roll;
			delete[] pitch;
			delete[] yall;
			delete[] barr_height;
			delete[] time;
			delete[] gyroX;
			delete[] gyroY;
			delete[] gyroZ;
			
			continue;
		}
		delete[] accX;
		delete[] accY;
		delete[] accZ;
		delete[] magX;
		delete[] magY;
		delete[] magZ;
		delete[] lat;
		delete[] lon;
		delete[] alt;
		delete[] roll;
		delete[] pitch;
		delete[] yall;
		delete[] barr_height;
		delete[] time;
		delete[] gyroX;
		delete[] gyroY;
		delete[] gyroZ;
		
		rollOut = result1;
		pitchOut = result2;
		yawOut = result3;
		altitudeOut = height;
		latGT = lat_val;
		longGT = lon_val;
		timeStamp = time_val;
		break;
	}

	// Close the serial device
	delete[] byte;
	delete[] data;
	delete[] temporary;
	delete[] bizimcrc;
	return 1;
}