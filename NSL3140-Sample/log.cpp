#include "main.h"

#include <iomanip>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <mutex>


#define LOG_FOLDER "LOGS"

void saveDataPCD(NslPCD *ptNslPCD, const std::string& filePath)
{
	if( !ptNslPCD->includeLidar ) return;

	std::ofstream stream(filePath + ".pcd");
	if (!stream.is_open()) return;

	stream << "VERSION 0.7\n";
	stream << "FIELDS x y z intensity\n";
	stream << "SIZE 4 4 4 4\n";
	stream << "TYPE F F F F\n";
	stream << "COUNT 1 1 1 1\n";
	stream << "VIEWPOINT 0 0 0 1 0 0 0\n";
	if(ptNslPCD->lidarType != LIDAR_TYPE_OPTIONS::TYPE_B )
	{
		stream << "WIDTH 320\n";
		stream << "HEIGHT 240\n";
		stream << "POINTS 76800\n";
	}
	else{
		stream << "WIDTH 800\n";
		stream << "HEIGHT 600\n";
		stream << "POINTS 480000\n";
	}

	stream << "DATA ascii\n";
	stream << std::fixed << std::setprecision(2);

	bool isAmplitude = false;

	if (ptNslPCD->operationMode == NslOption::OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE 
		|| ptNslPCD->operationMode == NslOption::OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE ) 
	{
		isAmplitude = true;
	}


	int lidarWidth = ptNslPCD->lidarType != LIDAR_TYPE_OPTIONS::TYPE_B ? NSL_LIDAR_TYPE_A_WIDTH : NSL_LIDAR_TYPE_B_WIDTH;
	int lidarHeight = ptNslPCD->lidarType != LIDAR_TYPE_OPTIONS::TYPE_B ? NSL_LIDAR_TYPE_A_HEIGHT : NSL_LIDAR_TYPE_B_HEIGHT;

	for (int y = 0; y < lidarHeight; ++y) {
		for (int x = 0; x < lidarWidth; ++x) {
			if( ptNslPCD->distance3D[OUT_Z][y][x] > 0 && ptNslPCD->distance3D[OUT_Z][y][x] < NSL_LIMIT_FOR_VALID_DATA )
			{
				if ( isAmplitude ) {
					stream << ptNslPCD->distance3D[OUT_X][y][x] << " " << ptNslPCD->distance3D[OUT_Y][y][x] << " " << ptNslPCD->distance3D[OUT_Z][y][x] << " " << ptNslPCD->amplitude[y][x] << "\n";
				} else {
					stream << ptNslPCD->distance3D[OUT_X][y][x] << " " << ptNslPCD->distance3D[OUT_Y][y][x] << " " << ptNslPCD->distance3D[OUT_Z][y][x] << " " << ptNslPCD->distance3D[OUT_Z][y][x] << "\n";
				}
			}
			else{
				stream <<"nan nan nan nan\n";
			}
		}
	}

	stream.close();
}

void saveRGB(NslPCD *ptNslPCD, const std::string& filePath)
{
	if( !ptNslPCD->includeRgb ) return;

	cv::Mat imageRgb(NSL_RGB_IMAGE_HEIGHT, NSL_RGB_IMAGE_WIDTH, CV_8UC3);  // BGR
	int totalPixels = NSL_RGB_IMAGE_HEIGHT * NSL_RGB_IMAGE_WIDTH;
	cv::Vec3b* dstPtr = imageRgb.ptr<cv::Vec3b>();
	NslOption::NslVec3b* srcPtr = gtViewerInfo.rgb.data();

	for (int i = 0; i < totalPixels; ++i) {
	    dstPtr[i] = cv::Vec3b(
	        srcPtr[i].b,  // blue
	        srcPtr[i].g,  // green
	        srcPtr[i].r   // red
	    );
	}

    // Save image as JPG
    cv::imwrite(filePath + ".jpg", imageRgb);
}

void saveIndex(const std::string& filePath, const std::string& str)
{
	std::ofstream file(filePath, std::ios::app);  // append mode
	if (file.is_open()) {
		file << str << '\n';
		file.close();
	}
}

void createLogDirectory() {
#ifdef _WINDOWS
    CreateDirectoryA(LOG_FOLDER, nullptr);
#else
    mkdir(LOG_FOLDER, 0755);
#endif
}

void logData(NslPCD *ptNslPCD)
{
	using namespace std::chrono;
	auto now = system_clock::now();
	std::time_t t_c = system_clock::to_time_t(now);
	std::tm localTime = *std::localtime(&t_c);
	static int count = -1;
	static int second = -1;
	static std::string indexFilePath = "";

	int sec = localTime.tm_sec;

	if (sec != second) {
		count = 0;
		second = sec;
	} else {
		count++;
	}

	std::ostringstream ossSuffix;
	ossSuffix << "-" << std::setw(3) << std::setfill('0') << count;

	std::ostringstream ossTimestamp;
	ossTimestamp << std::put_time(&localTime, "%Y%m%d-%H%M%S");

	std::string filenameIdx = "/image_" + ossTimestamp.str();
	std::string filename = filenameIdx + ossSuffix.str();

	if ( indexFilePath.length() == 0 ) {
		indexFilePath = LOG_FOLDER + filenameIdx + ".idx";
	}

	saveDataPCD(ptNslPCD, LOG_FOLDER + filename);
	saveRGB(ptNslPCD, LOG_FOLDER + filename);
	saveIndex(indexFilePath, filename);
}


