
/*
*
* Copyright (C) 2025 Nanosystems
* All rights reserved.
*
* SPDX-License-Identifier: MIT
*
* main.cpp
*
*/

//#define __USED_PCL_LIBLARY__

#ifdef _WINDOWS
#include <windows.h>
#include <setupapi.h>
#include <devguid.h>
#include <regstr.h>

#pragma comment(lib, "setupapi.lib")

#else
#include <sys/stat.h>
#endif

#include <iomanip>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <mutex>

#ifdef __USED_PCL_LIBLARY__
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <vtkRendererCollection.h>
#include <vtkRenderWindow.h>
#include <vtkCamera.h>
#endif

#include <opencv2/opencv.hpp>

#include "nanolib.h"

#define LOG_FOLDER "LOGS"

using namespace cv;
using namespace std;
using namespace NslOption;

#define DISTANCE_INFO_HEIGHT	80
#define VIEWER_SCALE_SIZE		2
#define DRAW_POINT_CLOUD	0x01
#define DRAW_OPENCV_VIEW	0x02

typedef struct ViewerInfo_
{
	// for management
	int 	drawView;
	int 	mainRunning;
	int		mouseX;
	int		mouseY;
	int 	frameCount;
	int 	drawframeCount;
	char 	ipAddress[20];
	double	temperature;
	int 	handle;
	int 	width;
	int		height;
	int 	xMin;
	int 	xMax;
	int		yMin;
	int 	yMax;
	bool	startLog;
	OPERATION_MODE_OPTIONS 	operationMode;

	// for Lidar
	NslConfig			nslConfig;

	// for detection Area box
	bool  area_enable;
	float area_left;
	float area_right;
	float area_top;
	float area_bottom;
	float area_start;
	float area_end;
	int area_inCount;

	// for PCL
#ifdef __USED_PCL_LIBLARY__
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;	  
	vector<pcl::visualization::PCLVisualizer::Ptr> viewers;
#endif
	ViewerInfo_()
	{
		startLog = false; // true:save PCD & RGB log, false:default
		drawView = DRAW_POINT_CLOUD | DRAW_OPENCV_VIEW;
		mainRunning = 1;
		mouseX = -1;
		mouseY = -1;

		area_enable = true; // true : area display, false : all display
		area_left = -800;// -800.0f; // mm
		area_right = 1500;// 1500.0f; // mm
		area_top = -500.0f; // mm
		area_bottom = 500.0f; // mm
		area_start = 0.0f; // mm
		area_end = 3000.0f; // mm
		area_inCount = 0;

#ifdef __USED_PCL_LIBLARY__
		cout << "PCL Version: " << PCL_VERSION_PRETTY << endl;
#endif
		frameCount = 0;
		drawframeCount = 0;
		temperature = 0;
		handle = -1;

		memset(&nslConfig, 0, sizeof(NslConfig));
		nslConfig.lidarAngle = 0;
		nslConfig.lensType = NslOption::LENS_TYPE::LENS_SF;
		
		//sprintf(ipAddress,"/dev/ttyNsl3140");
		//sprintf(ipAddress,"\\\\.\\COM12");
		sprintf(ipAddress,"192.168.0.220");
	}

}VIEWER_INFO, *LP_VIEWER_INFO;


VIEWER_INFO	gtViewerInfo;
std::unique_ptr<NslPCD> latestFrame = std::make_unique<NslPCD>();;

/////////////////////////////////// draw function /////////////////////////////////////////////////////
#ifdef _WINDOWS
int findPortsByVidPid(const std::string vid, const std::string pid, char *strPortName)
{
	bool findPort = false;
    HDEVINFO hDevInfo = SetupDiGetClassDevs(&GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);
    if (hDevInfo == INVALID_HANDLE_VALUE) {
        std::cerr << "SetupDiGetClassDevs failed." << std::endl;
        return -2;
    }

    SP_DEVINFO_DATA devInfoData;
    devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

    for (DWORD i = 0; !findPort && SetupDiEnumDeviceInfo(hDevInfo, i, &devInfoData); ++i) {
        char hardwareId[256] = {0};
        if (SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devInfoData, SPDRP_HARDWAREID, nullptr,
                                              (PBYTE)hardwareId, sizeof(hardwareId), nullptr)) {
            std::string hwid = hardwareId;

            if (hwid.find("VID_" + vid) != std::string::npos &&
                hwid.find("PID_" + pid) != std::string::npos) {

                // Open registry key for device
                HKEY hDeviceRegistryKey = SetupDiOpenDevRegKey(
                    hDevInfo, &devInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);

                if (hDeviceRegistryKey != INVALID_HANDLE_VALUE) {
					char portName[256];
                    DWORD size = 256;
                    DWORD type = 0;

                    if (RegQueryValueExA(hDeviceRegistryKey, "PortName", nullptr, &type, (LPBYTE)portName, &size) == ERROR_SUCCESS) 
                    {
                    	sprintf(strPortName,"\\\\.\\%s", portName);
						findPort = true;		
                    }

                    RegCloseKey(hDeviceRegistryKey);
                }
            }
        }
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
    return findPort ? 0 : -1;
}
#endif

#ifdef __USED_PCL_LIBLARY__


void onKeyboardEvent(const pcl::visualization::KeyboardEvent& event, void* viewer_void) {
    if (event.getKeySym() == "Escape" && event.keyDown()) {
		gtViewerInfo.mainRunning = 0;
    }
    else if (event.getKeySym() == "d" && event.keyDown()) {
		gtViewerInfo.drawView ^= DRAW_POINT_CLOUD;
		if( gtViewerInfo.drawView & DRAW_POINT_CLOUD ) printf("POINT CLOUD DrawView On\n");
		else printf("POINT CLOUD DrawView Off\n");
    }
}

void drawPointCloud()
{	
	pcl::visualization::PCLVisualizer::Ptr viewer = gtViewerInfo.viewers[0];
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr = gtViewerInfo.clouds[0];

	if( gtViewerInfo.area_enable ){
		viewer->updateText("Detection : " + std::to_string(gtViewerInfo.area_inCount), 1070, 50, "point-cnt");
	}

	if( !viewer->wasStopped() && point_cloud_ptr->points.size() > 0 ){
		viewer->updatePointCloud(point_cloud_ptr, "Viewer pointCloud");
		viewer->spinOnce(30);
	}
}

#endif


void mouseCallbackCV(int event, int x, int y, int flags, void* user_data)
{
    if (event == EVENT_LBUTTONUP)
    {
        gtViewerInfo.mouseX = x;
		gtViewerInfo.mouseY = y;
    }
}

void printConfiguration()
{
	printf("------------------------------------------------------------------------\n");
	printf("------------------------- Device configuration -------------------------\n");
	printf("------------------------------------------------------------------------\n");
	printf("firmware version = %d.%d\n", gtViewerInfo.nslConfig.firmware_release>>16&0xFFFF, gtViewerInfo.nslConfig.firmware_release&0xFFFF);
	printf("waferID = %d\n", gtViewerInfo.nslConfig.waferID);
	printf("chipID = %d\n", gtViewerInfo.nslConfig.chipID);
	printf("UDP RX port = %d\n", gtViewerInfo.nslConfig.udpDataPort);
	printf("HDR = %s\n", toString(gtViewerInfo.nslConfig.hdrOpt));
	printf("int time = %d.%d.%d.%d\n", gtViewerInfo.nslConfig.integrationTime3D
									, gtViewerInfo.nslConfig.integrationTime3DHdr1
									, gtViewerInfo.nslConfig.integrationTime3DHdr2
									, gtViewerInfo.nslConfig.integrationTimeGrayScale);
	printf("roi = %d,%d,%d,%d\n", gtViewerInfo.nslConfig.roiXMin
								, gtViewerInfo.nslConfig.roiYMin
								, gtViewerInfo.nslConfig.roiXMax
								, gtViewerInfo.nslConfig.roiYMax );
	printf("Modulation = %s, ch = %s, autoChannel = %s\n", toString(gtViewerInfo.nslConfig.mod_frequencyOpt), toString(gtViewerInfo.nslConfig.mod_channelOpt), toString(gtViewerInfo.nslConfig.mod_enabledAutoChannelOpt));
	printf("dual beam = %s, option = %s\n", toString(gtViewerInfo.nslConfig.dbModOpt), toString(gtViewerInfo.nslConfig.dbOpsOpt));
	printf("Binning vertical = %s, horizontal = %s\n", toString(gtViewerInfo.nslConfig.ver_binningOpt), toString(gtViewerInfo.nslConfig.horiz_binningOpt));
	printf("adc overflow = %s, saturation = %s\n", toString(gtViewerInfo.nslConfig.overflowOpt), toString(gtViewerInfo.nslConfig.saturationOpt));
	printf("Compensation drnu = %s, temperature = %s, grayscale = %s, ambient = %s\n", toString(gtViewerInfo.nslConfig.drnuOpt), toString(gtViewerInfo.nslConfig.temperatureOpt), toString(gtViewerInfo.nslConfig.grayscaleOpt), toString(gtViewerInfo.nslConfig.ambientlightOpt));
	printf("filter median = %s, gauss = %s, temporal factor = %d, temporal threshold = %d, edge threshold = %d, interferenceLimit = %d, used interference Last value = %s\n"
		, toString(gtViewerInfo.nslConfig.medianOpt)
		, toString(gtViewerInfo.nslConfig.gaussOpt)
		, gtViewerInfo.nslConfig.temporalFactorValue
		, gtViewerInfo.nslConfig.temporalThresholdValue
		, gtViewerInfo.nslConfig.edgeThresholdValue
		, gtViewerInfo.nslConfig.interferenceDetectionLimitValue
		, toString(gtViewerInfo.nslConfig.interferenceDetectionLastValueOpt));

	printf("UDP speed = %s\n", toString(gtViewerInfo.nslConfig.udpSpeedOpt));
	printf("frame rate = %s\n", toString(gtViewerInfo.nslConfig.frameRateOpt));

	printf("------------------------------------------------------------------------\n");
}

char *getDataTypeName(OPERATION_MODE_OPTIONS type)
{
	switch(type)
	{
		case OPERATION_MODE_OPTIONS::NONE_MODE:
			return (char *)"NONE";
		case OPERATION_MODE_OPTIONS::DISTANCE_MODE:
			return (char *)"D";
		case OPERATION_MODE_OPTIONS::GRAYSCALE_MODE:
			return (char *)"G";
		case OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE:
			return (char *)"DA";
		case OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE:
			return (char *)"DG";
		case OPERATION_MODE_OPTIONS::RGB_MODE:
			return (char *)"R";
		case OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE:
			return (char *)"RD";
		case OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE:
			return (char *)"RDA";
		case OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE:
			return (char *)"RDG";
	}

	return (char *)"NONE";
}

void timeDelay(int milli)
{
	auto start = std::chrono::steady_clock::now();
	while (gtViewerInfo.mainRunning != 0) {
		auto now = std::chrono::steady_clock::now();
		if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() >= milli) {
			break;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

}

void timeCheckThread(int void_data)
{
	while( gtViewerInfo.mainRunning != 0 ){
		timeDelay(1000);
		int count = gtViewerInfo.frameCount;
		gtViewerInfo.drawframeCount = count;
		gtViewerInfo.frameCount = 0;

		printf("### [%s][%d<%d:%d> x %d<%d:%d>] :: frame count = %d, %.2f'C ###\n", toString(gtViewerInfo.operationMode), gtViewerInfo.width, gtViewerInfo.xMin, gtViewerInfo.xMax, gtViewerInfo.height, gtViewerInfo.yMin, gtViewerInfo.yMax, count, gtViewerInfo.temperature);
	}
}


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
    cv::Mat imageRgb(NSL_RGB_IMAGE_HEIGHT, NSL_RGB_IMAGE_WIDTH, CV_8UC3);  // BGR
	int totalPixels = NSL_RGB_IMAGE_HEIGHT * NSL_RGB_IMAGE_WIDTH;
	cv::Vec3b* dstPtr = imageRgb.ptr<cv::Vec3b>();
	NslOption::NslVec3b* srcPtr = &ptNslPCD->rgb[0][0];

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

void createDirectory(const std::string path) {
#ifdef _WINDOWS
    CreateDirectoryA(path.c_str(), nullptr);
#else
    mkdir(path.c_str(), 0755);
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



/////////////////////////////////// cv display function /////////////////////////////////////////////////////


/**
 * @brief openCV draw function
 * 
 * @param distMat : Mat data to be drawn on the screen
 * @param handleIndex : handle index by nsl_open()
 * 
 * @return cv::Mat 
 */
Mat addDistanceInfo(Mat distMat, NslPCD *ptNslPCD, int lidarWidth, int lidarHeight, int scaleSize)
{
	int width = ptNslPCD->width;
	int height = ptNslPCD->height;
	int viewer_xpos = gtViewerInfo.mouseX;
	int viewer_ypos = gtViewerInfo.mouseY;
	float textSize = 0.8f;
	int xMin = ptNslPCD->roiXMin;
	int yMin = ptNslPCD->roiYMin;
	int xpos = viewer_xpos/scaleSize;
	int ypos = viewer_ypos/scaleSize;
	
	if( (ypos >= yMin && ypos < lidarHeight)){

		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		line(distMat, Point(viewer_xpos-13, viewer_ypos), Point(viewer_xpos+13, viewer_ypos), Scalar(255, 255, 0), 2);
		line(distMat, Point(viewer_xpos, viewer_ypos-15), Point(viewer_xpos, viewer_ypos+15), Scalar(255, 255, 0), 2);

		if( xpos >= lidarWidth ){ 
			xpos -= lidarWidth;
		}

		string dist2D_caption;
		string dist3D_caption;
		string info_caption;

		int distance2D = ptNslPCD->distance2D[ypos][xpos];
		double distance3D = ptNslPCD->distance3D[OUT_Z][ypos][xpos];
		if( distance3D > NSL_LIMIT_FOR_VALID_DATA ){

			if( distance3D == NSL_ADC_OVERFLOW )
				dist2D_caption = format("X:%d,Y:%d ADC_OVERFLOW", xpos, ypos);
			else if( distance3D == NSL_SATURATION )
				dist2D_caption = format("X:%d,Y:%d SATURATION", xpos, ypos);
			else if( distance3D == NSL_BAD_PIXEL )
				dist2D_caption = format("X:%d,Y:%d BAD_PIXEL", xpos, ypos);
			else if( distance3D == NSL_INTERFERENCE )
				dist2D_caption = format("X:%d,Y:%d INTERFERENCE", xpos, ypos);
			else if( distance3D == NSL_EDGE_DETECTED )
				dist2D_caption = format("X:%d,Y:%d EDGE_FILTERED", xpos, ypos);
			else
				dist2D_caption = format("X:%d,Y:%d LOW_AMPLITUDE", xpos, ypos);
		}
		else{
			if( ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE || ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE ) {
				dist2D_caption = format("2D X:%d Y:%d %dmm/%dlsb", xpos, ypos, ptNslPCD->distance2D[ypos][xpos], ptNslPCD->amplitude[ypos][xpos]);
				dist3D_caption = format("3D X:%.1fmm Y:%.1fmm Z:%.1fmm", ptNslPCD->distance3D[OUT_X][ypos][xpos], ptNslPCD->distance3D[OUT_Y][ypos][xpos], ptNslPCD->distance3D[OUT_Z][ypos][xpos]);
			}
			else{
				dist2D_caption = format("2D X:%d Y:%d <%d>mm", xpos, ypos, ptNslPCD->distance2D[ypos][xpos]);
				dist3D_caption = format("3D X:%.1fmm Y:%.1fmm Z:%.1fmm", ptNslPCD->distance3D[OUT_X][ypos][xpos], ptNslPCD->distance3D[OUT_Y][ypos][xpos], ptNslPCD->distance3D[OUT_Z][ypos][xpos]);
			}
		}

		info_caption = format("%s:%dx%d <%dfps> %.2f'C", getDataTypeName(ptNslPCD->operationMode), width, height, gtViewerInfo.drawframeCount, gtViewerInfo.temperature);

		putText(infoImage, info_caption.c_str(), Point(10, 23), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, dist2D_caption.c_str(), Point(10, 46), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, dist3D_caption.c_str(), Point(10, 70), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);
		vconcat(distMat, infoImage, distMat);
	}
	else{
		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		string info_caption = format("%s:%dx%d <%dfps> %.2f'C", getDataTypeName(ptNslPCD->operationMode), width, height, gtViewerInfo.drawframeCount, gtViewerInfo.temperature);
		putText(infoImage, info_caption.c_str(), Point(10, 23), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);		
		vconcat(distMat, infoImage, distMat);
	}

	return distMat;
}

/**
 * @brief Function to convert an NslVec3b bgr color to a 3-channel Mat
 * 
 * @param image : 3-channels Mat
 * @param x : x position along the horizontal axis
 * @param y : y position along the vertical axis
 * @param color : color value returned by nsl_getDistanceColor(), nsl_getAmplitudeColor()
 * 
 * @return void 
 */
void setMatrixColor(Mat image, int x, int y, NslVec3b color)
{
	image.at<Vec3b>(y,x)[0] = color.b;
	image.at<Vec3b>(y,x)[1] = color.g;
	image.at<Vec3b>(y,x)[2] = color.r;
}

/**
 * @brief Point cloud data to image conversion function
 * 
 * @param handleIndex : handle index by nsl_open()
 * 
 * @return NSL_ERROR_TYPE 
 */
void processPointCloud(NslPCD *ptNslPCD)
{
	int handle = gtViewerInfo.handle;
	bool includeDistance = false;
	bool includeAmplitude = false;
	bool includeGrayscale = false;
	bool includeRgb = false;
	int lidarWidth = ptNslPCD->lidarType != LIDAR_TYPE_OPTIONS::TYPE_B ? NSL_LIDAR_TYPE_A_WIDTH : NSL_LIDAR_TYPE_B_WIDTH;
	int lidarHeight = ptNslPCD->lidarType != LIDAR_TYPE_OPTIONS::TYPE_B ? NSL_LIDAR_TYPE_A_HEIGHT : NSL_LIDAR_TYPE_B_HEIGHT;

	Mat imageRgb = Mat(NSL_RGB_IMAGE_HEIGHT, NSL_RGB_IMAGE_WIDTH, CV_8UC3);

	gtViewerInfo.temperature = ptNslPCD->temperature;
	gtViewerInfo.operationMode = ptNslPCD->operationMode;
	gtViewerInfo.width = ptNslPCD->width;
	gtViewerInfo.height = ptNslPCD->height;
	gtViewerInfo.xMin = ptNslPCD->roiXMin;
	gtViewerInfo.xMax = ptNslPCD->roiXMax;
	gtViewerInfo.yMin = ptNslPCD->roiYMin;
	gtViewerInfo.yMax = ptNslPCD->roiYMax;
	gtViewerInfo.area_inCount = 0;

	
	if( ptNslPCD->includeRgb )
	{
		int totalPixels = NSL_RGB_IMAGE_HEIGHT * NSL_RGB_IMAGE_WIDTH;
		cv::Vec3b* dstPtr = imageRgb.ptr<cv::Vec3b>();
		NslOption::NslVec3b* srcPtr = &ptNslPCD->rgb[0][0];

		for (int i = 0; i < totalPixels; ++i) {
		    dstPtr[i] = cv::Vec3b(
		        srcPtr[i].b,  // blue
		        srcPtr[i].g,  // green
		        srcPtr[i].r   // red
		    );
		}
		
		if( gtViewerInfo.drawView ) includeRgb = true;
	}

	if( ptNslPCD->includeLidar )
	{
		int width = ptNslPCD->width;
		int height = ptNslPCD->height;
		int xMin = ptNslPCD->roiXMin;
		int yMin = ptNslPCD->roiYMin;

		Mat imageDistance = Mat(lidarHeight, lidarWidth, CV_8UC3, Scalar(255,255,255));
		Mat imageAmplitude = Mat(lidarHeight, lidarWidth, CV_8UC3, Scalar(255,255,255));

		if( gtViewerInfo.drawView ){
			includeDistance = ptNslPCD->operationMode != OPERATION_MODE_OPTIONS::GRAYSCALE_MODE ? true : false;
			includeAmplitude = (ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE 
								|| ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE) ? true : false;
			includeGrayscale = (ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE 
								|| ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::GRAYSCALE_MODE 
								|| ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE) ? true : false;
		}
		
#ifdef __USED_PCL_LIBLARY__
		gtViewerInfo.clouds[0]->clear();
		if( gtViewerInfo.clouds[0]->width != width || gtViewerInfo.clouds[0]->height != height){
			gtViewerInfo.clouds[0]->width = width;
			gtViewerInfo.clouds[0]->height = height;
			gtViewerInfo.clouds[0]->points.resize(width*height);
		}
#endif

		for(int y = 0, index = 0; y < height; y++)
		{
			for(int x = 0; x < width; x++, index++)
			{
				setMatrixColor(imageDistance, x+xMin, y+yMin, nsl_getDistanceColor(ptNslPCD->distance2D[y+yMin][x+xMin]));
				setMatrixColor(imageAmplitude, x+xMin, y+yMin, nsl_getAmplitudeColor(ptNslPCD->amplitude[y+yMin][x+xMin]));

#ifdef __USED_PCL_LIBLARY__
				if( ptNslPCD->distance3D[OUT_Z][y+yMin][x+xMin] < NSL_LIMIT_FOR_VALID_DATA ){
					pcl::PointXYZRGB &point = gtViewerInfo.clouds[0]->points[index];

					point.x = (double)(ptNslPCD->distance3D[OUT_X][y+yMin][x+xMin]/1000);
					point.y = (double)(ptNslPCD->distance3D[OUT_Y][y+yMin][x+xMin]/1000);
					point.z = (double)(ptNslPCD->distance3D[OUT_Z][y+yMin][x+xMin]/1000);

					if( gtViewerInfo.area_enable )
					{
						if( ptNslPCD->distance3D[OUT_X][y+yMin][x+xMin] >= gtViewerInfo.area_left && ptNslPCD->distance3D[OUT_X][y+yMin][x+xMin] <= gtViewerInfo.area_right
							&& ptNslPCD->distance3D[OUT_Y][y+yMin][x+xMin] >= gtViewerInfo.area_top && ptNslPCD->distance3D[OUT_Y][y+yMin][x+xMin] <= gtViewerInfo.area_bottom
							&& ptNslPCD->distance3D[OUT_Z][y+yMin][x+xMin] >= gtViewerInfo.area_start && ptNslPCD->distance3D[OUT_Z][y+yMin][x+xMin] <= gtViewerInfo.area_end )
						{
							NslVec3b color3D = nsl_getDistanceColor(ptNslPCD->distance3D[OUT_Z][y+yMin][x+xMin]);
							point.b = color3D.b;
							point.g = color3D.g;
							point.r = color3D.r;
							gtViewerInfo.area_inCount++;
						}
						else{
							point.b = 196;
							point.g = 196;
							point.r = 196;
						}
					}
					else{
						NslVec3b color3D = nsl_getDistanceColor(ptNslPCD->distance3D[OUT_Z][y+yMin][x+xMin]);
						point.b = color3D.b;
						point.g = color3D.g;
						point.r = color3D.r;
					}
				}
#endif
			}
		}

			
#ifdef __USED_PCL_LIBLARY__
		if( gtViewerInfo.drawView & DRAW_POINT_CLOUD )
			drawPointCloud();
#endif
		if( includeDistance && (gtViewerInfo.drawView & DRAW_OPENCV_VIEW) ){
			char distanceViewName[100];
			int scaleSize = ptNslPCD->lidarType != LIDAR_TYPE_OPTIONS::TYPE_B ? 2 : 1;
			int distanceWidth = lidarWidth*scaleSize;
			int distanceHeight = lidarHeight*scaleSize;
	
			if( includeAmplitude ){
				if( includeRgb )
					sprintf(distanceViewName,"RGB & Distance & Amplitude 2D <%d>", handle);
				else
					sprintf(distanceViewName,"Distance & Amplitude 2D <%d>", handle);
	
				distanceWidth = distanceWidth*2;
				hconcat(imageDistance, imageAmplitude, imageDistance);
			}
			else if( includeGrayscale ){
				if( includeRgb )
					sprintf(distanceViewName,"RGB & Distance & Grayscale 2D <%d>", handle);
				else
					sprintf(distanceViewName,"Distance & Grayscale 2D <%d>", handle);
	
				distanceWidth = distanceWidth*2;
				hconcat(imageDistance, imageAmplitude, imageDistance);
			}
			else{
				if( includeRgb )
					sprintf(distanceViewName,"RGB & Distance 2D <%d>", handle);
				else
					sprintf(distanceViewName,"Distance 2D <%d>", handle);
			}

			cv::resize( imageDistance, imageDistance, cv::Size( distanceWidth, distanceHeight ), 0, 0, INTER_LINEAR );
			
			if( includeRgb ){
				includeRgb = false;
	
				resize( imageRgb, imageRgb, Size( imageDistance.cols, imageDistance.rows ), 0, 0);
				vconcat( imageDistance, imageRgb, imageDistance );
	
				distanceHeight += 240;
			}
	
			imageDistance = addDistanceInfo(imageDistance, ptNslPCD, lidarWidth, lidarHeight, scaleSize);

			namedWindow(distanceViewName, WINDOW_NORMAL);
			resizeWindow(distanceViewName, distanceWidth, distanceHeight);
			imshow(distanceViewName, imageDistance);
			setMouseCallback(distanceViewName, mouseCallbackCV);
		}
		else if( includeGrayscale && (gtViewerInfo.drawView & DRAW_OPENCV_VIEW) ){
			char distanceViewName[100];
			sprintf(distanceViewName,"Grayscale 2D <%d>", handle);

			namedWindow(distanceViewName, WINDOW_NORMAL);
			resizeWindow(distanceViewName, 640, 480);
			imshow(distanceViewName, imageAmplitude);
			setMouseCallback(distanceViewName, mouseCallbackCV);
		}
	}
	else if( includeRgb && (gtViewerInfo.drawView & DRAW_OPENCV_VIEW) ) 
	{
		char distanceViewName[100];
		sprintf(distanceViewName,"RGB <%d>", handle);

		namedWindow(distanceViewName, WINDOW_NORMAL);
		resizeWindow(distanceViewName, 640, 480 );
		imshow(distanceViewName, imageRgb);
		setMouseCallback(distanceViewName, mouseCallbackCV);
	}

	if( gtViewerInfo.startLog ){
		logData(ptNslPCD);
	}

}


bool CaptureData()
{
	if( nsl_getPointCloudData(gtViewerInfo.handle, latestFrame.get()) == NSL_ERROR_TYPE::NSL_SUCCESS )
	{
		gtViewerInfo.frameCount++;
		return true;
	}

	return false;
}

//////////////////////////////////// main function /////////////////////////////////////////////

/*
	ubuntu usb device
	
	sudo apt-get install libopencv-dev
	sudo apt-get install libpcl-dev(1.8.1)

	$ sudo vi /etc/udev/rules.d/defined_lidar.rules
	KERNEL=="ttyACM*", ATTRS{idVendor}=="1FC9", ATTRS{idProduct}=="0094", MODE:="0777",SYMLINK+="ttyNsl3140"

	$ service udev reload
	$ service udev restart

	ubuntu Network UDP speed up
	sudo sysctl -w net.core.rmem_max=22020096
	sudo sysctl -w net.core.rmem_default=22020096
*/
int main(int argc, char *argv[]) 
{
	thread timeThread;
#ifdef _WINDOWS
	int ret = findPortsByVidPid("1FC9", "0094", gtViewerInfo.ipAddress);
	if( ret < 0 ){
		printf("findPortsByVidPid:: not find com port :: defined ipaddr = %s\n", gtViewerInfo.ipAddress);
	}
	else{
		printf("findPortsByVidPid:: find com port = %s\n", gtViewerInfo.ipAddress);
	}
#endif

	if( argc > 1 ){
		printf("changed User's IP : %s -> %s\n", gtViewerInfo.ipAddress, argv[1]);
		snprintf(gtViewerInfo.ipAddress, 20, "%s", argv[1]);
	}
		
#ifdef __USED_PCL_LIBLARY__
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->clear();
	cloud->is_dense = false;
	//point_cloud_ptr->reserve(NSL_LIDAR_IMAGE_WIDTH * NSL_LIDAR_IMAGE_HEIGHT);
	cloud->width = NSL_LIDAR_TYPE_B_WIDTH;
	cloud->height = NSL_LIDAR_TYPE_B_HEIGHT;
	cloud->points.resize(cloud->width*cloud->height);

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("NSL PCL 3D PointCloud"));
	viewer->getRenderWindow()->SetWindowName("NSL PCL 3D Viewer");
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "Viewer pointCloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Viewer pointCloud");
	viewer->addCoordinateSystem(1.0);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, -5, 0, 0, 0, 0, -1, 0, 0);
	viewer->setShowFPS(false);
	 // keyboard event callback
	viewer->registerKeyboardCallback(onKeyboardEvent, (void*)viewer.get());
//	viewer->setSize(1280, 960);
	viewer->setSize(640, 480);

	vtkSmartPointer<vtkRenderer> renderer = viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer();
	vtkCamera* camera = renderer->GetActiveCamera();
	camera->SetClippingRange(0.01, 50.0);  // 1cm ~ 50m

	viewer->addText("NANOSYSTEMS, PointCloud Sample !!!", 1070, 30, "nsl-pcl");

	if( gtViewerInfo.area_enable ){
		viewer->addText("Detection : ", 1070, 50, "point-cnt");
		viewer->addCube(gtViewerInfo.area_left/1000.0f, gtViewerInfo.area_right/1000.0f, gtViewerInfo.area_top/1000.0f, gtViewerInfo.area_bottom/1000.0f, gtViewerInfo.area_start, gtViewerInfo.area_end/1000.0f, 1.0, 1.0, 1.0, "area_box");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
											pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
											"area_box");
		
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
											3.0,
											"area_box");
	}
	
	gtViewerInfo.clouds.push_back(cloud);
	gtViewerInfo.viewers.push_back(viewer);
#endif	
	timeThread = thread(timeCheckThread, 0);

	createDirectory(LOG_FOLDER);

	gtViewerInfo.nslConfig.lidarAngle = 0;
	gtViewerInfo.nslConfig.lensType = NslOption::LENS_TYPE::LENS_SF;
	gtViewerInfo.handle = nsl_open(gtViewerInfo.ipAddress, &gtViewerInfo.nslConfig, FUNCTION_OPTIONS::FUNC_ON);
	if( gtViewerInfo.handle < 0 ){
		printf("nsl_open::handle open error::%d\n", gtViewerInfo.handle);
		exit(0);
	}

#if 0 // option example
	nsl_setModulation(gtViewerInfo.handle, MODULATION_OPTIONS::MOD_12Mhz, MODULATION_CH_OPTIONS::MOD_CH0, FUNCTION_OPTIONS::FUNC_OFF);
	nsl_setFrameRate(gtViewerInfo.handle, FRAME_RATE_OPTIONS::FRAME_15FPS);
	nsl_setIntegrationTime(gtViewerInfo.handle, 1000, 400, 50, 100);
	nsl_setGrayscaleillumination(gtViewerInfo.handle, FUNCTION_OPTIONS::FUNC_OFF);
	nsl_setUdpSpeed(gtViewerInfo.handle, UDP_SPEED_OPTIONS::NET_1000Mbps);
	nsl_setCorrection(gtViewerInfo.handle, FUNCTION_OPTIONS::FUNC_ON, FUNCTION_OPTIONS::FUNC_ON, FUNCTION_OPTIONS::FUNC_OFF, FUNCTION_OPTIONS::FUNC_OFF);
	nsl_setBinning(gtViewerInfo.handle, FUNCTION_OPTIONS::FUNC_OFF, FUNCTION_OPTIONS::FUNC_OFF);
	nsl_setHdrMode(gtViewerInfo.handle, HDR_OPTIONS::HDR_NONE_MODE);
	nsl_setAdcOverflowSaturation(gtViewerInfo.handle, FUNCTION_OPTIONS::FUNC_ON, FUNCTION_OPTIONS::FUNC_ON);
	nsl_setDualBeam(gtViewerInfo.handle, DUALBEAM_MOD_OPTIONS::DB_OFF, DUALBEAM_OPERATION_OPTIONS::DB_CORRECTION);
	nsl_setFilter(gtViewerInfo.handle, FUNCTION_OPTIONS::FUNC_OFF, FUNCTION_OPTIONS::FUNC_OFF, 300, 200, 300, 400, FUNCTION_OPTIONS::FUNC_ON);
	nsl_setRoi(gtViewerInfo.handle, 0, 0, 319, 239);
	nsl_setBinning(gtViewerInfo.handle, FUNCTION_OPTIONS::FUNC_OFF, FUNCTION_OPTIONS::FUNC_OFF);
	nsl_set3DFilter(gtViewerInfo.handle, 100);
	nsl_saveConfiguration(gtViewerInfo.handle);
	nsl_getCurrentConfig(gtViewerInfo.handle, &gtViewerInfo.nslConfig);
#endif

	nsl_setModulation(gtViewerInfo.handle, MODULATION_OPTIONS::MOD_12Mhz, MODULATION_CH_OPTIONS::MOD_CH0, FUNCTION_OPTIONS::FUNC_OFF);
	nsl_setFrameRate(gtViewerInfo.handle, FRAME_RATE_OPTIONS::FRAME_15FPS);
	nsl_setColorRange(13000, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_ON);
	nsl_setIntegrationTime(gtViewerInfo.handle, 500, 100, 0, 100);
	nsl_setFilter(gtViewerInfo.handle, FUNCTION_OPTIONS::FUNC_ON, FUNCTION_OPTIONS::FUNC_ON, 300, 100, 100, 0, FUNCTION_OPTIONS::FUNC_OFF);
	nsl_set3DFilter(gtViewerInfo.handle, 100);
//	nsl_setFilter(gtViewerInfo.handle, FUNCTION_OPTIONS::FUNC_OFF, FUNCTION_OPTIONS::FUNC_OFF, 0, 0, 0, 0, FUNCTION_OPTIONS::FUNC_OFF);
//	nsl_set3DFilter(gtViewerInfo.handle, 0);
	nsl_getCurrentConfig(gtViewerInfo.handle, &gtViewerInfo.nslConfig);
	printConfiguration();	

//	nsl_streamingOn(gtViewerInfo.handle, OPERATION_MODE_OPTIONS::GRAYSCALE_MODE);
	nsl_streamingOn(gtViewerInfo.handle, OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE);
//	nsl_streamingOn(gtViewerInfo.handle, OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE);
//	nsl_streamingOn(gtViewerInfo.handle, OPERATION_MODE_OPTIONS::DISTANCE_MODE);
//	nsl_streamingOn(gtViewerInfo.handle, OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE);
//	nsl_streamingOn(gtViewerInfo.handle, OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE);

	while( gtViewerInfo.mainRunning != 0 )
	{
		if( CaptureData() ){
			processPointCloud(latestFrame.get());
		}

		int key = waitKey(1);
		if( key == 27 ){ // ESC
			gtViewerInfo.mainRunning = 0;
			break;
		}
		else if( key == 'd' ){
			gtViewerInfo.drawView ^= DRAW_OPENCV_VIEW;
			if( gtViewerInfo.drawView & DRAW_OPENCV_VIEW ) printf("Opencv DrawView On\n");
			else printf("Opencv DrawView Off\n");
		}
		
		//usleep(100);
	}

	nsl_streamingOff(gtViewerInfo.handle); // off
	nsl_close();

	if (timeThread.joinable())	timeThread.join();
	
	cv::destroyAllWindows();
	printf("end sample main\n");
	
    return 0;
}
