#ifndef __MAIN_VIEWER_H__
#define __MAIN_VIEWER_H__

#include "nanolib.h"

//#define __USED_PCL_LIBLARY__

#ifdef _WINDOWS
#include <windows.h>
#else
#include <sys/stat.h>
#endif

#ifdef __USED_PCL_LIBLARY__
#ifdef _WINDOWS
#pragma warning(disable: 4819)
#endif

#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <vtkRendererCollection.h>
#include <vtkRenderWindow.h>
#include <vtkCamera.h>
#endif

#include <opencv2/opencv.hpp>
#include <chrono>



using namespace cv;
using namespace std;
using namespace std::chrono;
using namespace NslOption;


#define DRAW_POINT_CLOUD	0x01
#define DRAW_OPENCV_VIEW	0x02

typedef struct ViewerInfo_
{
	// for management
	int 		drawView;
	int 		mainRunning;
	int		mouseX;
	int		mouseY;
	int 		frameCount;
	int 		oneSecond;
	int 		drawframeCount;
	char 	ipAddress[20];
	double	temperature;
	int 		handle;
	int 		width;
	int		height;
	int 		xMin;
	int 		xMax;
	int		yMin;
	int 		yMax;
	bool	startLog;
	bool	streamingMode;
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

	// Auto Integration time ROI
	NslAutoIntROI autoIntRoi;	// x_start, x_end :: 0 ~ 319, y_start, y_end :: 0 ~ 239
	FUNCTION_OPTIONS	autoIntRoiEnable;
//	NslOption::NslVec3b  rgb[NSL_RGB_IMAGE_HEIGHT * NSL_RGB_IMAGE_WIDTH];
	std::vector<NslVec3b> rgb;
	std::unique_ptr<NslPCD> latestFrame;

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
		streamingMode = true;

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
		oneSecond = 0;
		frameCount = 0;
		drawframeCount = 0;
		temperature = 0;
		handle = -1;

		autoIntRoi.x_start = 40;
		autoIntRoi.y_start = 70;
		autoIntRoi.x_end = 279;
		autoIntRoi.y_end = 169;
		autoIntRoi.max_overflow = 100;	// minimum 10
		autoIntRoi.min_intTime = 100; // minimum 100
		autoIntRoiEnable = FUNCTION_OPTIONS::FUNC_OFF;

		memset(&nslConfig, 0, sizeof(NslConfig));
		nslConfig.lidarAngle = 0;
		nslConfig.lensType = NslOption::LENS_TYPE::LENS_SF;
		//operationMode = OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE;
		operationMode = OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE;
		
		if( isRgbCommand() ){
			rgb.resize(NSL_RGB_IMAGE_WIDTH * NSL_RGB_IMAGE_HEIGHT);	
		}

		latestFrame = std::make_unique<NslPCD>();
		
//		sprintf(ipAddress,"/dev/ttyNsl3140");	// virtual com device
//		sprintf(ipAddress,"\\\\.\\COM8");		// virtual com device
//		sprintf(ipAddress,"");					// Vendor specific device
		sprintf(ipAddress,"192.168.0.220");		// Network device

	}

	bool isRgbCommand()
	{
		return operationMode == OPERATION_MODE_OPTIONS::RGB_MODE
			|| operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE
			|| operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE
			|| operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE;
	}

}VIEWER_INFO, *LP_VIEWER_INFO;

#ifdef __USED_PCL_LIBLARY__
void initPclViewer();
void onKeyboardEvent(const pcl::visualization::KeyboardEvent& event, void* viewer_void) ;
void drawPointCloud();
void clearCloud();
void addPoint(pcl::PointXYZRGB point);
#endif
void saveDataPCD(NslPCD *ptNslPCD, const std::string& filePath);
void saveRGB(NslPCD *ptNslPCD, const std::string& filePath);
void saveIndex(const std::string& filePath, const std::string& str);
void createLogDirectory() ;
void logData(NslPCD *ptNslPCD);

void mouseCallbackCV(int event, int x, int y, int flags, void* user_data);
char *getDataTypeName(OPERATION_MODE_OPTIONS type);
void timeDelay(int milli);
void setMatrixColor(Mat image, int x, int y, NslVec3b color);
void addDistanceInfo(Mat &distMat, Mat &finalBuffer, NslPCD *ptNslPCD, int lidarWidth, int lidarHeight, int scaleSize);
void timeCheckThread(int void_data);
void printConfiguration();

#ifdef _WINDOWS
int findPortsByVidPid(const std::string vid, const std::string pid, char *strPortName);
#endif

extern VIEWER_INFO	gtViewerInfo;

#endif // __MAIN_VIEWER_H__

