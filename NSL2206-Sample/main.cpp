
//#define __USED_PCL_LIBLARY__

#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <chrono>

#ifdef _WINDOWS
#include <windows.h>
#include <setupapi.h>
#include <devguid.h>
#include <regstr.h>

#pragma comment(lib, "setupapi.lib")
#endif

#ifdef __USED_PCL_LIBLARY__
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#endif


#include <opencv2/opencv.hpp>

#include "nanolib.h"

using namespace cv;
using namespace std;
using namespace NslOption;

#define DISTANCE_INFO_HEIGHT	80
#define VIEWER_SCALE_SIZE		4

typedef struct ViewerInfo_
{
	// for management
	int 	drawView;
	int 	mainRunning;
	int		mouseX;
	int		mouseY;
	int 	frameCount;
	int 	drawframeCount;
	char 	portName[256];
	double	temperature;
	int 	handle;
	int 	width;
	int		height;
	OPERATION_MODE_OPTIONS 	operationMode;

	// for Lidar
	NslConfig			nslConfig;

	// for PCL
#ifdef __USED_PCL_LIBLARY__
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;	  
	vector<pcl::visualization::PCLVisualizer::Ptr> viewers;
#endif
	ViewerInfo_(){
		drawView = 1;
		mainRunning = 1;
		mouseX = -1;
		mouseY = -1;


#ifdef __USED_PCL_LIBLARY__
		cout << "PCL Version: " << PCL_VERSION_PRETTY << endl;
#endif
		frameCount = 0;
		drawframeCount = 0;
		temperature = 0;
		handle = -1;

		memset(&nslConfig, 0, sizeof(NslConfig));
		nslConfig.lidarAngleH = 0;
		nslConfig.lidarAngleV = 0;
		
		sprintf(portName,"/dev/ttyNsl2206");
	}

}VIEWER_INFO, *LP_VIEWER_INFO;


VIEWER_INFO	gtViewerInfo;
std::vector<std::unique_ptr<NslPCD>> backupFrame;
std::mutex backupFrameMutex;

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
		gtViewerInfo.drawView ^= 1;
		if( gtViewerInfo.drawView ) printf("gtViewerInfo.drawView On\n");
		else printf("gtViewerInfo.drawView Off\n");
    }
}

void drawPointCloud(Mat imageDistance)
{
	pcl::visualization::PCLVisualizer::Ptr viewer = gtViewerInfo.viewers[0];
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr = gtViewerInfo.clouds[0];

	if( !viewer->wasStopped() && point_cloud_ptr->points.size() > 0 ){
		viewer->updatePointCloud(point_cloud_ptr, "Viewer pointCloud");
		viewer->spinOnce();
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
	printf("HDR = %s\n", toString(gtViewerInfo.nslConfig.hdrOpt));
	printf("int time = %d.%d.%d.%d, gray = %d\n", gtViewerInfo.nslConfig.integrationTime3D[0]
									, gtViewerInfo.nslConfig.integrationTime3D[1]
									, gtViewerInfo.nslConfig.integrationTime3D[2]
									, gtViewerInfo.nslConfig.integrationTime3D[3]
									, gtViewerInfo.nslConfig.integrationTimeGrayScale);
	printf("roi = %d,%d,%d,%d\n", gtViewerInfo.nslConfig.roiXMin
								, gtViewerInfo.nslConfig.roiYMin
								, gtViewerInfo.nslConfig.roiXMax
								, gtViewerInfo.nslConfig.roiYMax );
	printf("Modulation = %s, ch = %s\n", toString(gtViewerInfo.nslConfig.mod_frequencyOpt), toString(gtViewerInfo.nslConfig.mod_channelOpt));
	printf("filter median = %s, gauss = %s, temporal factor = %d, temporal threshold = %d, edge threshold = %d, interferenceLimit = %d, used interference Last value = %s\n"
		, toString(gtViewerInfo.nslConfig.medianOpt)
		, toString(gtViewerInfo.nslConfig.gaussOpt)
		, gtViewerInfo.nslConfig.temporalFactorValue
		, gtViewerInfo.nslConfig.temporalThresholdValue
		, gtViewerInfo.nslConfig.edgeThresholdValue
		, gtViewerInfo.nslConfig.interferenceDetectionLimitValue
		, toString(gtViewerInfo.nslConfig.interferenceDetectionLastValueOpt));

	printf("frame rate = %s\n", toString(gtViewerInfo.nslConfig.frameRateOpt));

	printf("------------------------------------------------------------------------\n");
}

char *getDataTypeName(OPERATION_MODE_OPTIONS type)
{
	switch(type)
	{
		case OPERATION_MODE_OPTIONS::DISTANCE_MODE:
			return (char *)"D";
		case OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE:
			return (char *)"DA";
	}

	return (char *)"NONE";
}

void timeDelay(int milli)
{
	auto lastTime = chrono::steady_clock::now();
	while( gtViewerInfo.mainRunning != 0 ){
		auto now = chrono::steady_clock::now();
		auto elapsed = chrono::duration_cast<chrono::milliseconds>(now - lastTime).count();
		
		if (elapsed >= milli) {
			lastTime = now;
			return;
		}
		else{
			auto wakeUpTime = chrono::steady_clock::now() + chrono::milliseconds(1);
			this_thread::sleep_until(wakeUpTime);
		}
	}
}

void timeCheckThread(int void_data)
{
	while( gtViewerInfo.mainRunning != 0 ){
		timeDelay(1000);
		int count = gtViewerInfo.frameCount;
		gtViewerInfo.frameCount = 0;
		gtViewerInfo.drawframeCount = count;

		printf("### [%s][%dx%d] :: frame count = %d, %.2f'C ###\n", toString(gtViewerInfo.operationMode), gtViewerInfo.width, gtViewerInfo.height, count, gtViewerInfo.temperature);
	}
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
Mat addDistanceInfo(Mat distMat, NslPCD *ptNslPCD, int lidarWidth, int lidarHeight)
{
	int width = ptNslPCD->width;
	int height = ptNslPCD->height;
	int viewer_xpos = gtViewerInfo.mouseX;
	int viewer_ypos = gtViewerInfo.mouseY;
	float textSize = 0.8f;
	int xMin = ptNslPCD->roiXMin;
	int yMin = ptNslPCD->roiYMin;
	int xpos = viewer_xpos/VIEWER_SCALE_SIZE;
	int ypos = viewer_ypos/VIEWER_SCALE_SIZE;

	if( (ypos >= yMin && ypos < lidarHeight)){
		
		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		line(distMat, Point(viewer_xpos-10, viewer_ypos), Point(viewer_xpos+10, viewer_ypos), Scalar(255, 255, 0), 2);
		line(distMat, Point(viewer_xpos, viewer_ypos-10), Point(viewer_xpos, viewer_ypos+10), Scalar(255, 255, 0), 2);

		if( xpos >= lidarWidth ){ 
			xpos -= lidarWidth;
		}

		string dist2D_caption;
		string dist3D_caption;
		string info_caption;

		int distance2D = ptNslPCD->distance2D[ypos][xpos];
		if( distance2D > NSL_LIMIT_FOR_VALID_DATA ){

			if( distance2D == NSL_ADC_OVERFLOW )
				dist2D_caption = format("X:%d,Y:%d ADC_OVERFLOW", xpos, ypos);
			else if( distance2D == NSL_SATURATION )
				dist2D_caption = format("X:%d,Y:%d SATURATION", xpos, ypos);
			else if( distance2D == NSL_BAD_PIXEL )
				dist2D_caption = format("X:%d,Y:%d BAD_PIXEL", xpos, ypos);
			else if( distance2D == NSL_INTERFERENCE )
				dist2D_caption = format("X:%d,Y:%d INTERFERENCE", xpos, ypos);
			else if( distance2D == NSL_EDGE_DETECTED )
				dist2D_caption = format("X:%d,Y:%d EDGE_FILTERED", xpos, ypos);
			else
				dist2D_caption = format("X:%d,Y:%d LOW_AMPLITUDE", xpos, ypos);
		}
		else{
			if( ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE ) {
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
	bool includeDistance = false;
	bool includeAmplitude = false;
	
	int lidarWidth = NSL_LIDAR_WIDTH;
	int lidarHeight = NSL_LIDAR_HEIGHT;
	int width = ptNslPCD->width;
	int height = ptNslPCD->height;
	int xMin = ptNslPCD->roiXMin;
	int yMin = ptNslPCD->roiYMin;

	Mat imageDistance = Mat(lidarHeight, lidarWidth, CV_8UC3, Scalar(255,255,255));
	Mat imageAmplitude = Mat(lidarHeight, lidarWidth, CV_8UC3, Scalar(255,255,255));


	if( gtViewerInfo.drawView ){
		includeDistance = true;
		includeAmplitude = ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE ? true : false;

	}

#ifdef __USED_PCL_LIBLARY__
	gtViewerInfo.clouds[0]->clear();
	gtViewerInfo.clouds[0]->width = width;
	gtViewerInfo.clouds[0]->height = height;
	gtViewerInfo.clouds[0]->points.resize(width*height);
#endif
	gtViewerInfo.frameCount++;
	gtViewerInfo.temperature = ptNslPCD->temperature;
	gtViewerInfo.operationMode = ptNslPCD->operationMode;
	gtViewerInfo.width = ptNslPCD->width;
	gtViewerInfo.height = ptNslPCD->height;

	int index = 0;
	for(int y = 0; y < height; y++)
	{
		for(int x = 0; x < width; x++, index++)
		{
			setMatrixColor(imageDistance, x+xMin, y+yMin, nsl_getDistanceColor(ptNslPCD->distance2D[y+yMin][x+xMin]));
			setMatrixColor(imageAmplitude, x+xMin, y+yMin, nsl_getAmplitudeColor(ptNslPCD->amplitude[y+yMin][x+xMin]));
#ifdef __USED_PCL_LIBLARY__
			if( ptNslPCD->distance2D[y+yMin][x+xMin] < NSL_LIMIT_FOR_VALID_DATA ){
				pcl::PointXYZRGB &point = gtViewerInfo.clouds[0]->points[index];

				point.x = (double)(ptNslPCD->distance3D[OUT_X][y+yMin][x+xMin]/1000);
				point.y = (double)(ptNslPCD->distance3D[OUT_Y][y+yMin][x+xMin]/1000);
				point.z = (double)(ptNslPCD->distance3D[OUT_Z][y+yMin][x+xMin]/1000);
					
				if((y>=30 && y<=31) || (x>=80 && x<=81) )
				{ 
					point.b = 255;
					point.g = 255;
					point.r = 255;
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

	if( includeDistance ){
		char distanceViewName[100];
		int distanceWidth = NSL_LIDAR_WIDTH*VIEWER_SCALE_SIZE;
		int distanceHeight = NSL_LIDAR_HEIGHT*VIEWER_SCALE_SIZE;
	
		if( includeAmplitude ){
			sprintf(distanceViewName,"Distance & Amplitude 2D <%d>", gtViewerInfo.handle);
	
			distanceWidth = distanceWidth*2;
			hconcat(imageDistance, imageAmplitude, imageDistance);
		}
		else{
			sprintf(distanceViewName,"Distance 2D <%d>", gtViewerInfo.handle);
		}

		cv::resize( imageDistance, imageDistance, cv::Size( distanceWidth, distanceHeight ), 0, 0, INTER_LINEAR );
		imageDistance = addDistanceInfo(imageDistance, ptNslPCD, NSL_LIDAR_WIDTH, NSL_LIDAR_HEIGHT);
	
#ifdef __USED_PCL_LIBLARY__
		drawPointCloud(imageDistance);
#endif
		namedWindow(distanceViewName, WINDOW_NORMAL);
		resizeWindow(distanceViewName, cv::Size( distanceWidth, distanceHeight ));
		imshow(distanceViewName, imageDistance);
		setMouseCallback(distanceViewName, mouseCallbackCV);
	}

}


void nslQueueThread(int void_data)
{
	std::unique_ptr<NslPCD> frame = std::make_unique<NslPCD>();
	NSL_ERROR_TYPE ret;

	printf("start nslQueueThread()\n");
	
	while( gtViewerInfo.mainRunning != 0 )
	{	
		if( (ret = nsl_getPointCloudData(gtViewerInfo.handle, frame.get())) == NSL_ERROR_TYPE::NSL_SUCCESS )
		{
			// lock_guard is valid only until this block
			{
				std::lock_guard<std::mutex> lock(backupFrameMutex);
				backupFrame.push_back(std::move(frame));
			}
			
			frame = std::make_unique<NslPCD>(); 
		}


		timeDelay(1);
	}

	
	printf("end nslQueueThread()\n");
}

//////////////////////////////////// main function /////////////////////////////////////////////

/*
	ubuntu usb device
	
	sudo apt-get install libopencv-dev
	sudo apt-get install libpcl-dev(1.8.1)

	$ sudo vi /etc/udev/rules.d/defined_lidar.rules
	KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777",SYMLINK+="ttyNsl2206"
	
	$ service udev reload
	$ service udev restart
*/
int main() 
{
	thread timeThread;
	
#ifdef _WINDOWS
	int ret = findPortsByVidPid("0483", "5740", gtViewerInfo.portName);
	if( ret != 0 ) {
		printf("not find com port\n");
		return 0;
	}	
#endif
#ifdef __USED_PCL_LIBLARY__
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->clear();
	cloud->is_dense = false;
	//point_cloud_ptr->reserve(NSL_LIDAR_IMAGE_WIDTH * NSL_LIDAR_IMAGE_HEIGHT);
	cloud->width = NSL_LIDAR_WIDTH;
	cloud->height = NSL_LIDAR_HEIGHT;
	cloud->points.resize(cloud->width*cloud->height);

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("NSL PCL 3D <" + to_string(0) + ">"));
	viewer->getRenderWindow()->SetWindowName("NSL PCL 3D Viewer");
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "Viewer pointCloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Viewer pointCloud");
	viewer->addCoordinateSystem(1.0);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, -5, 0, 0, 0, 0, -1, 0, 0);
	viewer->setShowFPS(false);
	 // keyboard event callback
	viewer->registerKeyboardCallback(onKeyboardEvent, (void*)viewer.get());
	viewer->setSize(1280, 960);

	viewer->addText("NANOSYSTEMS, PointCloud Sample !!!", 1070, 30, "nsl-pcl");

	gtViewerInfo.clouds.push_back(cloud);
	gtViewerInfo.viewers.push_back(viewer);
#endif	

	timeThread = thread(timeCheckThread, 0);


	/*
		Before calling nsl_open, initialize lidarAngleH and lidarAngleV to the actual angles and pass them.
	*/
	gtViewerInfo.nslConfig.lidarAngleH = 0;
	gtViewerInfo.nslConfig.lidarAngleV = 0;
	gtViewerInfo.handle = nsl_open(gtViewerInfo.portName, &gtViewerInfo.nslConfig, FUNCTION_OPTIONS::FUNC_ON);
	if( gtViewerInfo.handle < 0 ){
		printf("nsl_open::handle open error::%d\n", gtViewerInfo.handle);
		exit(0);
	}

	printConfiguration();
#if 0 // option example
	nsl_setModulation(gtViewerInfo.handle, MODULATION_OPTIONS::MOD_10Mhz, MODULATION_CH_OPTIONS::MOD_CH0);
	nsl_setFrameRate(gtViewerInfo.handle, FRAME_RATE_OPTIONS::FRAME_10FPS);
	nsl_setIntegrationTime(gtViewerInfo.handle, 1000, 400, 50, 100, 100);
	nsl_setHdrMode(gtViewerInfo.handle, HDR_OPTIONS::HDR_NONE_MODE);
	nsl_setFilter(gtViewerInfo.handle, FUNCTION_OPTIONS::FUNC_OFF, FUNCTION_OPTIONS::FUNC_OFF, 300, 200, 300, 400, FUNCTION_OPTIONS::FUNC_OFF);
	nsl_setRoi(gtViewerInfo.handle, 0, 0, 159, 59);

	nsl_saveConfiguration(gtViewerInfo.handle);

	nsl_getCurrentConfig(gtViewerInfo.handle, &gtViewerInfo.nslConfig);
	printConfiguration();	
#endif

// 	nsl_streamingOn(gtViewerInfo.handle, OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE);
	nsl_streamingOn(gtViewerInfo.handle, OPERATION_MODE_OPTIONS::DISTANCE_MODE);

	thread queueThread = thread(nslQueueThread, 0);
	std::unique_ptr<NslPCD> frameToProcess;

	while( gtViewerInfo.mainRunning != 0 )
	{
		std::unique_ptr<NslPCD> frameToProcess;
		// lock_guard is valid only until this block
		{
			std::lock_guard<std::mutex> lock(backupFrameMutex);
			if (!backupFrame.empty()) {
				frameToProcess = std::move(backupFrame.front());
				backupFrame.erase(backupFrame.begin());
			}
		}
		
		if (frameToProcess) {
			processPointCloud(frameToProcess.get());
		}

		int key = waitKey(1);
		if( key == 27 ){ // ESC
			gtViewerInfo.mainRunning = 0;
			break;
		}
		else if( key == 'd' ){
			gtViewerInfo.drawView ^= 1;
			if( gtViewerInfo.drawView ) printf("gtViewerInfo.drawView On\n");
			else printf("gtViewerInfo.drawView Off\n");
		}
		
		//usleep(100);
	}

	nsl_streamingOff(gtViewerInfo.handle); // off
	nsl_close();

	if (timeThread.joinable())	timeThread.join();
	if (queueThread.joinable()) queueThread.join();
	
	cv::destroyAllWindows();
    printf("end sample main\n");
	
    return 0;
}
