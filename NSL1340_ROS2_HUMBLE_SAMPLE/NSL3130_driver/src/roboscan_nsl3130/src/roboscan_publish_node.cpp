#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sys/stat.h>
#include <unistd.h>

#include "roboscan_publish_node.hpp"

using namespace NslOption;
using namespace nanosys;
using namespace std::chrono_literals;
using namespace cv;
using namespace std;

#define WIN_NAME "NSL-3130AA IMAGE"

#define LEFTX_MAX	124	
#define RIGHTX_MIN	131
#define RIGHTX_MAX	319	
#define X_INTERVAL	4

#define LEFTY_MAX	116	
#define RIGHTY_MIN	123
#define RIGHTY_MAX	239	
#define Y_INTERVAL	2

#define DISTANCE_INFO_HEIGHT	80

std::atomic<int> x_start = -1, y_start = -1;

NslPCD tempFrame;

static void callback_mouse_click(int event, int x, int y, int flags, void* user_data)
{
	std::ignore = flags;
	std::ignore = user_data;
	
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		x_start = x;
		y_start = y;
	}
	else if (event == cv::EVENT_LBUTTONUP)
	{
	}
	else if (event == cv::EVENT_MOUSEMOVE)
	{
	}
}

roboscanPublisher::roboscanPublisher() : 
	Node("roboscan_publish_node")
#ifdef image_transfer_function
	,nodeHandle(std::shared_ptr<roboscanPublisher>(this, [](auto *) {}))
	,imageTransport(nodeHandle)
	,imagePublisher(imageTransport.advertise("roboscanImage", 1000))
#endif	
{ 
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    imgDistancePub = this->create_publisher<sensor_msgs::msg::Image>("roboscanDistance", qos_profile); 
    imgAmplPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanAmpl", qos_profile); 
    pointcloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("roboscanPointCloud", qos_profile); 


    roboscanPublisher::initialise();
    roboscanPublisher::startStreaming();
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&roboscanPublisher::parametersCallback, this, std::placeholders::_1));
	isReconfigure = false;
	mouseXpos = -1;
	mouseYpos = -1;
	runThread = true;
    publisherThread.reset(new boost::thread(boost::bind(&roboscanPublisher::publisher_callback, this)));
	subscriberThread.reset(new boost::thread(boost::bind(&roboscanPublisher::pcd_callback, this)));

    printf("\nRun rqt to view the image!\n");
    
} 

roboscanPublisher::~roboscanPublisher()
{
	runThread = false;
	publisherThread->join();
	subscriberThread->join();
	nsl_close();
	printf("\nEnd nsl_close()\n");
    printf("End roboscanPublisher()!\n");
}

void roboscanPublisher::publisher_callback()
{
	printf("start publisher_callback\n");

	while(runThread){
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
			publishFrame(frameToProcess.get());
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
	}

	while( backupFrame.size() > 0 ){
		backupFrame.erase(backupFrame.begin());
	}

	cv::destroyAllWindows();
	printf("end publisher_callback\n");
}


void roboscanPublisher::pcd_callback()
{
	printf("start pcd_callback\n");
	auto lastTime = chrono::steady_clock::now();
	int frameCount = 0;
	std::unique_ptr<NslPCD> frame = std::make_unique<NslPCD>();

	while(runThread){

		if( isReconfigure ){
			isReconfigure = false;
			reConfigure();
		}

		if( nslConfig.operationModeOpt != OPERATION_MODE_OPTIONS::NONE_MODE )
		{
			if( nsl_getPointCloudData(nsl_handle, frame.get()) == NSL_ERROR_TYPE::NSL_SUCCESS )
			{
				// lock_guard is valid only until this block
				{
					std::lock_guard<std::mutex> lock(backupFrameMutex);
					backupFrame.push_back(std::move(frame));
				}
				
				frame = std::make_unique<NslPCD>(); 
				frameCount++;			
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
		
		auto now = chrono::steady_clock::now();
		auto elapsed = chrono::duration_cast<chrono::milliseconds>(now - lastTime).count();
		if( elapsed > 1000 ){
			viewerParam.frameCount = frameCount;
			frameCount = 0;
			lastTime = now;
			printf("frame = %d fps\n", viewerParam.frameCount);
		}
		
	}

	printf("end pcd_callback\n");
}


rcl_interfaces::msg::SetParametersResult roboscanPublisher::parametersCallback( const std::vector<rclcpp::Parameter> &parameters)
{
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";
	// Here update class attributes, do some actions, etc.
	for (const auto &param: parameters)
	{
		if (param.get_name() == "D. hdr_mode")
		{
			nslConfig.hdrOpt = static_cast<NslOption::HDR_OPTIONS>(param.as_int());
		}
		else if (param.get_name() == "E. int0")
		{
			nslConfig.integrationTime3D = param.as_int();
		}
		else if (param.get_name() == "F. int1")
		{
			nslConfig.integrationTime3DHdr1 = param.as_int();
		}
		else if (param.get_name() == "G. int2")
		{
			nslConfig.integrationTime3DHdr2 = param.as_int();
		}
		else if (param.get_name() == "H. intGr")
		{
			nslConfig.integrationTimeGrayScale = param.as_int();
		}
		else if (param.get_name() == "I. minAmplitude")
		{
			nslConfig.minAmplitude = param.as_int();
		}
		else if (param.get_name() == "J. modIndex")
		{
			nslConfig.mod_frequencyOpt = static_cast<NslOption::MODULATION_OPTIONS>(param.as_int());
		}
		else if (param.get_name() == "K. channel")
		{
			nslConfig.mod_channelOpt = static_cast<NslOption::MODULATION_CH_OPTIONS>(param.as_int());
		}
		else if (param.get_name() == "L. roi_leftX")
		{
			int x1_tmp = param.as_int();

			if(x1_tmp % X_INTERVAL ) x1_tmp+=X_INTERVAL-(x1_tmp % X_INTERVAL );
			if(x1_tmp > LEFTX_MAX ) x1_tmp = LEFTX_MAX;

			nslConfig.roiXMin = x1_tmp;

		}
		else if (param.get_name() == "N. roi_rightX")
		{
			int x2_tmp = param.as_int();
			
			if((x2_tmp-RIGHTX_MIN) % X_INTERVAL)	x2_tmp-=((x2_tmp-RIGHTX_MIN) % X_INTERVAL);
			if(x2_tmp < RIGHTX_MIN ) x2_tmp = RIGHTX_MIN;
			if(x2_tmp > RIGHTX_MAX ) x2_tmp = RIGHTX_MAX;
			
			nslConfig.roiXMax = x2_tmp;
		}
		else if (param.get_name() == "M. roi_topY")
		{
			int y1_tmp = param.as_int();
			
			if(y1_tmp % Y_INTERVAL )	y1_tmp++;
			if(y1_tmp > LEFTY_MAX ) y1_tmp = LEFTY_MAX;
			
			nslConfig.roiYMin = y1_tmp;
			
			int y2_tmp = RIGHTY_MAX - y1_tmp;
			nslConfig.roiYMax = y2_tmp;
		}
		else if (param.get_name() == "O. roi_bottomY")
		{
			int y2_tmp = param.as_int();
			
			if(y2_tmp % Y_INTERVAL == 0 )	y2_tmp++;
			if(y2_tmp < RIGHTY_MIN ) y2_tmp = RIGHTY_MIN;
			if(y2_tmp > RIGHTY_MAX ) y2_tmp = RIGHTY_MAX;
			
			nslConfig.roiYMax = y2_tmp;
			
			int y1_tmp = RIGHTY_MAX - y2_tmp;
			nslConfig.roiYMin = y1_tmp;
		}
		else if (param.get_name() == "R. medianFilter")
		{
			nslConfig.medianOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
		}
		else if (param.get_name() == "S. gaussianFilter")
		{
			nslConfig.gaussOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
		}
		else if (param.get_name() == "T. temporalFilterFactor")
		{
			nslConfig.temporalFactorValue = static_cast<int>(param.as_double()*1000);
			if( nslConfig.temporalFactorValue > 1000 ) nslConfig.temporalFactorValue = 1000;
			if( nslConfig.temporalFactorValue < 0 ) nslConfig.temporalFactorValue = 0;
		}
		else if (param.get_name() == "T. temporalFilterFactorThreshold")
		{
			nslConfig.temporalThresholdValue = param.as_int();
			if( nslConfig.temporalThresholdValue < 0 ) nslConfig.temporalThresholdValue = 0;
		}
		else if (param.get_name() == "U. edgeFilterThreshold")
		{
			nslConfig.edgeThresholdValue = param.as_int();
			if( nslConfig.edgeThresholdValue < 0 ) nslConfig.edgeThresholdValue = 0;
		}
		else if (param.get_name() == "V. interferenceDetectionLimit")
		{
			nslConfig.interferenceDetectionLimitValue = param.as_int();
			if( nslConfig.interferenceDetectionLimitValue > 1000 ) nslConfig.interferenceDetectionLimitValue = 1000;
			if( nslConfig.interferenceDetectionLimitValue < 0 ) nslConfig.interferenceDetectionLimitValue = 0;
		}
		else if (param.get_name() == "V. useLastValue")
		{
			nslConfig.interferenceDetectionLastValueOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
		}
		else if (param.get_name() == "W. dualBeam")
		{
			int dualBeam = param.as_int();
			if( dualBeam > 2 || dualBeam < 0 ) dualBeam = 0;
			nslConfig.dbModOpt = static_cast<NslOption::DUALBEAM_MOD_OPTIONS>(dualBeam);
		}
		else if (param.get_name() == "X. dualBeam option")
		{
			int dualBeamOpt = param.as_int();
			if( dualBeamOpt > 2 || dualBeamOpt < 0 ) dualBeamOpt = 0;
			nslConfig.dbOpsOpt = static_cast<NslOption::DUALBEAM_OPERATION_OPTIONS>(dualBeamOpt);
		}
		else if (param.get_name() == "Y. grayscale LED")
		{
			nslConfig.grayscaleIlluminationOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
		}
		else if (param.get_name() == "A. cvShow")
		{
			
			bool showCv = param.as_bool();
			if( viewerParam.cvShow != showCv ){
				viewerParam.cvShow = showCv;
				viewerParam.changedCvShow = true;
			}
			
		}
		else if (param.get_name() == "B. lensType")
		{
			nslConfig.lensType = static_cast<NslOption::LENS_TYPE>(param.as_int());
			viewerParam.reOpenLidar = true;
		}
		else if (param.get_name() == "C. imageType")
		{
			int imgType = param.as_int();
			if( static_cast<int>(nslConfig.operationModeOpt) != imgType ){
				nslConfig.operationModeOpt = static_cast<NslOption::OPERATION_MODE_OPTIONS>(imgType);
				viewerParam.changedCvShow = true;
			}
		}
		else if (param.get_name() == "P. transformAngle")
		{
			nslConfig.lidarAngle = param.as_double();
			viewerParam.reOpenLidar = true;
		}
		else if (param.get_name() == "0. IP Addr")
		{
			string tmpIp = param.as_string();
			if( tmpIp != viewerParam.ipAddr ) {
				printf("changed IP addr %s -> %s\n", viewerParam.ipAddr.c_str(), tmpIp.c_str());

				viewerParam.changedIpInfo = true;
				viewerParam.ipAddr = tmpIp;
			}
		}
		else if (param.get_name() == "1. Net Mask")
		{
			string tmpIp = param.as_string();
			if( tmpIp != viewerParam.netMask ) {
				printf("changed Netmask addr %s -> %s\n", viewerParam.ipAddr.c_str(), tmpIp.c_str());
				viewerParam.changedIpInfo = true;
				viewerParam.netMask= tmpIp;
			}
		}
		else if (param.get_name() == "2. GW Addr")
		{
			string tmpIp = param.as_string();
			if( tmpIp != viewerParam.gwAddr ) {
				printf("changed Gw addr %s -> %s\n", viewerParam.ipAddr.c_str(), tmpIp.c_str());
				viewerParam.changedIpInfo = true;
				viewerParam.gwAddr= tmpIp;
			}
		}
	}

	isReconfigure = true;
	return result;
}

void roboscanPublisher::timeDelay(int milli)
{
	auto lastTime = chrono::steady_clock::now();
	while( runThread != 0 ){
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



void roboscanPublisher::reConfigure()
{
	nsl_streamingOff(nsl_handle);

	if( viewerParam.changedIpInfo || viewerParam.reOpenLidar ){

		if( viewerParam.changedIpInfo ){
			// automatic reboot device
			nsl_setIpAddress(nsl_handle, viewerParam.ipAddr.c_str(), viewerParam.netMask.c_str(), viewerParam.gwAddr.c_str());
			printf("wait time 5 sec\n");
			timeDelay(5000);
		}
		
		nsl_closeHandle(nsl_handle);
		nsl_handle = nsl_open(viewerParam.ipAddr.c_str(), &nslConfig, FUNCTION_OPTIONS::FUNC_ON);

		viewerParam.changedIpInfo = false;
		viewerParam.reOpenLidar = false;
	}

	nsl_setMinAmplitude(nsl_handle, nslConfig.minAmplitude);
	nsl_setIntegrationTime(nsl_handle, nslConfig.integrationTime3D, nslConfig.integrationTime3DHdr1, nslConfig.integrationTime3DHdr2, nslConfig.integrationTimeGrayScale);
	nsl_setHdrMode(nsl_handle, nslConfig.hdrOpt);
	nsl_setFilter(nsl_handle, nslConfig.medianOpt, nslConfig.gaussOpt, nslConfig.temporalFactorValue, nslConfig.temporalThresholdValue, nslConfig.edgeThresholdValue, nslConfig.interferenceDetectionLimitValue, nslConfig.interferenceDetectionLastValueOpt);
	nsl_setAdcOverflowSaturation(nsl_handle, nslConfig.overflowOpt, nslConfig.saturationOpt);
	nsl_setDualBeam(nsl_handle, nslConfig.dbModOpt, nslConfig.dbOpsOpt);
	nsl_setModulation(nsl_handle, nslConfig.mod_frequencyOpt, nslConfig.mod_channelOpt, nslConfig.mod_enabledAutoChannelOpt);
	nsl_setRoi(nsl_handle, nslConfig.roiXMin, nslConfig.roiYMin, nslConfig.roiXMax, nslConfig.roiYMax);
	nsl_setGrayscaleillumination(nsl_handle, nslConfig.grayscaleIlluminationOpt);
	
	printf("reConfigure OK!\n\n");

	startStreaming();
	setWinName();
}

void roboscanPublisher::setWinName()
{
	bool changedCvShow = viewerParam.changedCvShow;
	viewerParam.changedCvShow = false;
	
	if( changedCvShow ){
		cv::destroyAllWindows();
	}
	
	if( viewerParam.cvShow == false || changedCvShow == false ) return;

	if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_MODE){
		sprintf(winName,"%s(Dist)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE){
		sprintf(winName,"%s(Dist/Ampl)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE){
		sprintf(winName,"%s(Dist/Gray)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_MODE){
		sprintf(winName,"%s(RGB)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE){
		sprintf(winName,"%s(RGB/Dist)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE){
		sprintf(winName,"%s(RGB/Dist/Ampl)", WIN_NAME);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE){
		sprintf(winName,"%s(RGB/Dist/Gray)", WIN_NAME);
	}
	else{
		sprintf(winName,"%s(READY)", WIN_NAME);
	}

	
	cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
	cv::setWindowProperty(winName, cv::WND_PROP_TOPMOST, 1);	
	cv::setMouseCallback(winName, callback_mouse_click, NULL);
}
void roboscanPublisher::initialise()
{
	printf("Init roboscan_nsl3130 node\n");

	viewerParam.frameCount = 0;
	viewerParam.cvShow = false;
	viewerParam.changedCvShow = true;
	viewerParam.changedIpInfo = false;
	viewerParam.reOpenLidar = false;
	viewerParam.ipAddr = "192.168.0.220";
	viewerParam.netMask = "255.255.255.0";
	viewerParam.gwAddr = "192.168.0.1";

	nslConfig.lidarAngle = 0;
	nslConfig.lensType = NslOption::LENS_TYPE::LENS_SF;
	nsl_handle = nsl_open(viewerParam.ipAddr.c_str(), &nslConfig, FUNCTION_OPTIONS::FUNC_ON);
	if( nsl_handle < 0 ){
		printf("nsl_open::handle open error::%d\n", nsl_handle);
		exit(0);
	}
	
	nslConfig.overflowOpt = FUNCTION_OPTIONS::FUNC_ON;
	nslConfig.saturationOpt = FUNCTION_OPTIONS::FUNC_ON;
	nslConfig.operationModeOpt = OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE;

	nsl_streamingOff(nsl_handle);
	nsl_setMinAmplitude(nsl_handle, nslConfig.minAmplitude);
	nsl_setIntegrationTime(nsl_handle, nslConfig.integrationTime3D, nslConfig.integrationTime3DHdr1, nslConfig.integrationTime3DHdr2, nslConfig.integrationTimeGrayScale);
	nsl_setHdrMode(nsl_handle, nslConfig.hdrOpt);
	nsl_setFilter(nsl_handle, nslConfig.medianOpt, nslConfig.gaussOpt, nslConfig.temporalFactorValue, nslConfig.temporalThresholdValue, nslConfig.edgeThresholdValue, nslConfig.interferenceDetectionLimitValue, nslConfig.interferenceDetectionLastValueOpt);
	nsl_setAdcOverflowSaturation(nsl_handle, nslConfig.overflowOpt, nslConfig.saturationOpt);
	nsl_setDualBeam(nsl_handle, nslConfig.dbModOpt, nslConfig.dbOpsOpt);
	nsl_setModulation(nsl_handle, nslConfig.mod_frequencyOpt, nslConfig.mod_channelOpt, nslConfig.mod_enabledAutoChannelOpt);
	nsl_setRoi(nsl_handle, nslConfig.roiXMin, nslConfig.roiYMin, nslConfig.roiXMax, nslConfig.roiYMax);
	nsl_setGrayscaleillumination(nsl_handle, nslConfig.grayscaleIlluminationOpt);
	
	setWinName();

	rclcpp::Parameter pIPAddr("0. IP Addr", viewerParam.ipAddr);
//	rclcpp::Parameter pNetMask("1. Net Mask", viewerParam.netMask);
//	rclcpp::Parameter pGWAddr("2. GW Addr", viewerParam.gwAddr);

	rclcpp::Parameter pCvShow("A. cvShow", viewerParam.cvShow);
	rclcpp::Parameter pLensType("B. lensType", static_cast<int>(nslConfig.lensType));
	rclcpp::Parameter pImageType("C. imageType", static_cast<int>(nslConfig.operationModeOpt));
	rclcpp::Parameter pHdr_mode("D. hdr_mode", static_cast<int>(nslConfig.hdrOpt));
	rclcpp::Parameter pInt0("E. int0", nslConfig.integrationTime3D);
	rclcpp::Parameter pInt1("F. int1", nslConfig.integrationTime3DHdr1);
	rclcpp::Parameter pInt2("G. int2", nslConfig.integrationTime3DHdr2);
	rclcpp::Parameter pIntGr("H. intGr", nslConfig.integrationTimeGrayScale);
	rclcpp::Parameter pMinAmplitude("I. minAmplitude", nslConfig.minAmplitude);
	rclcpp::Parameter pModIndex("J. modIndex", static_cast<int>(nslConfig.mod_frequencyOpt));
	rclcpp::Parameter pChannel("K. channel", static_cast<int>(nslConfig.mod_channelOpt));
	rclcpp::Parameter pRoi_leftX("L. roi_leftX", nslConfig.roiXMin);
	rclcpp::Parameter pRoi_topY("M. roi_topY", nslConfig.roiYMin);
	rclcpp::Parameter pRoi_rightX("N. roi_rightX", nslConfig.roiXMax);
	rclcpp::Parameter pTransformAngle("P. transformAngle", nslConfig.lidarAngle);
	rclcpp::Parameter pMedianFilter("R. medianFilter", static_cast<int>(nslConfig.medianOpt));
	rclcpp::Parameter pAverageFilter("S. gaussianFilter", static_cast<int>(nslConfig.gaussOpt));
	rclcpp::Parameter pTemporalFilterFactor("T. temporalFilterFactor", nslConfig.temporalFactorValue);
	rclcpp::Parameter pTemporalFilterThreshold("T. temporalFilterFactorThreshold", nslConfig.temporalThresholdValue);
	rclcpp::Parameter pEdgeFilterThreshold("U. edgeFilterThreshold", nslConfig.edgeThresholdValue);
	rclcpp::Parameter pInterferenceDetectionLimit("V. interferenceDetectionLimit", nslConfig.interferenceDetectionLimitValue);
	rclcpp::Parameter pUseLastValue("V. useLastValue", static_cast<int>(nslConfig.interferenceDetectionLastValueOpt));

	rclcpp::Parameter pDualBeam("W. dualBeam", static_cast<int>(nslConfig.dbModOpt));
	rclcpp::Parameter pDualBeamOpt("X. dualBeam option", static_cast<int>(nslConfig.dbOpsOpt));
	rclcpp::Parameter pGrayscaleilluminationOpt("Y. grayscale LED", static_cast<int>(nslConfig.grayscaleIlluminationOpt));


	this->declare_parameter<string>("0. IP Addr", viewerParam.ipAddr);
//	this->declare_parameter<string>("1. Net Mask", viewerParam.netMask);
//	this->declare_parameter<string>("2. GW Addr", viewerParam.gwAddr);
	
	this->declare_parameter<bool>("A. cvShow", viewerParam.cvShow);
	this->declare_parameter<int>("B. lensType", static_cast<int>(nslConfig.lensType));
	this->declare_parameter<int>("C. imageType", static_cast<int>(nslConfig.operationModeOpt));
	this->declare_parameter<int>("D. hdr_mode", static_cast<int>(nslConfig.hdrOpt));
	this->declare_parameter<int>("E. int0", nslConfig.integrationTime3D);
	this->declare_parameter<int>("F. int1", nslConfig.integrationTime3DHdr1);
	this->declare_parameter<int>("G. int2", nslConfig.integrationTime3DHdr2);
	this->declare_parameter<int>("H. intGr",nslConfig.integrationTimeGrayScale);
	this->declare_parameter<int>("I. minAmplitude", nslConfig.minAmplitude);
	this->declare_parameter<int>("J. modIndex", static_cast<int>(nslConfig.mod_frequencyOpt));
	this->declare_parameter<int>("K. channel", static_cast<int>(nslConfig.mod_channelOpt));
	this->declare_parameter<int>("L. roi_leftX", nslConfig.roiXMin);
	this->declare_parameter<int>("M. roi_topY",  nslConfig.roiYMin);
	this->declare_parameter<int>("N. roi_rightX", nslConfig.roiXMax);
//	this->declare_parameter<int>("O. roi_bottomY", viewerParam.roi_bottomY);
	this->declare_parameter<double>("P. transformAngle", nslConfig.lidarAngle);
	this->declare_parameter<bool>("R. medianFilter", static_cast<int>(nslConfig.medianOpt));
	this->declare_parameter<bool>("S. gaussianFilter", static_cast<int>(nslConfig.gaussOpt));
	this->declare_parameter<double>("T. temporalFilterFactor", nslConfig.temporalFactorValue);
	this->declare_parameter<int>("T. temporalFilterFactorThreshold", nslConfig.temporalThresholdValue);
	this->declare_parameter<int>("U. edgeFilterThreshold", nslConfig.edgeThresholdValue);
	this->declare_parameter<int>("V. interferenceDetectionLimit", nslConfig.interferenceDetectionLimitValue);
	this->declare_parameter<bool>("V. useLastValue", static_cast<int>(nslConfig.interferenceDetectionLastValueOpt));

	this->declare_parameter<int>("W. dualBeam", static_cast<int>(nslConfig.dbModOpt));
	this->declare_parameter<int>("X. dualBeam option", static_cast<int>(nslConfig.dbOpsOpt));
	this->declare_parameter<bool>("Y. grayscale LED", static_cast<int>(nslConfig.grayscaleIlluminationOpt));

	this->set_parameter(pIPAddr);
//	this->set_parameter(pNetMask);
//	this->set_parameter(pGWAddr);

	this->set_parameter(pLensType);
	this->set_parameter(pImageType);
	this->set_parameter(pHdr_mode);
	this->set_parameter(pInt0);
	this->set_parameter(pInt1);
	this->set_parameter(pInt2);
	this->set_parameter(pIntGr);
	this->set_parameter(pMinAmplitude);
	this->set_parameter(pModIndex);
	this->set_parameter(pChannel);
	this->set_parameter(pRoi_leftX);
	this->set_parameter(pRoi_topY);
	this->set_parameter(pRoi_rightX);
//	this->set_parameter(pRoi_bottomY);
	this->set_parameter(pTransformAngle);
//	this->set_parameter(pCutpixels);
	this->set_parameter(pMedianFilter);
	this->set_parameter(pAverageFilter);
	this->set_parameter(pTemporalFilterFactor);
	this->set_parameter(pTemporalFilterThreshold);
	this->set_parameter(pEdgeFilterThreshold);
	this->set_parameter(pInterferenceDetectionLimit);
	this->set_parameter(pUseLastValue);

	this->set_parameter(pCvShow);
	this->set_parameter(pDualBeam);
	this->set_parameter(pDualBeamOpt);
	this->set_parameter(pGrayscaleilluminationOpt);
}


void roboscanPublisher::startStreaming()
{
	if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_MODE){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::DISTANCE_MODE);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_MODE){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_MODE);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE);
	}
	else if( nslConfig.operationModeOpt == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE);
	}
	else{
		printf("operation mode NONE~~~\n");
	}
}


cv::Mat roboscanPublisher::addDistanceInfo(cv::Mat distMat, NslPCD *frame)
{
	int xpos = mouseXpos;
	int ypos = mouseYpos;
	
	if( (ypos > 0 && ypos < frame->height)){
		// mouseXpos, mouseYpos
//		int origin_xpos = xpos;
		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		line(distMat, Point(xpos-10, ypos), Point(xpos+10, ypos), Scalar(255, 255, 0), 2);
		line(distMat, Point(xpos, ypos-10), Point(xpos, ypos+10), Scalar(255, 255, 0), 2);

		if( xpos >= frame->width ){ 
			xpos -= frame->width;
		}

		string dist2D_caption;
		string dist3D_caption;
		string info_caption;

		int distance2D = frame->distance2D[ypos][xpos];
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
			if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE || frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE ) {
				dist2D_caption = format("2D X:%d Y:%d %dmm/%dlsb", xpos, ypos, frame->distance2D[ypos][xpos], frame->amplitude[ypos][xpos]);
				dist3D_caption = format("3D X:%.1fmm Y:%.1fmm Z:%.1fmm", frame->distance3D[OUT_X][ypos][xpos], frame->distance3D[OUT_Y][ypos][xpos], frame->distance3D[OUT_Z][ypos][xpos]);
			}
			else{
				dist2D_caption = format("2D X:%d Y:%d <%d>mm", xpos, ypos, frame->distance2D[ypos][xpos]);
				dist3D_caption = format("3D X:%.1fmm Y:%.1fmm Z:%.1fmm", frame->distance3D[OUT_X][ypos][xpos], frame->distance3D[OUT_Y][ypos][xpos], frame->distance3D[OUT_Z][ypos][xpos]);
			}
		}
		
		info_caption = format("%s:%dx%d %.2f'C", toString(frame->operationMode), frame->width, frame->height, frame->temperature);

		putText(infoImage, info_caption.c_str(), Point(10, 23), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, dist2D_caption.c_str(), Point(10, 46), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, dist3D_caption.c_str(), Point(10, 70), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);
		vconcat(distMat, infoImage, distMat);
	}
	else{
		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		string info_caption = format("%s:%dx%d %.2f'C", toString(frame->operationMode), frame->width, frame->height, frame->temperature);
		putText(infoImage, info_caption.c_str(), Point(10, 23), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);		
		vconcat(distMat, infoImage, distMat);
	}

	return distMat;
}


void roboscanPublisher::publishFrame(NslPCD *frame)
{
	static rclcpp::Clock s_rclcpp_clock;
	auto data_stamp = s_rclcpp_clock.now();

//	std::chrono::system_clock::time_point update_start = std::chrono::system_clock::now();
//	printf("mode = %s, width = %d/%d\n", toString(frame->operationMode), frame->width, frame->height);

	cv::Mat distanceMat(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// distance
	cv::Mat amplitudeMat(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// amplitude
	cv::Mat rgbMat(NSL_RGB_IMAGE_HEIGHT, NSL_RGB_IMAGE_WIDTH, CV_8UC3, Scalar(255, 255, 255));

/*
	if(frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE )
	{
		sensor_msgs::msg::Image imgDistance;

		std::vector<uint8_t> result;
		result.reserve(frame->height * frame->width * 2);

		int xMin = frame->xMin;
		int yMin = frame->yMin;
		
		for (int y = 0; y < frame->height; ++y) {
		    for (int x = 0; x < frame->width; ++x) {
		        result.push_back(static_cast<uint8_t>(frame->distance2D[y+yMin][x+xMin] & 0xFF));        // LSB
		        result.push_back(static_cast<uint8_t>((frame->distance2D[y+yMin][x+xMin] >> 8) & 0xFF)); // MSB
		    }
		}

		imgDistance.header.stamp = data_stamp;
		imgDistance.header.frame_id = "roboscan_frame";
		imgDistance.height = static_cast<uint32_t>(frame->height);
		imgDistance.width = static_cast<uint32_t>(frame->width);
		imgDistance.encoding = sensor_msgs::image_encodings::MONO16;
		imgDistance.step = imgDistance.width * 2;
		imgDistance.is_bigendian = 0;
		imgDistance.data = result;
		imgDistancePub->publish(imgDistance);
	}

	if(frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE)
	{
		sensor_msgs::msg::Image imgAmpl;

		std::vector<uint8_t> result;
		result.reserve(frame->height * frame->width * 2);

		int xMin = frame->xMin;
		int yMin = frame->yMin;
		
		for (int y = 0; y < frame->height; ++y) {
		    for (int x = 0; x < frame->width; ++x) {
		        result.push_back(static_cast<uint8_t>(frame->amplitude[y+yMin][x+xMin] & 0xFF));        // LSB
		        result.push_back(static_cast<uint8_t>((frame->amplitude[y+yMin][x+xMin] >> 8) & 0xFF)); // MSB
		    }
		}

		imgAmpl.header.stamp = data_stamp;
		imgAmpl.header.frame_id = "roboscan_frame";
		imgAmpl.height = static_cast<uint32_t>(frame->height);
		imgAmpl.width = static_cast<uint32_t>(frame->width);
		imgAmpl.encoding = sensor_msgs::image_encodings::MONO16;
		imgAmpl.step = imgAmpl.width * 2;
		imgAmpl.is_bigendian = 0;
		imgAmpl.data = result;
		imgAmplPub->publish(imgAmpl);
	}	
*/
#ifdef image_transfer_function
	if(frame->operationMode == OPERATION_MODE_OPTIONS::RGB_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE)
	{
		for( int y = 0;y<NSL_RGB_IMAGE_HEIGHT;y++){
			for( int x = 0;x<NSL_RGB_IMAGE_WIDTH;x++){
				rgbMat.at<Vec3b>(y,x)[0] = frame->rgb[y][x][0];
				rgbMat.at<Vec3b>(y,x)[1] = frame->rgb[y][x][1];
				rgbMat.at<Vec3b>(y,x)[2] = frame->rgb[y][x][2];
			}
		}

/*
		cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
		cv_ptr->header.stamp = data_stamp;
		cv_ptr->header.frame_id = "roboscan_frame";
		cv_ptr->image = rgbMat;
		cv_ptr->encoding = "bgr8";
	
		imagePublisher.publish(cv_ptr->toImageMsg());
*/
		
	}
#endif

	if( frame->operationMode != OPERATION_MODE_OPTIONS::RGB_MODE )
	{
		const size_t nPixel = frame->width * frame->height;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
		cloud->header.frame_id = "roboscan_frame";
		cloud->header.stamp = pcl_conversions::toPCL(data_stamp);
		//cloud->header.stamp = static_cast<uint64_t>(data_stamp.nanoseconds());
		cloud->width = static_cast<uint32_t>(frame->width);
		cloud->height = static_cast<uint32_t>(frame->height);
		cloud->is_dense = false;
		cloud->points.resize(nPixel);

		uint16_t distance = 0;
		uint16_t amplitude = 0;

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;


		for(int y = 0, index = 0; y < frame->height; y++)
		{
			for(int x = 0; x < frame->width; x++, index++)
			{
				pcl::PointXYZI &point = cloud->points[index];
				
				distance = frame->distance2D[y+yMin][x+xMin];
				amplitude = frame->amplitude[y+yMin][x+xMin];
				
				if(frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE 
					|| frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE
					|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE
					|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE)
				{
					NslVec3b color = nsl_getAmplitudeColor(amplitude);
					amplitudeMat.at<Vec3b>(y, x)[0] = color[0];
					amplitudeMat.at<Vec3b>(y, x)[1] = color[1];
					amplitudeMat.at<Vec3b>(y, x)[2] = color[2];
				}
				else{
					amplitude = static_cast<float>(distance/1000.0);
				}

				if( distance < NSL_LIMIT_FOR_VALID_DATA )
				{
					point.x = (double)(frame->distance3D[OUT_Z][y+yMin][x+xMin]/1000);
					point.y = (double)(-frame->distance3D[OUT_X][y+yMin][x+xMin]/1000);
					point.z = (double)(-frame->distance3D[OUT_Y][y+yMin][x+xMin]/1000);
					point.intensity = amplitude;
				}
				else{
					point.x = std::numeric_limits<float>::quiet_NaN();
					point.y = std::numeric_limits<float>::quiet_NaN();
					point.z = std::numeric_limits<float>::quiet_NaN();
					point.intensity = std::numeric_limits<float>::quiet_NaN();
				}
				
				NslVec3b color = nsl_getDistanceColor(distance);
				distanceMat.at<Vec3b>(y, x)[0] = color[0];
				distanceMat.at<Vec3b>(y, x)[1] = color[1];
				distanceMat.at<Vec3b>(y, x)[2] = color[2];
			}
		}
/*
		sensor_msgs::msg::PointCloud2 msg;
		pcl::toROSMsg(*cloud, msg);
		msg.header.stamp = data_stamp;
		msg.header.frame_id = "roboscan_frame";
		pointcloudPub->publish(msg);  
*/		
	}
	
	if(viewerParam.cvShow == true)
	{	
		getMouseEvent(mouseXpos, mouseYpos);
			
		if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_MODE ){
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_MODE ){
			resize( rgbMat, rgbMat, Size( 640, 480 ), 0, 0);
			distanceMat = rgbMat;
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE ){
			resize( rgbMat, rgbMat, Size( distanceMat.cols, distanceMat.rows ), 0, 0);
			vconcat( distanceMat, rgbMat, distanceMat );
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			resize( rgbMat, rgbMat, Size( distanceMat.cols, distanceMat.rows ), 0, 0);
			vconcat( distanceMat, rgbMat, distanceMat );
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			resize( rgbMat, rgbMat, Size( distanceMat.cols, distanceMat.rows ), 0, 0);
			vconcat( distanceMat, rgbMat, distanceMat );
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		
		imshow(winName, distanceMat);
		waitKey(1);
	}

}

void roboscanPublisher::getMouseEvent( int &mouse_xpos, int &mouse_ypos )
{
	mouse_xpos = x_start;
	mouse_ypos = y_start;
}


int main(int argc, char ** argv)
{
	(void) argc;
	(void) argv;
	rclcpp::init(argc, argv);

	auto node = std::make_shared<roboscanPublisher>();

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
