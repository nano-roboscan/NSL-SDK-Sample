#include "main.h"
#include <thread>
#include <chrono>


#ifdef _WINDOWS
#include <setupapi.h>
#include <devguid.h>
#include <regstr.h>

#pragma comment(lib, "setupapi.lib")
#endif


/////////////////////////////////// definition /////////////////////////////////////////////////////
#define DISTANCE_INFO_HEIGHT	120


void mouseCallbackCV(int event, int x, int y, int flags, void* user_data)
{
	(void)flags;
	(void)user_data;
	if (event == EVENT_LBUTTONUP)
	{
		gtViewerInfo.mouseX = x;
		gtViewerInfo.mouseY = y;
	}
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
 * @brief openCV draw function
 * 
 * @param distMat : Mat data to be drawn on the screen
 * @param handleIndex : handle index by nsl_open()
 * 
 * @return cv::Mat 
 */
void addDistanceInfo(Mat &distMat, Mat &finalBuffer, NslPCD *ptNslPCD, int lidarWidth, int lidarHeight, int scaleSize)
{
	int width = ptNslPCD->width;
	int height = ptNslPCD->height;
	int viewer_xpos = gtViewerInfo.mouseX;
	int viewer_ypos = gtViewerInfo.mouseY;
	float textSize = 0.8f;
//	int xMin = ptNslPCD->roiXMin;
	int yMin = ptNslPCD->roiYMin;
	int xpos = viewer_xpos/scaleSize;
	int ypos = viewer_ypos/scaleSize;
	string int_caption;

	if( gtViewerInfo.autoIntRoiEnable == FUNCTION_OPTIONS::FUNC_ON ){
		int x1 = gtViewerInfo.autoIntRoi.x_start * scaleSize;
		int y1 = gtViewerInfo.autoIntRoi.y_start * scaleSize;
		int x2 = gtViewerInfo.autoIntRoi.x_end * scaleSize;
		int y2 = gtViewerInfo.autoIntRoi.y_end * scaleSize;

		rectangle(distMat, Point(x1, y1), Point(x2, y2), Scalar(0, 255, 0), 2);

		FUNCTION_OPTIONS	isEnable;
		int currentIntTime0;
		int currentOverflowCnt;

		nsl_getAutoIntegrationTime(gtViewerInfo.handle, &isEnable, &currentIntTime0, &currentOverflowCnt);

		int_caption = format("Int <%d, %d, %d, %d>, Overflow %d", currentIntTime0, gtViewerInfo.nslConfig.integrationTime3DHdr1, gtViewerInfo.nslConfig.integrationTime3DHdr2, gtViewerInfo.nslConfig.integrationTimeGrayScale, currentOverflowCnt);
		if( isEnable == FUNCTION_OPTIONS::FUNC_OFF ){
			nsl_setAutoIntegrationTime(gtViewerInfo.handle, &gtViewerInfo.autoIntRoi, gtViewerInfo.autoIntRoiEnable);
		}
	}
	else{
		int_caption = format("Int <%d, %d, %d, %d>", gtViewerInfo.nslConfig.integrationTime3D, gtViewerInfo.nslConfig.integrationTime3DHdr1, gtViewerInfo.nslConfig.integrationTime3DHdr2, gtViewerInfo.nslConfig.integrationTimeGrayScale);
	}
	
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
		putText(infoImage, int_caption.c_str(), Point(10, 46), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);		
		putText(infoImage, dist2D_caption.c_str(), Point(10, 70), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, dist3D_caption.c_str(), Point(10, 95), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);
		vconcat(distMat, infoImage, finalBuffer);
	}
	else{
		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		string info_caption = format("%s:%dx%d <%dfps> %.2f'C", getDataTypeName(ptNslPCD->operationMode), width, height, gtViewerInfo.drawframeCount, gtViewerInfo.temperature);
		putText(infoImage, info_caption.c_str(), Point(10, 23), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);		
		putText(infoImage, int_caption.c_str(), Point(10, 46), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);			

		vconcat(distMat, infoImage, finalBuffer);
	}

	return;
}



void timeCheckThread(int void_data)
{
	(void)void_data;
	while( gtViewerInfo.mainRunning != 0 ){
		timeDelay(1000);
		int count = gtViewerInfo.frameCount;
		gtViewerInfo.drawframeCount = count;
		gtViewerInfo.frameCount = 0;

		printf("### [%s][%d<%d:%d> x %d<%d:%d>] :: frame count = %d, %.2f'C ###\n", toString(gtViewerInfo.operationMode), gtViewerInfo.width, gtViewerInfo.xMin, gtViewerInfo.xMax, gtViewerInfo.height, gtViewerInfo.yMin, gtViewerInfo.yMax, count, gtViewerInfo.temperature);
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
	printf("3D edge filter theshold = %d\n", gtViewerInfo.nslConfig.edgeThresholdValue3D);

	printf("UDP speed = %s\n", toString(gtViewerInfo.nslConfig.udpSpeedOpt));
	printf("frame rate = %s\n", toString(gtViewerInfo.nslConfig.frameRateOpt));

	unsigned int sdk_version = nsl_getSdkVersion(0);
	printf("sdk version = %d.%d\n", sdk_version>>16&0xFFFF, sdk_version&0xFFFF);

	printf("------------------------------------------------------------------------\n");
}






