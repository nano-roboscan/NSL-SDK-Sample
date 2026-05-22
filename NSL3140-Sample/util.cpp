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

void mouseCallbackRgbCV(int event, int x, int y, int flags, void* user_data)
{
	(void)flags;
	(void)user_data;
	if (event == EVENT_LBUTTONUP)
	{
		gtViewerInfo.mouseRgbX = x;
		gtViewerInfo.mouseRgbY = y;
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


void timeCheckThread(int void_data)
{
	(void)void_data;
	while( gtViewerInfo.mainRunning != 0 ){
		timeDelay(1000);
		int count = gtViewerInfo.frameCount;
		gtViewerInfo.drawframeCount = count;
		gtViewerInfo.frameCount = 0;
		gtViewerInfo.oneSecond ++;

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


void drawCube(cv::Mat& imageMat, float roll, float pitch) 
{
	int roiSize = imageMat.cols / 15;
	cv::Rect roiRect(imageMat.cols - roiSize - 5, imageMat.rows - roiSize - 5, roiSize, roiSize);
	cv::Mat roi = imageMat(roiRect); 

	roi.setTo(cv::Scalar(50, 50, 50)); 
	cv::rectangle(imageMat, roiRect, cv::Scalar(200, 200, 200), 1);
	if (roi.empty()) return;

	std::vector<cv::Point3f> objectPoints;
	objectPoints.push_back(cv::Point3f(-1, -1, -1)); // 0
	objectPoints.push_back(cv::Point3f( 1, -1, -1)); // 1
	objectPoints.push_back(cv::Point3f( 1,	1, -1)); // 2
	objectPoints.push_back(cv::Point3f(-1,	1, -1)); // 3
	objectPoints.push_back(cv::Point3f(-1, -1,	1)); // 4
	objectPoints.push_back(cv::Point3f( 1, -1,	1)); // 5
	objectPoints.push_back(cv::Point3f( 1,	1,	1)); // 6
	objectPoints.push_back(cv::Point3f(-1,	1,	1)); // 7

	double cx = roi.cols / 2.0;
	double cy = roi.rows / 2.0;
	float focalLength = roiSize * 0.66f;

	cv::Mat K = (cv::Mat_<double>(3, 3) << focalLength, 0, cx, 0, focalLength, cy, 0, 0, 1);
	cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

	cv::Vec3f rvec(pitch * static_cast<float>(CV_PI) / 180.0f, 0, roll * static_cast<float>(CV_PI) / 180.0f);
	cv::Vec3f tvec(0, 0, 3);

	std::vector<cv::Point2f> imagePoints;
	cv::projectPoints(objectPoints, rvec, tvec, K, distCoeffs, imagePoints);

	if (imagePoints.size() >= 8) {
		std::vector<cv::Point> facePoints;
		facePoints.push_back(cv::Point((int)imagePoints[0].x, (int)imagePoints[0].y));
		facePoints.push_back(cv::Point((int)imagePoints[1].x, (int)imagePoints[1].y));
		facePoints.push_back(cv::Point((int)imagePoints[5].x, (int)imagePoints[5].y));
		facePoints.push_back(cv::Point((int)imagePoints[4].x, (int)imagePoints[4].y));

		cv::fillConvexPoly(roi, facePoints, cv::Scalar(0, 100, 255), cv::LINE_AA);
	}

	int edges[] = {0,1, 1,2, 2,3, 3,0, 4,5, 5,6, 6,7, 7,4, 0,4, 1,5, 2,6, 3,7};
	for (int i = 0; i < 24; i += 2) {
		int startIdx = edges[i];
		int endIdx  = edges[i + 1];

		if (startIdx < imagePoints.size() && endIdx < imagePoints.size()) {
			cv::line(roi, imagePoints[startIdx], imagePoints[endIdx], 
					 cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
		}
	}
}


