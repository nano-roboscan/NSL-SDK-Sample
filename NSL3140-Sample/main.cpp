
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
#include "main.h"
#include <thread>
#include <chrono>

/////////////////////////////////// global variable /////////////////////////////////////////////////////
VIEWER_INFO	gtViewerInfo;

/////////////////////////////////// function /////////////////////////////////////////////////////
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

	static Mat imageRgb = Mat(NSL_RGB_IMAGE_HEIGHT, NSL_RGB_IMAGE_WIDTH, CV_8UC3);

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
		NslOption::NslVec3b* srcPtr = gtViewerInfo.rgb.data();

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
		int scaleSize = ptNslPCD->lidarType != LIDAR_TYPE_OPTIONS::TYPE_B ? 2 : 1;
		int distanceWidth = lidarWidth*scaleSize;
		int distanceHeight = lidarHeight*scaleSize; 

		static Mat imageDistance = Mat(lidarHeight, lidarWidth, CV_8UC3, Scalar(255,255,255));
		static Mat imageAmplitude = Mat(lidarHeight, lidarWidth, CV_8UC3, Scalar(255,255,255));

		if( gtViewerInfo.drawView ){
			includeDistance = (ptNslPCD->operationMode != OPERATION_MODE_OPTIONS::RGB_MODE
								&& ptNslPCD->operationMode != OPERATION_MODE_OPTIONS::GRAYSCALE_MODE) ? true : false;
			includeAmplitude = (ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE 
								|| ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE) ? true : false;
			includeGrayscale = (ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE 
								|| ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::GRAYSCALE_MODE 
								|| ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE) ? true : false;
		}
		
#ifdef __USED_PCL_LIBLARY__
		clearCloud();
#endif
		for(int y = 0, index = 0; y < height; y++)
		{
			for(int x = 0; x < width; x++, index++)
			{
				setMatrixColor(imageDistance, x+xMin, y+yMin, nsl_getDistanceColor(ptNslPCD->distance2D[y+yMin][x+xMin]));
				setMatrixColor(imageAmplitude, x+xMin, y+yMin, nsl_getAmplitudeColor(ptNslPCD->amplitude[y+yMin][x+xMin]));

#ifdef __USED_PCL_LIBLARY__
				if( ptNslPCD->distance3D[OUT_Z][y+yMin][x+xMin] < NSL_LIMIT_FOR_VALID_DATA ){
					pcl::PointXYZRGB point;
					point.x = ptNslPCD->distance3D[OUT_X][y+yMin][x+xMin]/1000.0;
					point.y = ptNslPCD->distance3D[OUT_Y][y+yMin][x+xMin]/1000.0;
					point.z = ptNslPCD->distance3D[OUT_Z][y+yMin][x+xMin]/1000.0;

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
					
					addPoint(point);
				}
#endif
			}
		}

			
#ifdef __USED_PCL_LIBLARY__
		drawPointCloud();
#endif
		if( includeDistance && (gtViewerInfo.drawView & DRAW_OPENCV_VIEW) ){
			char distanceViewName[100];
			static Mat concatBuffer;
			static Mat resizeBuffer;
			static Mat finalBuffer;

			if( includeAmplitude ){
				sprintf(distanceViewName,"Distance & Amplitude 2D <%d>", handle);
	
				distanceWidth = distanceWidth*2;
				hconcat(imageDistance, imageAmplitude, concatBuffer);
			}
			else if( includeGrayscale ){
				sprintf(distanceViewName,"Distance & Grayscale 2D <%d>", handle);
	
				distanceWidth = distanceWidth*2;
				hconcat(imageDistance, imageAmplitude, concatBuffer);
			}
			else{
				sprintf(distanceViewName,"Distance 2D <%d>", handle);
				concatBuffer = imageDistance.clone();
			}

			cv::resize( concatBuffer, resizeBuffer, cv::Size( distanceWidth, distanceHeight ), 0, 0, INTER_LINEAR );
	
			addDistanceInfo(resizeBuffer, finalBuffer, ptNslPCD, lidarWidth, lidarHeight, scaleSize);

			namedWindow(distanceViewName, WINDOW_NORMAL);
			imshow(distanceViewName, finalBuffer);
			setMouseCallback(distanceViewName, mouseCallbackCV);
			
			if( includeRgb ){
				includeRgb = false;
				static Mat resizeRgbBuffer;
				cv::resize( imageRgb, resizeRgbBuffer, cv::Size( 640, 360 ), 0, 0, INTER_LINEAR );

				sprintf(distanceViewName,"RGB <%d>", handle);
				imshow(distanceViewName, resizeRgbBuffer);
			}
		}
		else if( includeGrayscale && (gtViewerInfo.drawView & DRAW_OPENCV_VIEW) ){
			char distanceViewName[100];
			sprintf(distanceViewName,"Grayscale 2D <%d>", handle);

			namedWindow(distanceViewName, WINDOW_NORMAL);
			resizeWindow(distanceViewName, distanceWidth, distanceHeight);
			imshow(distanceViewName, imageAmplitude);
			setMouseCallback(distanceViewName, mouseCallbackCV);
		}
	}
	else if( includeRgb && (gtViewerInfo.drawView & DRAW_OPENCV_VIEW) ) 
	{
		char distanceViewName[100];

		sprintf(distanceViewName,"RGB <%d>", handle);

		namedWindow(distanceViewName, WINDOW_NORMAL);
		imshow(distanceViewName, imageRgb);
		setMouseCallback(distanceViewName, mouseCallbackCV);
	}

	if( gtViewerInfo.startLog ){
		logData(ptNslPCD);
	}

}

/**
 * @brief Read lidar & rgb data
 * 
 * @return bool 
 */
bool CaptureData()
{
	if( gtViewerInfo.isRgbCommand() ){
		if( nsl_getPointCloudRgbData(gtViewerInfo.handle, gtViewerInfo.latestFrame.get(), gtViewerInfo.rgb.data(), 1000) == NSL_ERROR_TYPE::NSL_SUCCESS )
		{
			gtViewerInfo.frameCount++;
			return true;
		}
	}
	else{
		if( nsl_getPointCloudData(gtViewerInfo.handle, gtViewerInfo.latestFrame.get(), 1000) == NSL_ERROR_TYPE::NSL_SUCCESS )
		{
			gtViewerInfo.frameCount++;
			return true;
		}
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
		//printf("findPortsByVidPid:: not find com port :: defined ipaddr = %s\n", gtViewerInfo.ipAddress);
	}
	else{
		//printf("findPortsByVidPid:: find com port = %s\n", gtViewerInfo.ipAddress);
	}
#endif

	if( argc > 1 ){
		printf("changed User's IP : %s -> %s\n", gtViewerInfo.ipAddress, argv[1]);
		snprintf(gtViewerInfo.ipAddress, 20, "%s", argv[1]);
	}
		
#ifdef __USED_PCL_LIBLARY__
	initPclViewer();
#endif	
	timeThread = thread(timeCheckThread, 0);

	createLogDirectory();

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
	nsl_setCorrection(gtViewerInfo.handle, FUNCTION_OPTIONS::FUNC_ON, FUNCTION_OPTIONS::FUNC_ON, FUNCTION_OPTIONS::FUNC_OFF, FUNCTION_OPTIONS::FUNC_OFF);
	nsl_setBinning(gtViewerInfo.handle, FUNCTION_OPTIONS::FUNC_OFF, FUNCTION_OPTIONS::FUNC_OFF);
	nsl_setHdrMode(gtViewerInfo.handle, HDR_OPTIONS::HDR_NONE_MODE);
	nsl_setAdcOverflowSaturation(gtViewerInfo.handle, FUNCTION_OPTIONS::FUNC_ON, FUNCTION_OPTIONS::FUNC_ON);
	nsl_setDualBeam(gtViewerInfo.handle, DUALBEAM_MOD_OPTIONS::DB_OFF, DUALBEAM_OPERATION_OPTIONS::DB_CORRECTION);
	nsl_setFilter(gtViewerInfo.handle, FUNCTION_OPTIONS::FUNC_OFF, FUNCTION_OPTIONS::FUNC_OFF, 300, 200, 0, 0, FUNCTION_OPTIONS::FUNC_OFF);
	nsl_set3DFilter(gtViewerInfo.handle, 100);
	nsl_setRoi(gtViewerInfo.handle, 0, 0, 319, 239);
	nsl_setAutoIntegrationTime(gtViewerInfo.handle, &gtViewerInfo.autoIntRoi, gtViewerInfo.autoIntRoiEnable);
	nsl_saveConfiguration(gtViewerInfo.handle);
	nsl_getCurrentConfig(gtViewerInfo.handle, &gtViewerInfo.nslConfig);
#endif
	nsl_setIntegrationTime(gtViewerInfo.handle, 1000, 300, 0, 100);
	//nsl_setAutoIntegrationTime(gtViewerInfo.handle, &gtViewerInfo.autoIntRoi, gtViewerInfo.autoIntRoiEnable);
	nsl_setFilter(gtViewerInfo.handle, FUNCTION_OPTIONS::FUNC_ON, FUNCTION_OPTIONS::FUNC_ON, 300, 200, 0, 0, FUNCTION_OPTIONS::FUNC_OFF);
	nsl_set3DFilter(gtViewerInfo.handle, 100);
	nsl_setColorRange(MAX_DISTANCE_12MHZ, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_ON);
	nsl_getCurrentConfig(gtViewerInfo.handle, &gtViewerInfo.nslConfig);
	printConfiguration();	

	if( !gtViewerInfo.streamingMode )
		nsl_requestSingleFrame(gtViewerInfo.handle, gtViewerInfo.operationMode);
	else
		nsl_streamingOn(gtViewerInfo.handle, gtViewerInfo.operationMode);

	while( gtViewerInfo.mainRunning != 0 )
	{
		if( CaptureData() ){
			if( !gtViewerInfo.streamingMode )
				nsl_requestSingleFrame(gtViewerInfo.handle, gtViewerInfo.operationMode);
			processPointCloud(gtViewerInfo.latestFrame.get());
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
