
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
 * @brief openCV draw function
 * 
 * @param distMat : Input distance image used for drawing cursor and overlay information.
 * @param finalRgbBuffer : Output image buffer containing the original RGB image and appended distance information area
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
	string imu_caption = "Not Support IMU";

	if( gtViewerInfo.bImuData ){
		imu_caption = format("Roll : %.6f, Pitch : %.6f", gtViewerInfo.roll, gtViewerInfo.pitch);
		drawCube(distMat, gtViewerInfo.roll, gtViewerInfo.pitch);
	}
 
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

		putText(infoImage, info_caption.c_str(), Point(10, 25), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, int_caption.c_str(), Point(10, 50), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);		
		putText(infoImage, dist2D_caption.c_str(), Point(10, 75), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, dist3D_caption.c_str(), Point(10, 100), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, imu_caption.c_str(), Point(10, 125), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);
		vconcat(distMat, infoImage, finalBuffer);
	}
	else{
		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		string info_caption = format("%s:%dx%d <%dfps> %.2f'C", getDataTypeName(ptNslPCD->operationMode), width, height, gtViewerInfo.drawframeCount, gtViewerInfo.temperature);
		putText(infoImage, info_caption.c_str(), Point(10, 25), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);		
		putText(infoImage, int_caption.c_str(), Point(10, 50), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);			

		vconcat(distMat, infoImage, finalBuffer);
	}

	return;
}

/**
 * @brief Draws RGB cursor position and distance information on the image.
 * 
 * @param rgbMat : Input RGB image used for drawing cursor and overlay information.
 * @param finalRgbBuffer : Output image buffer containing the original RGB image and appended distance information area
 * @param ptNslPCD : Pointer to distance/point cloud data received from nsl_getPointCloudRgbData().
 *
 * @return void
 */
void addRgbInfo(Mat &rgbMat, Mat &finalRgbBuffer, NslPCD *ptNslPCD)
{
	int rgb_xpos = gtViewerInfo.mouseRgbX;
	int rgb_ypos = gtViewerInfo.mouseRgbY;
	float textSize = 2.0f;
//	int xMin = ptNslPCD->roiXMin;

	if( rgb_xpos > -1 && rgb_ypos > -1 && rgb_xpos < NSL_RGB_IMAGE_WIDTH &&  rgb_ypos < NSL_RGB_IMAGE_HEIGHT ){
		Point3D dist = nsl_getDepthAtPixel(gtViewerInfo.handle, rgb_xpos, rgb_ypos, ptNslPCD);
		//printf("dist = %.2f, rgb_xpos = %d, rgb_ypos = %d\n", dist,rgb_xpos, rgb_ypos);

		line(rgbMat, Point(rgb_xpos-23, rgb_ypos), Point(rgb_xpos+23, rgb_ypos), Scalar(255, 255, 0), 4);
		line(rgbMat, Point(rgb_xpos, rgb_ypos-25), Point(rgb_xpos, rgb_ypos+25), Scalar(255, 255, 0), 4);

		string info_caption;
		Mat infoImage(DISTANCE_INFO_HEIGHT, rgbMat.cols, CV_8UC3, Scalar(255, 255, 255));

		if( dist.z < 0 ) // The condition must be based solely on the z value.
			info_caption = format("X:%d,Y:%d (X/Y/Z None)", rgb_xpos, rgb_ypos);
		else
			info_caption = format("X:%d,Y:%d, %.1fmm/%.1fmm/%.1fmm", rgb_xpos, rgb_ypos, dist.x, dist.y, dist.z);

		putText(infoImage, info_caption.c_str(), Point(10, 55), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 3, cv::LINE_AA);
		vconcat(rgbMat, infoImage, finalRgbBuffer);
	}
	else{
		string info_caption;
		Mat infoImage(DISTANCE_INFO_HEIGHT, rgbMat.cols, CV_8UC3, Scalar(255, 255, 255));

		info_caption = format("X:0,Y:0, X/Y/Z None", rgb_xpos, rgb_ypos);
		putText(infoImage, info_caption.c_str(), Point(10, 55), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 3, cv::LINE_AA);
		vconcat(rgbMat, infoImage, finalRgbBuffer);
	}

	return;
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
	gtViewerInfo.bImuData = false;

	if( ptNslPCD->includeImu ){
		gtViewerInfo.bImuData = true;
		gtViewerInfo.roll = ptNslPCD->imuData.roll;
		gtViewerInfo.pitch = ptNslPCD->imuData.pitch;
	}	

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
				if(gtViewerInfo.drawView & DRAW_OPENCV_VIEW){
					setMatrixColor(imageDistance, x+xMin, y+yMin, nsl_getDistanceColor(ptNslPCD->distance2D[y+yMin][x+xMin]));
					setMatrixColor(imageAmplitude, x+xMin, y+yMin, nsl_getAmplitudeColor(ptNslPCD->amplitude[y+yMin][x+xMin]));
				}
				
#ifdef __USED_PCL_LIBLARY__
				if(gtViewerInfo.drawView & DRAW_POINT_CLOUD){
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
								//NslVec3b color3D = nsl_getDistanceColor(ptNslPCD->distance3D[OUT_Z][y+yMin][x+xMin]);
								NslVec3b color3D = ptNslPCD->includeRgb && ptNslPCD->includeYml ? nsl_getPixelAtDepth(handle, x+xMin, y+yMin, gtViewerInfo.rgb.data()) : nsl_getDistanceColor(ptNslPCD->distance3D[OUT_Z][y+yMin][x+xMin]);
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
//							NslVec3b color3D = nsl_getDistanceColor(ptNslPCD->distance3D[OUT_Z][y+yMin][x+xMin]);
							NslVec3b color3D = ptNslPCD->includeRgb && ptNslPCD->includeYml ? nsl_getPixelAtDepth(handle, x+xMin, y+yMin, gtViewerInfo.rgb.data()) : nsl_getDistanceColor(ptNslPCD->distance3D[OUT_Z][y+yMin][x+xMin]);

							point.b = color3D.b;
							point.g = color3D.g;
							point.r = color3D.r;
						}
						
						addPoint(point);
					}
				}
#endif
			}
		}
#ifdef __USED_PCL_LIBLARY__
		drawPointCloud();
#endif

		if(gtViewerInfo.drawView & DRAW_OPENCV_VIEW){
			if( includeDistance ){
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
					static Mat finalRgbBuffer;

					addRgbInfo(imageRgb, finalRgbBuffer, ptNslPCD);
	
					sprintf(distanceViewName,"RGB <%d>", handle);

					namedWindow(distanceViewName, cv::WINDOW_NORMAL);
					resizeWindow(distanceViewName, 640, 370);
					imshow(distanceViewName, finalRgbBuffer);
					setMouseCallback(distanceViewName, mouseCallbackRgbCV);
				}
			}
			else if( includeGrayscale ){
				char distanceViewName[100];
				sprintf(distanceViewName,"Grayscale 2D <%d>", handle);
	
				namedWindow(distanceViewName, WINDOW_NORMAL);
				resizeWindow(distanceViewName, distanceWidth, distanceHeight);
				imshow(distanceViewName, imageAmplitude);
				setMouseCallback(distanceViewName, mouseCallbackCV);
			}
		}			
	}
	else if( includeRgb && (gtViewerInfo.drawView & DRAW_OPENCV_VIEW) ) 
	{
		char distanceViewName[100];
		static Mat rgbBuffer;
		
		addRgbInfo(imageRgb, rgbBuffer, ptNslPCD);
		
		sprintf(distanceViewName,"RGB <%d>", handle);
		
		namedWindow(distanceViewName, cv::WINDOW_NORMAL);
		resizeWindow(distanceViewName, 640, 370);
		imshow(distanceViewName, rgbBuffer);
		setMouseCallback(distanceViewName, mouseCallbackRgbCV);
		
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
	// 

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
	//nsl_setIntegrationTime(gtViewerInfo.handle, 700, 300, 0, 100);
	//nsl_setAutoIntegrationTime(gtViewerInfo.handle, &gtViewerInfo.autoIntRoi, gtViewerInfo.autoIntRoiEnable);
	//nsl_setFrameRate(gtViewerInfo.handle, FRAME_RATE_OPTIONS::FRAME_20FPS);
	//gtViewerInfo.streamingMode = false;
	//gtViewerInfo.operationMode = OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE;
	//gtViewerInfo.rgb.resize(NSL_RGB_IMAGE_WIDTH * NSL_RGB_IMAGE_HEIGHT); 

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
			processPointCloud(gtViewerInfo.latestFrame.get());	// multi frame viewer
			gtViewerInfo.oneSecond = 0;
		}
		else if( !gtViewerInfo.streamingMode && gtViewerInfo.oneSecond > 1 ) {
			processPointCloud(gtViewerInfo.latestFrame.get()); // update one frame viewer
			gtViewerInfo.oneSecond = 1; 
		}

		int key = waitKey(10);
		if( key == 27 ){ // ESC
			break;
		}
		else if( key == 'd' ){
			gtViewerInfo.drawView ^= DRAW_OPENCV_VIEW;
			if( gtViewerInfo.drawView & DRAW_OPENCV_VIEW ) printf("Opencv DrawView On\n");
			else printf("Opencv DrawView Off\n");
		}
		
		//usleep(100);
	}

	gtViewerInfo.mainRunning = 0;
	nsl_streamingOff(gtViewerInfo.handle); // off
	nsl_close();

	if (timeThread.joinable())	timeThread.join();
	
	cv::destroyAllWindows();
	printf("end sample main\n");
	
	return 0;
}

