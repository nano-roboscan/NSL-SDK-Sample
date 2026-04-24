
#include "main.h"

#ifdef __USED_PCL_LIBLARY__

void initPclViewer()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->clear();
	cloud->is_dense = true;
	cloud->width = NSL_LIDAR_TYPE_B_WIDTH*NSL_LIDAR_TYPE_B_HEIGHT;
	cloud->height = 1;
	cloud->points.reserve(cloud->width*cloud->height);

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("NSL PCL 3D PointCloud"));
	viewer->getRenderWindow()->SetWindowName("NSL PCL 3D Viewer");
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "Viewer pointCloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Viewer pointCloud");
	viewer->addCoordinateSystem(1.0);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, -2, 0, 0, 0, 0, -1, 0, 0);
	viewer->setShowFPS(false);
	 // keyboard event callback
	viewer->registerKeyboardCallback(onKeyboardEvent, (void*)viewer.get());
//	viewer->setSize(1280, 960);
	viewer->setSize(640, 480);

	vtkSmartPointer<vtkRenderer> renderer = viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer();
	vtkCamera* camera = renderer->GetActiveCamera();
	camera->SetClippingRange(0.01, 50.0);  // 1cm ~ 50m
	viewer->addText("NANOSYSTEMS, PointCloud Sample !!!", 30, 30, "nsl-pcl");

	if( gtViewerInfo.area_enable ){
		viewer->addText("Detection : ", 30, 50, "point-cnt");
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
}


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
	gtViewerInfo.clouds[0]->width = gtViewerInfo.clouds[0]->points.size();
	gtViewerInfo.clouds[0]->height = 1;
	if( !(gtViewerInfo.drawView & DRAW_POINT_CLOUD) ) return;

	pcl::visualization::PCLVisualizer::Ptr viewer = gtViewerInfo.viewers[0];
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr = gtViewerInfo.clouds[0];

	if( gtViewerInfo.area_enable ){
		viewer->updateText("Detection : " + std::to_string(gtViewerInfo.area_inCount), 30, 50, "point-cnt");
	}

	if( !viewer->wasStopped() && point_cloud_ptr->points.size() > 0 ){
//		viewer->removePointCloud("Viewer pointCloud");
//		viewer->addPointCloud(point_cloud_ptr, "Viewer pointCloud");		
		viewer->updatePointCloud(point_cloud_ptr, "Viewer pointCloud");
		viewer->spinOnce(1, true);
	}

//	0 = 1 - 0;
}


void clearCloud()
{
	if(gtViewerInfo.drawView & DRAW_POINT_CLOUD)
		gtViewerInfo.clouds[0]->clear();
}


void addPoint(pcl::PointXYZRGB point)
{
	gtViewerInfo.clouds[0]->points.push_back(point);
}

#endif

