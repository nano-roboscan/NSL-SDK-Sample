#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <boost/thread.hpp>
#include <boost/signals2.hpp>

#include "nanolib.h"

#define image_transfer_function

#ifdef image_transfer_function
#include <image_transport/image_transport.hpp>
#endif

namespace nanosys {

	struct ViewerParameter {
		int	 frameCount;
		
		bool cvShow;
		bool changedCvShow;
		bool changedIpInfo;
		bool reOpenLidar;

		std::string	ipAddr;
		std::string	netMask;
		std::string	gwAddr;
    };

	class roboscanPublisher : public rclcpp::Node { 

	public:
		roboscanPublisher();
		~roboscanPublisher();

		void publisher_callback();
		void reConfigure();
		void publishFrame(NslPCD *frame);
		void startStreaming();

		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgDistancePub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgAmplPub;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudPub;

#ifdef image_transfer_function
		rclcpp::Node::SharedPtr nodeHandle;
		image_transport::ImageTransport imageTransport;
		image_transport::Publisher imagePublisher;
#endif

		boost::scoped_ptr<boost::thread> publisherThread;
		bool runThread;

	    struct ViewerParameter 	viewerParam;
		NslConfig 		nslConfig;
		int 			nsl_handle;
	private:
		void initialise();
		void getMouseEvent( int &mouse_xpos, int &mouse_ypos );
		cv::Mat addDistanceInfo(cv::Mat distMat, NslPCD *frame);
		void setWinName();
		void timeDelay(int milli);

		
		OnSetParametersCallbackHandle::SharedPtr callback_handle_;
		rcl_interfaces::msg::SetParametersResult parametersCallback( const std::vector<rclcpp::Parameter> &parameters);
		int mouseXpos, mouseYpos;
		bool isReconfigure;
		char winName[100];
	};


} //end namespace nanosys
