#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


class ImageConverter{
    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;

    public: 
        const std::string OPENCV_WINDOW;

        ImageConverter();
        ~ImageConverter();     
        void imageCb(const sensor_msgs::ImageConstPtr& msg);
        

};