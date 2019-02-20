#include <image_converter/image_converter.hpp>
#include <image_converter/draw.hpp>
     
ImageConverter::ImageConverter() : it_(nh_) {
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    cv::namedWindow(OPENCV_WINDOW);
}

ImageConverter::~ImageConverter(){
    cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImageConstPtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if(cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60){
        draw_cv::Draw::circle(cv_ptr, 50, 50 ,255, 0 , 0);
    }

    //update GUI window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    //output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}      
