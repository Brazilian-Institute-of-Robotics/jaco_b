#include <image_converter/draw.hpp>

void draw_cv::Draw::circle (cv_bridge::CvImageConstPtr cv_ptr, int x, int y, int r, int g, int b){
    cv::circle(cv_ptr->image, cv::Point(x, y), 10, CV_RGB(r,g,b));


}  


