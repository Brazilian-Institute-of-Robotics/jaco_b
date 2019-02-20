#include <image_converter/image_converter.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}