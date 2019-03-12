#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char** argv){
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);
    cv::namedWindow("OPENCV_WINDOW");
    cv::imshow("OPENCV_WINDOW", markerImage);
    cv::waitKey(0);
    return 0;
}
