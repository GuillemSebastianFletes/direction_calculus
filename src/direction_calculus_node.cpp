#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <ros/ros.h>

static const double pi = 3.14159265358979323846;

using namespace cv;
using namespace std;

static Mat image, translation, rotation;


void disparityCallback(const ImageConstPtr& disp)
{
    vector<Point2f> points[2];
    bool needToInit = false;

    cv_bridge::CvImagePtr cv_disp_ptr;
    try{
        cv_disp_ptr = cv_bridge::toCvCopy(disp, "mono8");
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from image to 'mono8'.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "direction_calculus");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("stereo_camera/free_map", 1, disparityCallback);

    int rows = image.rows;
    int cols = image.cols;
    bool first_atempt = true;
    vector<Point2f> old_interest_points, interest_points;
    char* status;
    float* track_error;
    int counter1, counter;
    int x,y;

    while (ros::ok())
    {
        ros::spin();
        cv::Mat tmp = cv::Mat(frame,false);
        image= tmp.clone();
        translation = image(0, rows-100, cols, 100, 0, 0, cols, 130); // Region Of Interest

        if (first_atempt)
        {
             translation_prev=translation;
             first_atempt = false;
        }

        else if (first_atempt == false)
        {
             goodFeaturesToTrack(translation_prev, old_interest_points, 500, 0.01, 10, Mat(), 3, 0, 0.04);
             goodFeaturesToTrack(translation, interest_points, 500, 0.01, 10, Mat(), 3, 0, 0.04);
             calcOpticalFlowPyrLK(translation_prev, translation, old_interest_points, interest_points, status, track_error);







             translation_prev=translation;
        }

    }



return 0;
}


