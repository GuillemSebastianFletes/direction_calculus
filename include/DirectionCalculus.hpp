#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include "opencv2/video/tracking.hpp"

using namespace cv;
using namespace std;


class DirectionCalculus
{
    private:
    //bucle control variables
    //static int i=0, j=0, a=0;

    //filtering variables
    //static double distance=0, x=0, y=0;
    //static int param_high=30, param_low=3;	//Mayor y menor longitud permitida para descartar outliers

    //OpenCV variables

    static Mat err;
    static Mat translation_prev;
    static Mat translation;
    static Rect region_of_interest;
    static Mat imatge;

    //features variables
    static vector<Point2f> NewFeatures;
    static vector<Point2f> FeaturesDetected;
    static vector<Point2f> OldFeatures;

    //movement calculation's variables
    static double angle;
    double angles[];
    //int position_in_angles=0;
    int max_position;

    //More repetitive angle calculation's variable
    double repited_angle[];
    int number_of_repetitions[];
    int repited_angle_counter;
    int size_repited_angle;
    int max_repetition;
    double max_value;
    bool esta;

    //Mean variables
    //int number_of_elements=0;
   

    public:

    DirectionCalculus(ros::NodeHandle &nh);
    ~DirectionCalculus();

    // Init
    void init();
    void disparityCallback(const sensor_msgs::ImageConstPtr& disp);
    double position_calculus();
    double final_angle;
    double final_position;
