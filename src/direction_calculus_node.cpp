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

#define PI 3.14159265

static Mat image;//, translation, rotation;
static Mat translation;
static vector<Point2f> OldFeatures;


void disparityCallback(const ImageConstPtr& disp)
{
    cv_bridge::CvImagePtr cv_disp_ptr;
    try
    {
        cv_disp_ptr = cv_bridge::toCvCopy(disp, "mono8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from image to 'mono8'.");
    }
    image = cv_disp_ptr->image;
}


void position_calculus()
{
    //bucle control variables
    static int i=0, j=0, a=0;

    //filtering variables
    static double distance=0, x=0, y=0;
    static int param_high=30, param_low=3;	//Mayor y menor longitud permitida para descartar outliers

    //OpenCV variables
    static Mat err;
    static Mat translation_prev;
    static Mat translation;
    static Rect region_of_interest;

    //features variables
    //static vector<Point2f> OldFeatures;
    static vector<Point2f> NewFeatures;
    static vector<Point2f> FeaturesDetected;

    //movement calculation's variables
    static double angle;
    double angles[];
    int position_in_angles=0;
    int max_position;

    //More repetitive angle calculation's variable
    double repited_angle[];
    int number_of_repetitions[];
    int repited_angle_counter;
    int size_repited_angle;
    int max_repetition;
    double max_value;

    //Obtenemos el tamaño de la imagen
    int rows = image.rows;
    int cols = image.cols;
    cv::Size s = image.size();
    rows = s.height;
    cols = s.width;

    //Ajustamos un ROI según el tamaño de la imagen
    region_of_interest = Rect (0, 0, 0.3*cols, 0.5*rows);
    translation = image(region_of_interest);

    //Optical Flow Calculus
    //static vector<uchar> FeaturesDetected;
    goodFeaturesToTrack(translation, NewFeatures, 500, 0.01, 10, Mat(), 3, 0, 0.04);
    calcOpticalFlowPyrLK(translation_prev,translation,OldFeatures,NewFeatures,FeaturesDetected,err);

    for(size_t features_vector_position=0; features_vector_position<NewFeatures.size(); features_vector_position++)
    {
        if(FeaturesDetected[features_vector_position])
        {
            //Outlieres purge checking the longitude and ignoring the biggest ones
            x=(NewFeatures[features_vector_position].x)-(OldFeatures[features_vector_position].x);
            y=(NewFeatures[features_vector_position].y)-(OldFeatures[features_vector_position].y);
            distance=sqrt((x*x)+(y*y));
            //cout << distance << endl;	//this line is only useful if the user wans to check the distances
            if((distance<param_high)&&(distance>param_low))
            {
                //Checking each array's angle
                angle = (atan2 (y,x) * 180.0 / PI)*(-1);
                angles[position_in_angles]=angle;
                position_in_angles++;
            }
        }
    }
    max_position=position_in_angles;
    for(position_in_angles=0;position_in_angles<max_position;position_in_angles++)
    {
        for(repited_angle_counter=0;repited_angle_counter<size_repited_angle;repited_angle+++)
        {
            if (repited_angle[repited_angle_counter] != angles[position_in_angles])
            {
                repited_angle[repited_angle_counter] = angles[position_in_angles];
                number_of_repetitions[repited_angle_counter]++;
                if(number_of_repetitions[repited_angle_counter]>max_repetition)
                {
                    max_repetition=number_of_repetitions[repited_angle_counter];
                    max_value = repited_angle[repited_angle_counter];
                }
            }
        }

    }


    NewFeatures = OldFeatures;
    translation_prev = translation;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "overtaking");
    ros::NodeHandle nh;
    ros::Publisher overtaking_pub = nh.advertise<geometry_msgs::Point>("overtaking_xy",1);

    ros::Rate loop_rate(10); // Hz

    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub = it.subscribe("xb3_no_rect/left", 1, imageCallback); //usb_cam/image_rawxb3_no_rect/left
    while(ros::ok())
    {
        ros::spinOnce();
        if(new_overtaking)
        {
            overtaking_pub.publish(point);
            new_overtaking = 0;
        }

        position_calculus();//this function is going to calculate the direction

        loop_rate.sleep();
    }

}
