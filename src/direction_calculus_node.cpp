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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace cv;
using namespace std;

bool first_atempt = true;

#define PI 3.14159265

//void disparityCallback(const sensor_msgs::ImageConstPtr& disp)

void callback(const sensor_msgs::ImageConstPtr& original, const sensor_msgs::ImageConstPtr& mascara)
{
    //ROS variables
    //image publisher
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub_freemap;
    pub_freemap = it.advertise("optical_flow_test", 1);

    //OpenCV variables

    static Mat err;
    static Mat translation_prev;
    static Mat translation;
    static Rect region_of_interest;
    static Mat imatge;
    static Mat imatge_original;
    static Mat mascara_imatge;


    //features variables
    static vector<Point2f> NewFeatures;
    //static vector<Point2f> FeaturesDetected;
    static vector<Point2f> OldFeatures;
    std::vector<uchar> status;

    cv_bridge::CvImagePtr cv_disp_ptr;
    try
    {
        cv_disp_ptr = cv_bridge::toCvCopy(original, "mono8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from image to 'mono8'.");
    }
    imatge_original = cv_disp_ptr->image;

    try
    {
        cv_disp_ptr = cv_bridge::toCvCopy(mascara, "mono8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from image to 'mono8'.");
    }
    mascara_imatge = cv_disp_ptr->image;

    //multiply(InputArray src1, InputArray src2, OutputArray dst, double scale=1, int dtype=-1 )
    multiply(imatge_original, mascara_imatge, imatge);

    //Get the image size
    int rows = imatge.rows;
    int cols = imatge.cols;
    cv::Size s = imatge.size();
    rows = s.height;
    cols = s.width;


    //Get the ROI of the botom area (translation movement)
    region_of_interest = Rect (0, 0.7*rows, cols, rows);
    translation = imatge(region_of_interest);

    //Optical Flow Calculus
    goodFeaturesToTrack(translation, NewFeatures, 500, 0.01, 10, Mat(), 3, 0, 0.04);

    if (first_atempt)
    {
        OldFeatures = NewFeatures;
        first_atempt = false;

    }

    else
    {
        calcOpticalFlowPyrLK(translation_prev,translation,OldFeatures,NewFeatures,status,err);
        //Outlieres purge checking the longitude and ignoring the biggest ones
        // for(size_t features_vector_position=0; features_vector_position<NewFeatures.size(); features_vector_position++)
        //{
        //representation of the optical flux
        size_t i, k;
        for( i = k = 0; i < NewFeatures.size(); i++ )
        {
            if( !status[i] )
                continue;
            if ((abs(OldFeatures[i].x-NewFeatures[i].x)+(abs(OldFeatures[i].y-NewFeatures[i].y))<2))
                continue;

            cv::line( translation, OldFeatures[i],NewFeatures[i], cv::Scalar(0,255,0));
            NewFeatures[k++] = NewFeatures[i];
            cv::circle( translation, NewFeatures[i], 3, cv::Scalar(0,255,0), -1, 8);
        }
        NewFeatures.resize(k);
    }
    NewFeatures = OldFeatures;
    translation_prev = translation;

    cv_bridge::CvImage cv_freemap;
    //cv_freemap.header = disp->header;
    cv_freemap.encoding = "mono8";
    cv_freemap.image = translation;
    pub_freemap.publish(cv_freemap.toImageMsg());
}





int main(int argc, char **argv)
{
    /*
    //bucle control variables
    //static int i=0, j=0, a=0;

    //filtering variables
    //static double distance=0, x=0, y=0;
    //static int param_high=30, param_low=3;	//Mayor y menor longitud permitida para descartar outliers



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
    bool esta;


    //Mean variables
    //int number_of_elements=0;

    double final_angle;
    double final_position;*/

    ros::init(argc, argv, "overtaking");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10); // Hz

    //subscriber to 2 different topics
    message_filters::Subscriber<sensor_msgs::ImageConstPtr> original(nh, "stereo_camara/disparity", 1);
    message_filters::Subscriber<sensor_msgs::ImageConstPtr> mascara(nh, "stereo_camera/free_map", 1);
    message_filters::TimeSynchronizer<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr> sync(original, mascara, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    while(ros::ok())
    {
        /*
        if(FeaturesDetected[features_vector_position])
        {
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

    //angle calculus
    for(position_in_angles=0;position_in_angles<max_position;position_in_angles++)
    {
        for(repited_angle_counter=0;repited_angle_counter<size_repited_angle;repited_angle_counter+++)
        {
            if (repited_angle[repited_angle_counter] == angles[position_in_angles])
            {
                number_of_repetitions[repited_angle_counter]++;
                esta=true;

                if(number_of_repetitions[repited_angle_counter]>max_repetition)
                {
                    max_repetition=number_of_repetitions[repited_angle_counter];
                    max_value = repited_angle[repited_angle_counter];

                }
                break;//if there are the same the loop should stop
            }
        }
        if(!esta)
        {
            repited_angle[repited_angle_counter] = angles[position_in_angles];
            size_repited_angle++;
        }
        esta=false;
    }

    //Mean calculation
    for(repited_angle_counter=0;repited_angle_counter<size_repited_angle;repited_angle+++)
    {
        if(number_of_repetitions[repited_angle_counter]==0.8*max_repetition)
        {
            final_angle=repited_angle[repited_angle_counter]*number_of_repetitions[repited_angle_counter];
            number_of_elements=number_of_elements+number_of_repetitions[repited_angle_counter];
        }

    }*/

        // final_angle=final_angle/number_of_elements;

        ros::spinOnce();
        loop_rate.sleep();
    }
}



