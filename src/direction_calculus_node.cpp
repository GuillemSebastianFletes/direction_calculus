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


/*cv::Mat grayOld;
geometry_msgs::Point point;*/


void position_calculus()
{
    //buvle control variables
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
            //Eliminamos outliers:

            //Comprobando sus longitudes y descartando las mayores
            x=(NewFeatures[features_vector_position].x)-(OldFeatures[features_vector_position].x);
            y=(NewFeatures[features_vector_position].y)-(OldFeatures[features_vector_position].y);
            distance=sqrt((x*x)+(y*y));
            //cout << distance << endl;	//esta linea es solo para comprobar las distancias, no sirve
            if((distance<param_high)&&(distance>param_low))
            {
                //Agrupando vectores y descartando los alejados

                //Comprobamos el angulo de cada vector
                static double angle = (atan2 (y,x) * 180.0 / PI)*(-1);

                //Dibujamos las lineas que conectan las posiciones antiguas y nuevas
                if((angle<90)&&(angle>0))	//Adelantamiento
                {
                    line(image,OldFeatures[features_vector_position],NewFeatures[features_vector_position],Scalar(0,0,255));

                    /*Eliminamos las detecciones de adelantamiento erroneas
featuresDetection[j]=OldFeatures[features_vector_position];
*/
                    j++;
                }
                else	//No adelantamiento
                {
                    line(image,OldFeatures[features_vector_position],NewFeatures[features_vector_position],Scalar(0,255,0));
                }
            }
        }
    }

    /*Comprobamos que la deteccion haya sido correcta
if(j>50)
{
cout<<endl<<"Se esta produciendo un adelantamiento"<<endl<<a<<endl;
}
else
{
cout<<endl<<" "<<endl<<a<<endl;
}*/
    a++;

    point.x = 12;
    point.y = 56;

    //cv::imshow("Window", image);
    cv::imshow("Window2", image_roi);
    cv::waitKey(3);

    NewFeatures = OldFeatures;
    translation_prev = translation
}

i++;
image_roi.copyTo(grayOld);

//Deteccion de bordes
goodFeaturesToTrack(grayOld, OldFeatures, 500, 0.01, 10, Mat(), 3, 0, 0.04);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "overtaking");
    ros::NodeHandle nh;
    ros::Publisher overtaking_pub = nh.advertise<geometry_msgs::Point>("overtaking_xy",1);

    ros::Rate loop_rate(10); // Hz

    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub = it.subscribe("xb3_no_rect/left", 1, imageCallback); //usb_cam/image_rawxb3_no_rect/left
    while(ros::ok()){
        ros::spinOnce();
        if(new_overtaking){
            overtaking_pub.publish(point);
            new_overtaking = 0;
        }
        loop_rate.sleep();
    }

}
