#include <DirectionCalculus.hpp>


DirectionCalculus(ros::NodeHandle &nh)
    {
        nodehandle_ = nh;
        image_transport::ImageTransport it(nodehandle_);
        image_transport::Subscriber sub = it.subscribe("topic's name", 1, disparityCallback);
        esta=false;
    }

// Destructor
~DirectionCalculus(){}

// Init
void DirectionCalculus::init(){}

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
        imatge = cv_disp_ptr->image;
    }

double position_calculus()
    {
        //Get the image size
        int rows = image.rows;
        int cols = image.cols;
        cv::Size s = image.size();
        rows = s.height;
        cols = s.width;
        cv::namedWindow("Extracted Frame");

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
            calcOpticalFlowPyrLK(translation_prev,translation,OldFeatures,NewFeatures,FeaturesDetected,err,winSize,3, termcrit, 0);
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
                points[1][k++] = points[1][i];
                cv::circle( translation, NewFeatures[i], 3, cv::Scalar(0,255,0), -1, 8);
            }
            NewFeatures.resize(k);
        }
        if(NewFeatures.size()<=10)
            firstframe=true;
        std::swap(points[1], points[0]);
        cv::swap(gray_prev, gray);


        cv::imshow("Extracted Frame",translation);
        final_position=0.0015;
        return final_position;

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

        final_angle=final_angle/number_of_elements;
        NewFeatures = OldFeatures;
        translation_prev = translation;
    }



