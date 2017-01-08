#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/time.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr srcImgPtr;
    try{
        srcImgPtr = cv_bridge::toCvCopy(msg, "bgr8");
        int scaleFactor = 3;
        
            struct timeval tp;
            gettimeofday(&tp, NULL);
            long int begin = tp.tv_usec / 1000;
        
        // put the image upright and scale 50%
        cv::resize(srcImgPtr->image, srcImgPtr->image, cv::Size(), 1.0/scaleFactor, 1.0/scaleFactor, CV_INTER_LINEAR);
        // cv::transpose(srcImgPtr->image, srcImgPtr->image);
        // cv::flip(srcImgPtr->image, srcImgPtr->image, 1);
        
        // blur the image slightly to level the noise and smoothen the edges
        cv::Mat blurred;
        cv::medianBlur(srcImgPtr->image, blurred, 5);
        
        // split the RGB channels
        cv::Mat bgr[3];
        cv::split(blurred, bgr);
        
        // turn every channel into a binary image
        // note that this only works correctly if the line is not too thin
        //     and the line does not take up more than 50% of the screen
        cv::Mat binaryBgr[3];
        for(int i=0; i < 3; i++){
            // get the minimum and maximum of the current picture
            double min, max;
            cv::minMaxIdx(bgr[i], &min, &max, NULL, NULL, cv::noArray());
            double mean = (min + max) / 2;
            // also get the std deviation to check if the current color channel is not empty
            cv::Scalar placeholderMean, std;
            cv::meanStdDev(bgr[i], placeholderMean, std, cv::noArray());
            
            // if the the current channel is not empty
            if (std.val[0] > 10.0){
                // threshold the image to binary on the mean
                cv::threshold(bgr[i], binaryBgr[i], mean/2, 255, 0);
                // if more than 50% of the image is white, invert
                if (cv::countNonZero(binaryBgr[i]) > (1920 * 1080 / scaleFactor / scaleFactor / 2)){
                    cv::bitwise_not(binaryBgr[i], binaryBgr[i]);
                }
            } else{
                binaryBgr[i] = cv::Mat::zeros(1080 / scaleFactor, 1920 / scaleFactor, 0);
            }
        }
        
        // combine the three color channels back into a single matrix
        cv::Mat res;
        cv::bitwise_or(binaryBgr[0], binaryBgr[1], res, cv::noArray());
        cv::bitwise_or(res, binaryBgr[2], res);
        
        /*
            // search for edges in the seperated images
            cv::Mat edges[3];
            for(int i=0; i < 3; i++){
                cv::Canny(bgr[i], edges[i], thresh, thresh*3, 3);
            }
            
            // combine images in single matrix
            cv::Mat res;
            res = edges[0]/3 + edges[1]/3 + edges[2]/3;
        */
        
            gettimeofday(&tp, NULL);
            long int middle = tp.tv_usec / 1000;
        
        cv::imshow("view", res);
        cv::waitKey(30);
        
            gettimeofday(&tp, NULL);
            long int end = tp.tv_usec / 1000;
        
        long int dif1 = middle-begin;
        long int dif2 = end-middle;
        
        ROS_INFO("Time between begin and middle: %lo ms, time between middle and end: %lo ms", dif1, dif2);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv){
    // initialize ros
    ros::init(argc, argv, "linefollower");
    ros::NodeHandle nh;
    
    cv::namedWindow("view");
    cv::startWindowThread();
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);
    
    ros::spin();
    
    cv::destroyWindow("view");

    return 0;
}

