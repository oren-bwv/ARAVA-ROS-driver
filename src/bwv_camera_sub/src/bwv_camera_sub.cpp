#include <ros/ros.h>                            // ROS_INFO
#include <image_transport/image_transport.h>    // image_transport::ImageTransport
#include <opencv2/highgui/highgui.hpp>          // opencv
#include <opencv2/core.hpp>                     // opencv
#include <opencv2/imgproc.hpp>                  // opencv
#include <cv_bridge/cv_bridge.h>                // cv_bridge
#include <sys/timeb.h>                          // struct timeb


unsigned long long get_timestamp()
{
	struct timeb t;
	ftime(&t);
	return (unsigned long long)t.time*1000ULL+(unsigned long long)t.millitm;
}

static uint64_t tss = get_timestamp();

void ImageCallbackMaster(const sensor_msgs::ImageConstPtr& img)
{
    cv_bridge::CvImagePtr cv_ptr;

    if (img->encoding == "mono8")
    {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_GRAY2BGR);
        cv::imshow("8-bit Mono", cv_ptr->image);
        cv::waitKey(1);
    }
    else // encoding == mono16 - for 10 bit video
    {
        cv::Mat img8(482, 800, CV_8UC1);
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
        cv_ptr->image.convertTo(img8, CV_8UC1, 1/4.0);
        cv::cvtColor(img8, img8, CV_GRAY2BGR);
        cv::imshow("10-bit Mono Master", img8);
        cv::waitKey(1);
    }

    static uint64_t tsm = get_timestamp();
    static uint32_t masterCounter = 0;
    ++masterCounter;
    if ((get_timestamp() - tsm) > 1000) {
        printf("Master FPS: %d\n", masterCounter);
        tsm = get_timestamp();
        masterCounter = 0;
    }
}

void ImageCallbackSlave(const sensor_msgs::ImageConstPtr& img)
{
    cv_bridge::CvImagePtr cv_ptr;

    if (img->encoding == "mono8")
    {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_GRAY2BGR);
        cv::imshow("8-bit Mono", cv_ptr->image);
        cv::waitKey(1);
    }
    else // encoding == mono16 - for 10 bit video
    {
        cv::Mat img8(482, 800, CV_8UC1);
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
        cv_ptr->image.convertTo(img8, CV_8UC1, 1/4.0);
        cv::cvtColor(img8, img8, CV_GRAY2BGR);
        cv::imshow("10-bit Mono Slave", img8);
        cv::waitKey(1);
    }

    static uint64_t tss = get_timestamp();
    static uint32_t slaveCounter = 0;
    ++slaveCounter;
    if ((get_timestamp() - tss) > 1000) {
        printf("\tSlave FPS: %d\n", slaveCounter);
        tss = get_timestamp();
        slaveCounter = 0;
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bwv_camera_sub");
    ros::NodeHandle nh;
	ROS_INFO("bwv_camera_sub init");

    ros::Subscriber subMater = nh.subscribe("/bwv_camera_driver/bwv_camera/bwv_camera/image_master",
        1000, ImageCallbackMaster);

    ros::Subscriber subSlave = nh.subscribe("/bwv_camera_driver/bwv_camera/bwv_camera/image_slave",
        1000, ImageCallbackSlave);

    ros::spin();

    return (0);
}
