#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <eigen3/Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>

#include "parameters.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/PinholeCamera.h"

camodocal::CameraPtr camera;

ros::Publisher img_pub;
ros::Publisher pcd_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

// callback to get rgb image and depth image to create the pcd
void imgs_callback(const sensor_msgs::ImageConstPtr &depth_img_msg) {
    ROS_WARN("1");

    cv_bridge::CvImageConstPtr depth_img_ptr, rgb_img_ptr;

    depth_img_ptr = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::MONO16);
    cv::Mat depth_img = depth_img_ptr->image;

    rgb_img_ptr = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat rgb_img = rgb_img_ptr->image;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZRGB>);


    for (int i = 1; i <= depth_img.rows; i += pcd_stride) {
        ROS_INFO("%d out of %d", i, depth_img.rows);
        for (int j = 1; j <= depth_img.cols; j += pcd_stride) {
            ROS_INFO("%d out of %d || inner: %d out of %d", i, depth_img.rows, j, depth_img.cols);
            const Eigen::Vector2d a(j, i);
            Eigen::Vector3d b;
            camera->liftSphere(a, b);

            uint16_t depth = depth_img.at<uint16_t>(i - 1, j - 1);
            if ((depth == 0) or (depth >= UINT16_MAX)) {
                continue;
            }


            // converting it to meters
            float depth_f = depth * .001f;
            b *= depth_f;
            pcl::PointXYZRGB p;
            p.x = b.x();
            p.y = b.y();
            p.z = b.z();
            p.r = rgb_img.at<cv::Vec3b>(i - 1, j - 1)[2];
            p.g = rgb_img.at<cv::Vec3b>(i - 1, j - 1)[1];
            p.b = rgb_img.at<cv::Vec3b>(i - 1, j - 1)[0];
//            p.g = rgb_img.at<uint8_t>(i - 1, j - 1, 1);
//            p.b = rgb_img.at<uint8_t>(i - 1, j - 1, 0);
            pcd->points.push_back(p);
        }
    }


    pcd->header.frame_id = "world";
    
    pcd_pub.publish(pcd);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bw_pcd_publisher");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    std::string node_namespace = ros::this_node::getNamespace();
    readParameters(nh);

    // std::cerr << node_namespace + DEPTH_IMAGE_TOPIC << std::endl;
    // std::cerr << RGB_IMAGE_TOPIC << std::endl;

    ROS_INFO("config_file path %s", config_file.c_str());

    ROS_INFO("BW NODE");   

    // create camera
    camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file);

    // message_filters::Subscriber<sensor_msgs::Image> depth_img_sub(nh, node_namespace + DEPTH_IMAGE_TOPIC, 1),
    //                                                 rgb_image_sub(nh, node_namespace + RGB_IMAGE_TOPIC, 1);

    // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_img_sub, rgb_image_sub);
    // sync.registerCallback(boost::bind(&imgs_callback, _1, _2));



    ros::Subscriber sub = nh.subscribe("/d1" + DEPTH_IMAGE_TOPIC, 10, imgs_callback);
    pcd_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/newPcd", 1);
    // img_pub = nh.advertise<sensor_msgs::Image>(node_namespace + "/newImage", 1);

    ros::spin();

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    ROS_INFO("Image received");
    img_pub.publish(msg);
    ROS_INFO("Image published");

    cv_bridge::CvImageConstPtr depth_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
    cv::Mat depth_img = depth_img_ptr->image;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZ>);

    ROS_INFO("1");

    for(int i = 1; i < depth_img.rows; i += pcd_stride) {
        for(int j = 1; j <= depth_img.cols; j += pcd_stride) {
            const Eigen::Vector2d a(j, i);
            Eigen::Vector3d b;

            camera->liftSphere(a, b);

            uint16_t depth = depth_img.at<uint16_t>(i - 1, j - 1);
            if((depth == 0) or (depth >= UINT16_MAX)) {
                continue;
            }
        
            float depth_f = depth * .01f;
            b *= depth_f;
            pcl::PointXYZ p;
            p.x = b.x();
            p.y = b.y();
            p.z = b.z();

            pcd->points.push_back(p);
        }
    }

    ROS_INFO("2");

    pcd->header.frame_id = TF_PREFIX + "camera";
    pcd_pub.publish(pcd);
    ROS_INFO("PCD published");
}