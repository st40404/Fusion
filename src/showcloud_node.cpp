#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Core>

#include "opencv2/opencv.hpp"
#include "opencv/cxeigen.hpp"

#include "../camera_models/include/EquidistantCamera.h"
#include "../camera_models/include/PinholeCamera.h"


// add pcl publish header
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// add orb feature
#include "../include/ORBextractor.h"



cv::Mat cv_T_;
CameraPtr cameraptr_;



class PubPointClouds
{
    public:
        PubPointClouds() {};


        // define scan topic
        ros::Publisher Scan_pub;
        ros::Publisher KeyFrame_pub;

        void TranScanToPoints(const sensor_msgs::LaserScanConstPtr& msgSCAN, std::vector<Eigen::Vector3d>& Points);
        void showScanCallback(const sensor_msgs::LaserScanConstPtr& msgSCAN, const sensor_msgs::ImageConstPtr& msgRGB);
        void PubScan(const sensor_msgs::LaserScanConstPtr& msgSCAN);
        void PubORB();

        void PubAllPointClouds(const sensor_msgs::LaserScanConstPtr &msgSCAN, const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);
        // void PubAllPointClouds(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

        ORB::ORBExtrackor extrackor; 

        tf::TransformListener tfListener_;


};


void PubPointClouds::PubAllPointClouds(const sensor_msgs::LaserScanConstPtr &msgSCAN, 
                                       const sensor_msgs::ImageConstPtr &msgRGB, 
                                       const sensor_msgs::ImageConstPtr &msgD) {
// void PubPointClouds::PubAllPointClouds(const sensor_msgs::ImageConstPtr &msgRGB, 
//                                        const sensor_msgs::ImageConstPtr &msgD) {

    // sent RGB and Depth msg to get ORB point cloud
    extrackor.GrabRGB(msgRGB, msgD);

    // PubScan(msgSCAN);
    showScanCallback(msgSCAN, msgRGB);
    // PubORB();
}

// for testing publish scan feature
void PubPointClouds::PubScan(const sensor_msgs::LaserScanConstPtr& msgSCAN)
{
    sensor_msgs::PointCloud2 cloud;
    laser_geometry::LaserProjection projector_;
    // PointCloud cloud;
    // projector_.projectLaser(*msgSCAN, cloud);
    projector_.transformLaserScanToPointCloud("map", *msgSCAN, cloud, tfListener_);
    Scan_pub.publish(cloud);
}

// for testing publish orb feature
void PubPointClouds::PubORB()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.header.frame_id = "camera_aligned_depth_to_color_frame";
    cloud.points.resize (extrackor.mvKeys.size());
    for (size_t i=0; i<extrackor.mvKeys.size(); i++)
    {
        cloud.points[i].x = extrackor.mvKeys[i].pt.x / 100.0;
        cloud.points[i].y = extrackor.mvKeys[i].pt.y / 100.0;
        cloud.points[i].z = extrackor.mvDepth[i];
    }
    KeyFrame_pub.publish(cloud);
}

void PubPointClouds::showScanCallback(const sensor_msgs::LaserScanConstPtr& msgSCAN, 
                                      const sensor_msgs::ImageConstPtr& msgRGB) 
{
    std::vector<Eigen::Vector3d> p_l;
    // TranScanToPoints(msgSCAN, p_l);

    Eigen::Matrix4d Tlc;
    Tlc.setZero();

    if(!cv_T_.empty())
        cv::cv2eigen(cv_T_, Tlc);
    else{
        std::cerr <<" You Do not have calibra result Tlc. We use a Identity matrix." << std::endl;
        Tlc.setIdentity();
    }

//     std::cerr << Tlc << std::endl;
//     Eigen::Matrix3d Rlc = Tlc.block(0,0,3,3);
//     Eigen::Vector3d tlc(Tlc(0,3),Tlc(1,3),Tlc(2,3));

//     Eigen::Matrix3d Rcl = Rlc.transpose();
//     Eigen::Vector3d tcl = - Rcl * tlc;

// //        std::cout << " Tlc: \n"<< Tlc<<std::endl;

//     std::vector<Eigen::Vector3d> p_c;
//     std::vector<cv::KeyPoint> keypoints;

//     int n_pts = msgSCAN->ranges.size();
//     for (int i = 0; i < n_pts; i++) {
//         // std::cout << p_l[i].transpose() << std::endl;
//         p_c.emplace_back(Rcl * p_l[i] + tcl); 
//     }

//     std::vector< cv::Point2f > pixel;
//     std::vector< cv::Point3f> pts;

//     for (int i = 0; i < n_pts; i++) {            
//         double X = p_c[i].x();
//         double Y = p_c[i].y();
//         double Z = p_c[i].z();

//         pts.push_back( cv::Point3f(X,Y,Z) );

//     }
//     cv::Mat r = cv::Mat::zeros(3,1,CV_64F);
//     cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
//     cameraptr_->projectPoints(pts, r, t,pixel);
// //        cv::projectPoints(pts, cv::Vec3d::zeros(), cv::Vec3d::zeros(), cvK_, cvD_, pixel);

//     cv::Mat img_src = cv_bridge::toCvShare(msgRGB, "bgr8")->image;

//     for (size_t j = 0; j < pixel.size(); ++j) {
//         cv::circle(img_src, pixel[j],1, cv::Scalar(0,255,0),1);
//     }
//     cv::imshow("show", img_src);
//     cv::waitKey(10);
}


void PubPointClouds::TranScanToPoints(const sensor_msgs::LaserScanConstPtr& msgSCAN, std::vector<Eigen::Vector3d>& Points)
{
    // http://wiki.ros.org/laser_geometry
    size_t n_pts = msgSCAN->ranges.size ();
    Eigen::ArrayXXd ranges (n_pts, 2);
    Eigen::ArrayXXd output (n_pts, 2);
    Eigen::ArrayXXd co_sine_map (n_pts, 2);

//        std::cout << "---------- read scan -----------"<<std::endl;
    for (size_t i = 0; i < n_pts; ++i)
    {
        // std::cout << msgSCAN->ranges[i]<< " "<<msgSCAN->angle_min + (double) i * msgSCAN->angle_increment <<std::endl;
        ranges (i, 0) = (double) msgSCAN->ranges[i];
        ranges (i, 1) = (double) msgSCAN->ranges[i];

        co_sine_map (i, 0) = cos (msgSCAN->angle_min + (double) i * msgSCAN->angle_increment);
        co_sine_map (i, 1) = sin (msgSCAN->angle_min + (double) i * msgSCAN->angle_increment);
    }

    output = ranges * co_sine_map;

    for (size_t i = 0; i < n_pts; ++i) {

        // TODO: range_cutoff threashold
        double range_cutoff = 30.0;
        const float range = msgSCAN->ranges[i];
        if (range < range_cutoff && range >= msgSCAN->range_min)
        {
            Points.push_back(Eigen::Vector3d(output (i, 0), output (i, 1), 0) );
        }
    }

}

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    std::cout << name <<std::endl;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "show_scan");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    PubPointClouds ppc;
    // define /Ron/Scan topic
    ppc.Scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/Ron/Scan", 1);
    ppc.KeyFrame_pub = nh.advertise<PointCloud>("/Ron/KeyFrame", 1);

    // ppc.Scan_pub = nh.advertise<PointCloud>("/Ron/Scan", 1);

    /// ==========================================
    std::string config_file;
    config_file = readParam<std::string>(pnh, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        return 0;
    } else
    {
        std::cout << "READ CONFIG_FILE...\n";
    }


    std::string scan_topic_name;
    std::string depth_topic_name;
    std::string img_topic_name;
    std::string bag_path;
    std::string save_path;

    fsSettings["scan_topic_name"] >> scan_topic_name;
    fsSettings["img_topic_name"] >> img_topic_name;
    fsSettings["depth_topic_name"] >> depth_topic_name;
    fsSettings["savePath"] >> save_path;
    fsSettings["bag_path"] >> bag_path;


    std::string sModelType;
    fsSettings["model_type"] >> sModelType;


    cameraptr_ = nullptr;
    if (sModelType.compare("KANNALA_BRANDT") == 0)
    {
        EquidistantCamera::Parameters paras;
        paras.readFromYamlFile(config_file);
        cameraptr_ = CameraPtr (new EquidistantCamera(paras));
        ROS_INFO("LOAD KANNALA BRANDT CAMERA!");
    }
        // todo:: PIN HOLE MODEL
    else
    {
        PinholeCamera::Parameters paras;
        paras.readFromYamlFile(config_file);
        cameraptr_ = CameraPtr (new PinholeCamera(paras));
    }


    fsSettings.release();

    std::cout <<"\t"<<scan_topic_name << std::endl;
    std::cout <<"\t"<<img_topic_name << std::endl;
    std::cout <<"\t"<<depth_topic_name << std::endl;
    std::cout <<"\t"<<bag_path << std::endl;

    std::string fn = save_path + "result.yaml";
    fsSettings.open(fn,cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "You should run calibra tool first to generate result.yaml in your savePath." << std::endl;
        return 0;
    }
    fsSettings["extrinsicTlc"] >> cv_T_;
    std::cout <<"Load extrinsicTlc:\n"<<cv_T_<<std::endl;
    fsSettings.release();

    std::cout << "END CONFIG_FILE\n" << std::endl;
    /// ==========================================

    ppc.extrackor.SetConfig(config_file);

    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, scan_topic_name, 10);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, img_topic_name, 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic_name, 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), scan_sub, image_sub, depth_sub);
    sync.registerCallback(boost::bind(&PubPointClouds::PubAllPointClouds, &ppc, _1, _2, _3));

    // message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, img_topic_name, 10);
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic_name, 10);

    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    // message_filters::Synchronizer<sync_pol> sync(sync_pol(10), image_sub, depth_sub);
    // sync.registerCallback(boost::bind(&PubPointClouds::PubAllPointClouds, &ppc, _1, _2));
    std::cout << "start spin.." << std::endl;

    ros::spin();

    return 0;
}
