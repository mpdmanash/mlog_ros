/**
 * Computes disparity and 3D points to be published in various formats.
 * 
 * When the class vlog is initialized, message_filters are used to 
 * approximately synchronize two image topics and two camera info topics based on their time stamp.
 * Synchronization is required in order to process topics of the same time stamp and not by the order
 * they are received. The combined topics callback the "vlog::callback" method.
 * 
 * The following is the sequence of significant methods called:
 * callback -> elas -> getQMatrix -> generate3dPoints -> publisher -> heightMapColor.
 *
 * Two significant structs are defined to hold shared configuration data and easy modification. These
 * configurations have some default value but can be changed by the method initializing the class vlog.
 * The structs are:
 *  - TopicNames: Hold the names of the topics to be subscribed and published.
 *  - ConfigSettings: Holds configuration settings
 */

#ifndef VLOG_H
#define VLOG_H

#include <ros/ros.h>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h> // includes "opencv2/core/core.hpp", "opencv2/imgproc/imgproc.hpp and types_c.h"
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h> // includes "Eigen/Core" and "Eigen/Geometry"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include "ceres/ceres.h"
#include "glog/logging.h"

#include <cmath>
#include <random>
#include <algorithm>

#include "GRANSAC.hpp"
#include "LineModel.hpp"

#include <iostream>
#include <fstream>

#include <omp.h>

// #include <Eigen/Core>
// #include <Eigen/Geometry>

 // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
 //                                                       nav_msgs::Odometry> MySyncPolicy;
struct TopicNames {
    TopicNames() : inBottomImage("/erlecopter/bottom/image_raw"), 
                   inFrontImage("/erlecopter/front/image_front_raw"), 
                   inBottomInfo("/erlecopter/bottom/camera_info"),
                   inFrontInfo("/erlecopter/front/camera_front_info") {}
    std::string inBottomImage;
    std::string inFrontImage;
    std::string inBottomInfo;
    std::string inFrontInfo;
};

struct ConfigSettings {
    ConfigSettings() : outFileName("vlog.txt"), 
                       useFrontImage(false),
                       m(1.0),
                       f(395.1658039969752), //323.451202597659 , 395.1658039969752
                       debug(false) {}
    std::string outFileName;
    bool useFrontImage, debug;
    double m, f;
};

class vlog

{
public:
    vlog(ros::NodeHandle innh, TopicNames intn = TopicNames(), ConfigSettings incs = ConfigSettings());
    ~vlog();
    void callback(const sensor_msgs::ImageConstPtr& left_image);
    void gtcallback(const nav_msgs::OdometryConstPtr& odom);

private:
    TopicNames s_tn;
    ConfigSettings s_cs;
    ros::NodeHandle s_nh;
    // message_filters::Subscriber<sensor_msgs::Image> * image_sub;
    // message_filters::Subscriber<nav_msgs::Odometry> * info_sub;
    // message_filters::Synchronizer<MySyncPolicy> * sync;
    ros::Publisher disparity_image_pub;
    ros::Subscriber imagenew_sub;
    ros::Subscriber gt_sub;
    std_msgs::Header common_header;
    cv::Mat disparity_image;
    unsigned int num_pc_points;
    void publisher();
    void processImage(cv::Mat &display, cv::Mat &canny);
    void filterLines(std::vector<cv::Vec2f> inlines, std::vector<cv::Vec2f> & outlines);
    void optimize(std::vector<cv::Vec2f>& gridLines, int center_rho, double & phi, double & o, double & h, double & out_i, ceres::Solver::Summary & summary);
    double getSpppi(double phi, double o, double m, int i, double h, double f);
    double getCost(double phi, double o, double h, int start_i, std::vector<cv::Vec2f> & rhos, std::vector<int> & is);
    void filter(float ox, float & px, float & pox, float & pvx);
    float square(float input);

    int h1; int s1; int v1;
    int th;
    int lines;
    int hough_rho, hough_theta, hough_threshold;
    bool first_run, gtfr;
    float m_px, m_pox, m_py, m_poy, m_pvx, m_pvy, m_ofx, m_ofy;
    nav_msgs::Odometry m_odom;

    std::ofstream myfile;
};

#endif