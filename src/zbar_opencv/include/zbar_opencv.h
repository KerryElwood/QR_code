#ifndef ZBAR_OPENCV_H
#define ZBAR_OPENCV_H

#include <ros/ros.h>
#include <zbar.h>
#include <iostream>
#include <iomanip>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cmath>


using namespace cv;
using namespace Eigen;


class ImageConverter
{
    public:
    int state;
    int count;
    double fx,fy;
    double L;
    double length;
    //Camera parameter
    MatrixXd K;
    Matrix3d Kn;
    Vector3d C1;
    Vector3d C2;
    Matrix3d R1;
    Matrix3d R2;
    Vector3d P3D;
    Vector3d P3DN;
    Vector2d pic0pnts;
    Vector2d pic1pnts;
    Vector3d theta_set;
    Vector3d campnt;
    
    
    ImageConverter() : state(0),K(MatrixXd::Zero(3,4)),Kn(Matrix3d::Zero()),C1(Vector3d::Zero()),C2(Vector3d::Zero()),
                       R1(Matrix3d::Identity()),R2(Matrix3d::Identity()),P3D(Vector3d::Zero()),P3DN(Vector3d::Zero()),theta_set(Vector3d::Zero()),
                       pic0pnts(Vector2d::Zero()),pic1pnts(Vector2d::Zero()),count(0),fx(0),fy(0),L(0.183),length(0),campnt(Vector3d::Zero())
    {
    }
    ~ImageConverter(){}

    Vector3d triangulationPoints();
    Vector3d simtriangle();
};

void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);

void image_callback(const sensor_msgs::ImageConstPtr& msg);

void zbarscanner(cv_bridge::CvImagePtr cv_ptr);

Matrix3d Vec2Skew(Vector3d pnt);


#endif
