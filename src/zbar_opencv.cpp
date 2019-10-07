#include "/home/kerry/zbar_opencv/src/zbar_opencv/include/zbar_opencv.h"

using namespace std;
using namespace cv;
using namespace zbar;
using namespace Eigen;

nav_msgs::Odometry current_odometry;

ros::Publisher marker_pub;

RNG rng(12345);

ImageConverter image_converter;



int main(int argc, char **argv) {

    ros::init(argc,argv,"zbar_opencv");
    ros::NodeHandle nh;
    
    ros::Subscriber odometry_sub = nh.subscribe("/vins_estimator/odometry", 10, &odometry_callback);
    //使用image_transport订阅图像话题“in” 和 发布图像话题“out”
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw",1,&image_callback);

    marker_pub = nh.advertise<visualization_msgs::Marker>("target_marker", 1);

    // image_transport::Publisher image_pub;
    //image_pub   = it.advertise("zbar_opencv",1); 

    image_converter.K << 860.216512883632,            0, 0,
                                    0, 864.897891399564, 0,
                     338.631799407610, 246.185292256007, 1;

    ros::spin();
    return 0;
}

void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_odometry = *msg;
}

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //将ROS图像消息转化为适合Opencv的CvImage
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }
    zbarscanner(cv_ptr);
    //image_pub.publish(cv_ptr->toImageMsg());
    //image_pub.publish(cv_ptr->toImageMsg(), cam_info);
}


void zbarscanner(cv_bridge::CvImagePtr cv_ptr)
{
    // Create a zbar reader
    ImageScanner scanner;

    // Configure the reader
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    nav_msgs::Odometry tmp_odometry;
    Quaterniond q;

    // Capture an OpenCV frame
    Mat frame,frame_grayscale;
    frame=cv_ptr->image;
    // Convert to grayscale
    cvtColor(frame, frame_grayscale, CV_BGR2GRAY);

    // Obtain image data
    int width = frame_grayscale.cols;
    int height = frame_grayscale.rows;
    uchar *raw = (uchar *)(frame_grayscale.data);

    // Wrap image data
    Image image(width, height, "Y800", raw, width * height);

    // Scan the image for barcodes
    scanner.scan(image);
    double length = 0.0;

    // Extract results
    int counter = 0;
    
    for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) 
    {
        //cout<< symbol->get_data() << endl;

        //cout<< counter << endl;
        // Draw location of the symbols found
        if (symbol->get_location_size() == 4) 
        {
            length = sqrtf((symbol->get_location_x(0)-symbol->get_location_x(1))*(symbol->get_location_x(0)-symbol->get_location_x(1)) + 
                           (symbol->get_location_y(0)-symbol->get_location_y(1))*(symbol->get_location_y(0)-symbol->get_location_y(1)));

            //cout<< "length is: " << length << endl;

            tmp_odometry = current_odometry; // register the current odometry
        
            switch(image_converter.state)
            {
                case 0:
                    if(image_converter.count==10)
                    {
                        q.x() = tmp_odometry.pose.pose.orientation.x;
                        q.y() = tmp_odometry.pose.pose.orientation.y;
                        q.z() = tmp_odometry.pose.pose.orientation.z;
                        q.w() = tmp_odometry.pose.pose.orientation.w;
                        image_converter.R1 = q.toRotationMatrix();
                        image_converter.C1(0) = tmp_odometry.pose.pose.position.x;
                        image_converter.C1(1) = tmp_odometry.pose.pose.position.y;
                        image_converter.C1(2) = tmp_odometry.pose.pose.position.z;
                        image_converter.pic0pnts(0) = symbol->get_location_x(0);
                        image_converter.pic0pnts(1) = symbol->get_location_y(0);
                        image_converter.state = 1;
                        image_converter.count = 0;
                    } else {
                        image_converter.count +=1;
                    }
                    break;

                case 1:
                    if(image_converter.count==10)
                    {
                        q.x() = tmp_odometry.pose.pose.orientation.x;
                        q.y() = tmp_odometry.pose.pose.orientation.y;
                        q.z() = tmp_odometry.pose.pose.orientation.z;
                        q.w() = tmp_odometry.pose.pose.orientation.w;
                        image_converter.R2 = q.toRotationMatrix();
                        image_converter.C2(0) = tmp_odometry.pose.pose.position.x;
                        image_converter.C2(1) = tmp_odometry.pose.pose.position.y;
                        image_converter.C2(2) = tmp_odometry.pose.pose.position.z;
                        image_converter.pic1pnts(0) = symbol->get_location_x(0);
                        image_converter.pic1pnts(1) = symbol->get_location_y(0);                        
                        image_converter.state = 2;
                        image_converter.count = 0;
                    } else {
                        image_converter.count +=1;
                    }
                    break;

                case 2:
                    image_converter.P3D = image_converter.triangulationPoints();
                    cout<< "target location: "<< image_converter.P3D(0)<<","<<image_converter.P3D(1)<<","<<image_converter.P3D(2)<<endl;
                    
                    visualization_msgs::Marker marker;
                    // 设置帧 ID和时间戳
                    marker.header.frame_id = "world";
                    marker.header.stamp = ros::Time::now();

                    // 设置该标记的命名空间和ID，ID应该是独一无二的
                    // 具有相同命名空间和ID的标记将会覆盖前一个
                    marker.ns = "basic_shapes";
                    marker.id = 0;

                    // 设置标记类型，初始值为立方体。在立方体、球体、箭头和 圆柱体之间循环
                    marker.type = visualization_msgs::Marker::CUBE;;

                    // 设置标记行为：ADD（添 加），DELETE（删 除）
                    marker.action = visualization_msgs::Marker::ADD;

                    //设置标记位姿。 
                    marker.pose.position.x = image_converter.P3D(0);
                    marker.pose.position.y = image_converter.P3D(1);
                    marker.pose.position.z = image_converter.P3D(2);
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.17;
                    marker.scale.y = 0.17;
                    marker.scale.z = 0.01;
                    marker.color.r = 0.0f;
                    marker.color.g = 1.0f;
                    marker.color.b = 0.0f;
                    marker.color.a = 1.0;

                    marker_pub.publish(marker);


                    image_converter.state = 0;
                    break;
            }
        }
        counter++;
    }

}

Vector3d ImageConverter::triangulationPoints()
{
    Matrix3d eye = Matrix3d::Identity();//3*3单位矩阵
    Matrix3d zero = Matrix3d::Zero();//3*3单位矩阵
    Vector3d out;
    MatrixXd A1,A2;

    Vector3d  x1, x2;
    MatrixXd  P1(3,4), P2(3,4);
    MatrixXd  CN1(3,4), CN2(3,4);
    Matrix3d  x_skew1, x_skew2;
    MatrixXd  A(6,4);
    MatrixXd  U,W,V,VN;

    x1 << pic0pnts, 1;
    x2 << pic1pnts, 1;//像素矩阵补成1*3

    //std::cout<< x1 << "\n"<<std::endl;

    x_skew1 = Vec2Skew(x1);
    x_skew2 = Vec2Skew(x2);//Vec2Skew

    CN1 << eye,-C1;
    CN2 << eye,-C2;
    //std::cout<<CN1<<"\n"<<std::endl;

    P1 = K*R1*CN1;
    P2 = K*R2*CN2;//P1 = K*R1*[eye(3) -C1];

    //std::cout<<P1<<"\n"<<std::endl;


    A1 = x_skew1*P1;
    A2 = x_skew2*P2;
//
//    std::cout<< A1 << std::endl;
//    std::cout<< A2 << std::endl;

    A << A1, A2;
//    std::cout<<A<<"\n"<<std::endl;

    JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
    V = svd.matrixV();
    //std::cout<<V<<"\n"<<std::endl;

    out(0) = V(0,3)/V(3,3);
    out(1) = V(1,3)/V(3,3);
    out(2) = V(2,3)/V(3,3);
    //std::cout<<out<<"\n"<<std::endl;

    return out;
}


Matrix3d Vec2Skew(Vector3d pnt)
{
    Matrix3d XN;
    XN << 0,-pnt(2),pnt(1),pnt(2),0,-pnt(0),-pnt(1),pnt(0),0;
    return XN;
}

