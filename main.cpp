
//참고 https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/group___enumerations_ga3507ee60c1ffe1909096e2080dd2a05d.html
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <k4a/k4a.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

using namespace std;

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
//-----------VO------------------
#include <System_SLAM.h>

using namespace cv;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
int main(int argc, char** argv)
{
    
    ros::init(argc,argv,"simplebox");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("points", 1);
    ros::Rate loop_rate(1);

    //카메라 자세를 보낸다. tag : ros, tf
    tf::TransformBroadcaster br_tf;
    tf::Transform transform;
    //https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/transformation/main.cpp
    k4a_device_t device = NULL;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;//처음에 모든 설정을 꺼둡니다.
    config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;//320x288
    //NFOV : 좁은 FOV/ WFOC : 넓은 FOV
    //binned mode : 캡쳐하는 카메라의 레솔루션을 인전한 센서 픽셀을 결합하여 줄입니다.
    config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.synchronized_images_only = true;


    uint32_t device_count = k4a_device_get_installed_count();
    cout<<device_count<<endl;
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        printf("Failed to open device\n");
        k4a_device_close(device);
        return 1;
    }
    
    k4a_calibration_t calibration; // calibration정보를 넣는 구조체
    k4a_device_get_calibration(device,config.depth_mode,config.color_resolution, &calibration);
    k4a_transformation_t transform_d2c = k4a_transformation_create(&calibration);
    k4a_device_start_cameras(device, &config);
    
    // depth이미지 얻어오기
    // 카메라 변환 메트릭스를 만들자. (u,v) ->(x,y,1) ->(x*z,y*z,z) : k^-1이 필요함.
    k4a_calibration_intrinsic_parameters_t *intrinsics = &calibration.color_camera_calibration.intrinsics.parameters;
    vector<float> _camera_matrix = {
        intrinsics->param.fx, 0.f, intrinsics->param.cx, 0.f, intrinsics->param.fy, intrinsics->param.cy, 0.f, 0.f, 1.f
    };
    Mat camera_matrix = Mat(3, 3, CV_32F, &_camera_matrix[0]);
    cout<<"Camera K"<<endl;
    cout<<camera_matrix<<endl; 
    vector<float> _dist_coeffs = { intrinsics->param.k1, intrinsics->param.k2, intrinsics->param.p1,
                                   intrinsics->param.p2, intrinsics->param.k3, intrinsics->param.k4,
                                   intrinsics->param.k5, intrinsics->param.k6 };
    Mat dist_coeffs = Mat(8, 1, CV_32F, &_dist_coeffs[0]);
    cout<<"Camera Distortion"<<endl;
    cout<<dist_coeffs<<endl;
    int camera_height = calibration.color_camera_calibration.resolution_height;
    int camera_width = calibration.color_camera_calibration.resolution_width;
    Mat PixelPoints = Mat::zeros(camera_height*camera_width,1,CV_32FC2);
    for(int i=0; i<camera_height; i++)
    {
        for(int j=0; j<camera_width; j++)
        {
            PixelPoints.at<Vec2f>(i*camera_width+j,0)[0] = (float)j;
            PixelPoints.at<Vec2f>(i*camera_width+j,0)[1] = (float)i;
        }
    }
    Mat Undist_pixel;
    undistortPoints(PixelPoints,Undist_pixel,camera_matrix,dist_coeffs,cv::Mat(),camera_matrix);
    
    Mat inv_inst = camera_matrix.inv();
    vector<double> additional_data;
    System vo_module(TYPE_RGBD,camera_matrix,dist_coeffs,additional_data);
    while (ros::ok())
    {
        k4a_capture_t capture = NULL;
        // k4a_image_t xy_table = NULL;
        
        const int32_t TIMEOUT_IN_MS = 1000;

        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a capture\n");
            return 0;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a capture\n");
            return 0;
        }
        k4a_image_t depth_image = NULL;
        k4a_image_t color_image = NULL;
        k4a_image_t transformed_depth_image = NULL;
        depth_image = k4a_capture_get_depth_image(capture);
        color_image =k4a_capture_get_color_image(capture);
        int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
        int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
        k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                    color_image_width_pixels,
                                                    color_image_height_pixels,
                                                    color_image_width_pixels * (int)sizeof(uint16_t),
                                                    &transformed_depth_image);//이미지를 하나 만듭니다.

        k4a_result_t res= k4a_transformation_depth_image_to_color_camera(transform_d2c,depth_image,transformed_depth_image);
        int color_width =k4a_image_get_width_pixels(transformed_depth_image);
        int color_height =k4a_image_get_height_pixels(transformed_depth_image);

        // cout<<color_width<<", "<<color_height<<endl;
        //opencv를 통한 최적화
        k4a_calibration_intrinsic_parameters_t *intrinsics = &calibration.color_camera_calibration.intrinsics.parameters;
        uint8_t* color_buffer = k4a_image_get_buffer(color_image);

        int rows_color = k4a_image_get_height_pixels(color_image);
        int cols_color = k4a_image_get_width_pixels(color_image);
        Mat color(rows_color, cols_color, CV_8UC4, (void*)color_buffer, cv::Mat::AUTO_STEP);
        // transformed_depth_image =depth_image;
        int rows_depth = k4a_image_get_height_pixels(transformed_depth_image);
        int cols_depth = k4a_image_get_width_pixels(transformed_depth_image);
        uint8_t* depth_buffer = k4a_image_get_buffer(transformed_depth_image);
        // int rows_depth = k4a_image_get_height_pixels(depth_image);
        // int cols_depth = k4a_image_get_width_pixels(depth_image);
        // uint8_t* depth_buffer = k4a_image_get_buffer(depth_image);
        Mat depth(rows_depth,
                    cols_depth,
                    CV_16U,
                    (void *)depth_buffer
                    , cv::Mat::AUTO_STEP);
        cv::cvtColor(color,color, COLOR_BGRA2BGR);//bgr이미지로 바꿉니다.
        
        PointCloud::Ptr msg (new PointCloud);
        msg->header.frame_id = "camera";
        
        msg->height = 1;

        float x_3d;
        float y_3d;
        int size =0;
        Mat pixelPoint(3,1,CV_32F);
        Mat Point_Width_depth;
        for(int i=0; i<camera_height; i++)
        {
            for(int j=0; j<camera_width; j++)
            {
                uint16_t depth_i_j = depth.at<uint16_t>(i,j);
                if(depth_i_j ==0)
                {
                    continue;
                }
                else
                {
                    float depth_meter = (1.0f / 1000.0f)*depth_i_j;
                    pixelPoint.at<float>(0,0) = Undist_pixel.at<Vec2f>(i*camera_width+j,0)[0];
                    pixelPoint.at<float>(1,0) = Undist_pixel.at<Vec2f>(i*camera_width+j,0)[1];
                    pixelPoint.at<float>(2,0) = 1.0;
                    Point_Width_depth =(inv_inst*pixelPoint)*depth_meter;
                    pcl::PointXYZRGB point = pcl::PointXYZRGB((int)color.at<Vec3b>(i,j)[2],(int)color.at<Vec3b>(i,j)[1],(int)color.at<Vec3b>(i,j)[0]);
                    x_3d = Point_Width_depth.at<float>(0,0);
                    y_3d = Point_Width_depth.at<float>(1,0);
                    point.x=x_3d;
                    point.y=y_3d;
                    point.z=depth_meter;
                    msg->points.push_back(point);
                    size++;
                }
            }
        }
        msg->width = size;
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        pub.publish (*msg);

        //추정된 카메라 위치 보내기 map에 대한 카메라 위치
        transform.setOrigin( tf::Vector3(0, 0.0, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        br_tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera"));

        k4a_image_release(depth_image);
        k4a_image_release(color_image);
        k4a_image_release(transformed_depth_image);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
