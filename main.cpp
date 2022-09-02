
//참고 https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/group___enumerations_ga3507ee60c1ffe1909096e2080dd2a05d.html
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <k4a/k4a.h>
#include <math.h>

using namespace std;

#include <opencv4/opencv2/core.hpp>
// #include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/opencv.hpp>
// #include <opencv4/opencv2/rgbd.hpp>
// #include <opencv4/opencv2/viz.hpp>
using namespace cv;
string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


int main()
{
    //https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/transformation/main.cpp
    k4a_device_t device = NULL;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;//처음에 모든 설정을 꺼둡니다.
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;//320x288
    //NFOV : 좁은 FOV/ WFOC : 넓은 FOV
    //binned mode : 캡쳐하는 카메라의 레솔루션을 인전한 센서 픽셀을 결합하여 줄입니다.
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.synchronized_images_only = true;


    uint32_t device_count = k4a_device_get_installed_count();
    cout<<"kinect camera 수 "<< device_count<<endl;
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
    while (waitKey(1) !=27)
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
        double min;
        double max;
        cv::minMaxIdx(depth, &min, &max);
        cv::Mat adjMap;
        depth.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);
        cv::Mat falseColorsMap;
        applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_AUTUMN);

        cv::cvtColor(color,color, COLOR_BGRA2BGR);//bgr이미지로 바꿉니다.
        
        // cout<<type2str(depth.type())<<endl;
        imshow("tst",color);
        imshow("tstdfdsf",falseColorsMap);
        k4a_image_release(depth_image);
        k4a_image_release(color_image);
        k4a_image_release(transformed_depth_image);
    }
    
    return 0;
}
