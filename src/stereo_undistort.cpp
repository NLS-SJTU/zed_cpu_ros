#include <stdio.h>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/image_encodings.h"
#include "boost/endian/conversion.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicyCam;

int getCvType(const std::string& encoding)
{
    // Check for the most common encodings first
    if (encoding == sensor_msgs::image_encodings::BGR8)   return CV_8UC3;
    if (encoding == sensor_msgs::image_encodings::MONO8)  return CV_8UC1;
    if (encoding == sensor_msgs::image_encodings::RGB8)   return CV_8UC3;
    if (encoding == sensor_msgs::image_encodings::MONO16) return CV_16UC1;
    if (encoding == sensor_msgs::image_encodings::BGR16)  return CV_16UC3;
    if (encoding == sensor_msgs::image_encodings::RGB16)  return CV_16UC3;
    if (encoding == sensor_msgs::image_encodings::BGRA8)  return CV_8UC4;
    if (encoding == sensor_msgs::image_encodings::RGBA8)  return CV_8UC4;
    if (encoding == sensor_msgs::image_encodings::BGRA16) return CV_16UC4;
    if (encoding == sensor_msgs::image_encodings::RGBA16) return CV_16UC4;

    // For bayer, return one-channel
    if (encoding == sensor_msgs::image_encodings::BAYER_RGGB8) return CV_8UC1;
    if (encoding == sensor_msgs::image_encodings::BAYER_BGGR8) return CV_8UC1;
    if (encoding == sensor_msgs::image_encodings::BAYER_GBRG8) return CV_8UC1;
    if (encoding == sensor_msgs::image_encodings::BAYER_GRBG8) return CV_8UC1;
    if (encoding == sensor_msgs::image_encodings::BAYER_RGGB16) return CV_16UC1;
    if (encoding == sensor_msgs::image_encodings::BAYER_BGGR16) return CV_16UC1;
    if (encoding == sensor_msgs::image_encodings::BAYER_GBRG16) return CV_16UC1;
    if (encoding == sensor_msgs::image_encodings::BAYER_GRBG16) return CV_16UC1;

    // Miscellaneous
    if (encoding == sensor_msgs::image_encodings::YUV422) return CV_8UC2;
}

cv::Mat matFromImage(const sensor_msgs::Image& source)
{
    int source_type = getCvType(source.encoding);
    int byte_depth = sensor_msgs::image_encodings::bitDepth(source.encoding) / 8;
    int num_channels = sensor_msgs::image_encodings::numChannels(source.encoding);

    if (source.step < source.width * byte_depth * num_channels)
    {
        std::stringstream ss;
        ss << "Image is wrongly formed: step < width * byte_depth * num_channels  or  " << source.step << " != " <<
              source.width << " * " << byte_depth << " * " << num_channels;
        throw ss.str();
    }

    if (source.height * source.step != source.data.size())
    {
        std::stringstream ss;
        ss << "Image is wrongly formed: height * step != size  or  " << source.height << " * " <<
              source.step << " != " << source.data.size();
        throw ss.str();
    }

    // If the endianness is the same as locally, share the data
    cv::Mat mat(source.height, source.width, source_type, const_cast<uchar*>(&source.data[0]), source.step);
    if ((boost::endian::order::native == boost::endian::order::big && source.is_bigendian) ||
            (boost::endian::order::native == boost::endian::order::little && !source.is_bigendian) ||
            byte_depth == 1)
        return mat;

    // Otherwise, reinterpret the data as bytes and switch the channels accordingly
    mat = cv::Mat(source.height, source.width, CV_MAKETYPE(CV_8U, num_channels*byte_depth),
                  const_cast<uchar*>(&source.data[0]), source.step);
    cv::Mat mat_swap(source.height, source.width, mat.type());

    std::vector<int> fromTo;
    fromTo.reserve(num_channels*byte_depth);
    for(int i = 0; i < num_channels; ++i)
        for(int j = 0; j < byte_depth; ++j)
        {
            fromTo.push_back(byte_depth*i + j);
            fromTo.push_back(byte_depth*i + byte_depth - 1 - j);
        }
    cv::mixChannels(std::vector<cv::Mat>(1, mat), std::vector<cv::Mat>(1, mat_swap), fromTo);

    // Interpret mat_swap back as the proper type
    mat_swap.reshape(num_channels);

    return mat_swap;
}

class StereoCamera
{
public:
    StereoCamera()
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        // get ros param

        private_nh.param("config_file_location", config_file_location_, std::string(""));
        private_nh.param("output_config_xml", output_config_xml_, std::string(""));
        private_nh.param("left_frame_id", left_frame_id_, std::string("left_camera"));
        private_nh.param("right_frame_id", right_frame_id_, std::string("right_camera"));
        private_nh.param("show_image", show_image_, false);
        private_nh.param("encoding", encoding_, std::string("mono8"));

        // setup publisher stuff
        image_transport::ImageTransport it(nh);
        left_image_pub = it.advertise("left/image_raw", 1);
        right_image_pub = it.advertise("right/image_raw", 1);

        left_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
        right_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);

        ROS_INFO("Try load camera calibration files");
        ROS_INFO("Loading from calibration files");
        // get camera info
        if (!config_file_location_.empty())
        {
            try
            {
                getCameraInfo(config_file_location_, left_info, right_info);
            }
            catch (std::runtime_error& e)
            {
                ROS_INFO("Can't load camera info");
                ROS_ERROR("%s", e.what());
                throw e;
            }
        }
        else
        {
            ROS_FATAL("Please input zed config file path");
        }

        ROS_INFO("Got camera calibration files");


        message_filters::Subscriber<sensor_msgs::Image> *imagel_sub;
        message_filters::Subscriber<sensor_msgs::Image> *imager_sub;
        message_filters::Synchronizer<syncPolicyCam> *sync;

        imagel_sub=new message_filters::Subscriber<sensor_msgs::Image>(nh, "/cam0/image_raw",1);
        imager_sub=new message_filters::Subscriber<sensor_msgs::Image>(nh,  "/cam1/image_raw", 1);
        sync=new message_filters::Synchronizer<syncPolicyCam>(syncPolicyCam(2), *imagel_sub, *imager_sub);
        sync->registerCallback(boost::bind(&StereoCamera::processStereoImage,this, _1, _2));

        ros::Rate r(50);
        while (nh.ok())
        {
            ros::spinOnce();
            r.sleep();
        }
    }

    void processStereoImage(const sensor_msgs::ImageConstPtr& imagel,const sensor_msgs::ImageConstPtr& imager)
    {
        now=ros::Time::now();

        cv::Mat left_image_undist=matFromImage(*imagel),right_image_undist=matFromImage(*imager);
        cv::Mat left_image, right_image;
        cv::remap(left_image_undist, left_image, left_map1, left_map2, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        cv::remap(right_image_undist, right_image, right_map1, right_map2, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        ROS_INFO_ONCE("Success, found camera");
        if (show_image_)
        {
            cv::imshow("left", left_image);
            cv::imshow("right", right_image);
            cv::waitKey(1);
        }
        if (left_image_pub.getNumSubscribers() > 0)
        {
            publishImage(left_image, left_image_pub, "left_frame", now);
        }
        if (right_image_pub.getNumSubscribers() > 0)
        {
            publishImage(right_image, right_image_pub, "right_frame", now);
        }
        if (left_cam_info_pub.getNumSubscribers() > 0)
        {
            publishCamInfo(left_cam_info_pub, left_info, now);
        }
        if (right_cam_info_pub.getNumSubscribers() > 0)
        {
            publishCamInfo(right_cam_info_pub, right_info, now);
        }

    }

    void getCameraInfo(std::string config_file, sensor_msgs::CameraInfo& left_info,
                       sensor_msgs::CameraInfo& right_info)
    {
        cv::FileStorage fs(config_file, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            std::cout << "Could not open the setting file: \"" << config_file << "\"" << std::endl;
            exit(-1);
        }

        left_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        right_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        // TODO(dizeng) verify loading default zed config is still working

        fs["left_image_width"] >> width_;
        fs["left_image_height"] >> height_;
        // distortion parameters
        // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
        left_D = cv::Mat::zeros(1,5,CV_64F);
        fs["left_distortion_coefficients"] >> left_D;
        left_info.D.resize(5);
        left_info.D[0] = left_D.at<double>(0,0);
        left_info.D[1] = left_D.at<double>(0,1);
        left_info.D[2] = left_D.at<double>(0,2);
        left_info.D[3] = left_D.at<double>(0,3);
        left_info.D[4] = left_D.at<double>(0,4);

        right_D = cv::Mat::zeros(1,5,CV_64F);
        fs["right_distortion_coefficients"] >> right_D;
        right_info.D.resize(5);
        right_info.D[0] = right_D.at<double>(0,0);
        right_info.D[1] = right_D.at<double>(0,1);
        right_info.D[2] = right_D.at<double>(0,2);
        right_info.D[3] = right_D.at<double>(0,3);
        right_info.D[4] = right_D.at<double>(0,4);

        // Intrinsic camera matrix
        // 	[fx  0 cx]
        // K =  [ 0 fy cy]
        //	[ 0  0  1]
        left_K = cv::Mat::zeros(3,3,CV_64F);
        fs["left_camera_matrix"] >> left_K;
        left_info.K.fill(0.0);
        left_info.K[0] = left_K.at<double>(0,0);
        left_info.K[2] = left_K.at<double>(0,2);
        left_info.K[4] = left_K.at<double>(1,1);
        left_info.K[5] = left_K.at<double>(1,2);
        left_info.K[8] = 1.0;

        right_K = cv::Mat::zeros(3,3,CV_64F);
        fs["right_camera_matrix"] >> right_K;
        right_info.K.fill(0.0);
        right_info.K[0] = right_K.at<double>(0,0);
        right_info.K[2] = right_K.at<double>(0,2);
        right_info.K[4] = right_K.at<double>(1,1);
        right_info.K[5] = right_K.at<double>(1,2);
        right_info.K[8] = 1.0;


        left_T_BS = cv::Mat::zeros(4,4,CV_64F);
        fs["left_T_BS"] >> left_T_BS;

        right_T_BS = cv::Mat::zeros(4,4,CV_64F);
        fs["right_T_BS"] >> right_T_BS;

        // rectification matrix
        // Rl = R_rect, R_r = R * R_rect
        // since R is identity, Rl = Rr;
        left_info.R.fill(0.0);
        right_info.R.fill(0.0);
        cv::Mat rmat_imu2cam = (cv::Mat_<double>(3, 3) << 0, 1, 0,-1,0,0,0,0,1);

        cv::Mat r_left = rmat_imu2cam*left_T_BS.colRange(0,3).rowRange(0,3);
        cv::Mat r_right = rmat_imu2cam*right_T_BS.colRange(0,3).rowRange(0,3);
        cv::Mat t_left = rmat_imu2cam*left_T_BS.colRange(3,4).rowRange(0,3);
        cv::Mat t_right = rmat_imu2cam*right_T_BS.colRange(3,4).rowRange(0,3);
        cv::Mat rmat(3, 3, CV_64F);
        rmat = r_right*r_left.t();
        cv::Mat tvc= -rmat*t_left+t_right;

        std::cout<<"rmat:"<<tvc<<std::endl;
//        std::cout<<"tvc:"<tvc<<std::endl;

        int id = 0;
        cv::MatIterator_<double> it, end;
        for (it = rmat.begin<double>(); it != rmat.end<double>(); ++it, id++)
        {
            left_info.R[id] = *it;
            right_info.R[id] = *it;
        }

        cv::Size image_size(width_,height_);
        output_width_=width_, output_height_=height_;
        cv::Size new_image_size(output_width_,output_height_);

        cv::Mat R1,R2,P1,P2,Q;

        cv::stereoRectify(left_K,left_D, right_K, right_D, image_size,
                          rmat, tvc,  R1,  R2,  P1,  P2,  Q,
                          cv::CALIB_ZERO_DISPARITY,0,new_image_size);


        cv::initUndistortRectifyMap(left_K, left_D, R1,P1,new_image_size, CV_32FC1, left_map1, left_map2);
        cv::initUndistortRectifyMap(right_K, right_D, R2, P2,new_image_size, CV_32FC1, right_map1, right_map2);
        // Projection/camera matrix
        //     [fx'  0  cx' Tx]
        // P = [ 0  fy' cy' Ty]
        //     [ 0   0   1   0]
        left_info.P.fill(0.0);
        left_info.P[0] = P1.at<double>(0,0);
        left_info.P[2] = P1.at<double>(0,2);
        left_info.P[5] = P1.at<double>(1,1);
        left_info.P[6] = P1.at<double>(1,2);
        left_info.P[7] = P1.at<double>(1,3);
        left_info.P[10] = 1.0;
        left_info.P[11] = P1.at<double>(2,3);

        right_info.P.fill(0.0);
        right_info.P[0] = P2.at<double>(0,0);
        right_info.P[2] = P2.at<double>(0,2);
        right_info.P[3] = P2.at<double>(0,3);
        right_info.P[5] = P2.at<double>(1,1);
        right_info.P[6] = P2.at<double>(1,2);
        right_info.P[7] = P2.at<double>(1,3);
        right_info.P[10] = 1.0;
        right_info.P[11] = P2.at<double>(2,3);

        left_info.width = right_info.width = output_width_;
        left_info.height = right_info.height = output_height_;

        left_info.header.frame_id = left_frame_id_;
        right_info.header.frame_id = right_frame_id_;

        cv::Mat M1,M2,D1,D2;
        cv::Mat_<int> sizeM(2,1);
        sizeM.at<int>(0)=output_width_;
        sizeM.at<int>(1)=output_height_;
        M1 = P1.rowRange(0,3).colRange(0,3);
        M2 = P2.rowRange(0,3).colRange(0,3);
        D1 = cv::Mat::zeros(5,1,CV_64F);
        D2 = cv::Mat::zeros(5,1,CV_64F);
        cv::FileStorage fswrite(output_config_xml_,cv::FileStorage::WRITE);
        fswrite<<"M1"<<M1;
        fswrite<<"D1"<<D1;
        fswrite<<"M2"<<M2;
        fswrite<<"D2"<<D2;
        fswrite<<"R1"<<R1;
        fswrite<<"R2"<<R2;
        fswrite<<"P1"<<P1;
        fswrite<<"P2"<<P2;
        fswrite<<"Q"<<Q;
        fswrite<<"size"<<sizeM;
        fswrite.release();


    }

    /**
     * @brief      { publish camera info }
     *
     * @param[in]  pub_cam_info  The pub camera information
     * @param[in]  cam_info_msg  The camera information message
     * @param[in]  now           The now
     */
    void publishCamInfo(const ros::Publisher& pub_cam_info, sensor_msgs::CameraInfo& cam_info_msg, ros::Time now)
    {
        cam_info_msg.header.stamp = now;
        pub_cam_info.publish(cam_info_msg);
    }

    /**
     * @brief      { publish image }
     *
     * @param[in]  img           The image
     * @param      img_pub       The image pub
     * @param[in]  img_frame_id  The image frame identifier
     * @param[in]  t             { parameter_description }
     * @param[in]  encoding      image_transport encoding
     */
    void publishImage(const cv::Mat& img, image_transport::Publisher& img_pub, const std::string& img_frame_id,
                      ros::Time t)
    {
        sensor_msgs::ImagePtr ros_image = boost::make_shared<sensor_msgs::Image>();
        toImageMsg(*ros_image,img,img_frame_id,t);
        img_pub.publish(ros_image);

    }

    void toImageMsg(sensor_msgs::Image& ros_image,cv::Mat image,const std::string& img_frame_id,ros::Time t) const
    {
      ros_image.header.frame_id = img_frame_id;
      ros_image.header.stamp = t;
      ros_image.height = image.rows;
      ros_image.width = image.cols;
      ros_image.encoding = encoding_;

      ros_image.is_bigendian = (boost::endian::order::native == boost::endian::order::big);
      ros_image.step = image.cols * image.elemSize();
      size_t size = ros_image.step * image.rows;
      ros_image.data.resize(size);

      if (image.isContinuous())
      {
        memcpy((char*)(&ros_image.data[0]), image.data, size);
      }
      else
      {
        // Copy by row by row
        uchar* ros_data_ptr = (uchar*)(&ros_image.data[0]);
        uchar* cv_data_ptr = image.data;
        for (int i = 0; i < image.rows; ++i)
        {
          memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
          ros_data_ptr += ros_image.step;
          cv_data_ptr += image.step;
        }
      }
    }
private:

    image_transport::Publisher left_image_pub;
    image_transport::Publisher right_image_pub;

    ros::Publisher left_cam_info_pub ;
    ros::Publisher right_cam_info_pub ;

    sensor_msgs::CameraInfo left_info, right_info;

    ros::Time now;

    bool show_image_;
    double width_, height_;
    double output_width_, output_height_;
    std::string left_frame_id_, right_frame_id_;
    std::string config_file_location_;
    std::string output_config_xml_;
    std::string encoding_;

    cv::Mat left_K,left_D;
    cv::Mat right_K,right_D;
    cv::Mat left_T_BS,right_T_BS;
    cv::Mat left_map1,left_map2;
    cv::Mat right_map1,right_map2;
};


int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "stereo_camera");
        StereoCamera _st_cam;
        return EXIT_SUCCESS;
    }
    catch (std::runtime_error& e)
    {
        ros::shutdown();
        return EXIT_FAILURE;
    }
}
