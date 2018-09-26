#include <stdio.h>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "boost/endian/conversion.hpp"
#include <boost/filesystem.hpp>
#include <camera_info_manager/camera_info_manager.h>

#define WIDTH_ID 3
#define HEIGHT_ID 4
#define FPS_ID 5

namespace arti
{
class StereoCamera
{
public:
  /**
   * @brief      { stereo camera driver }
   *
   * @param[in]  resolution  The resolution
   * @param[in]  frame_rate  The frame rate
   */
  StereoCamera(std::string device_name, int resolution, double frame_rate) : frame_rate_(frame_rate)
  {
      boost::filesystem::directory_iterator end;             //迭代器的终点
      boost::filesystem::directory_iterator dir("/sys/class/video4linux");     //迭代器的起点
      std::vector<std::string> fileNameList;
      int dev_id =0;
      for (dir;dir!=end;dir++)
      {
          std::string fileName(dir->path().string()+"/name") ;
          fileNameList.push_back(fileName);
      }
      std::sort(fileNameList.begin(),fileNameList.end());
      for (int i = 0 ;i<fileNameList.size();++i)
      {
          std::ifstream infile;
          std::string stringLine;
          infile.open (fileNameList[i].c_str());
          std::getline(infile,stringLine);
          infile.close();
          if(stringLine == device_name)break;
          dev_id++;

      }

    camera_ = new cv::VideoCapture(dev_id);
    cv::Mat raw;
    cv::Mat left_image;
    cv::Mat right_image;
    setResolution(resolution);
    // // this function doesn't work very well in current Opencv 2.4, so, just use ROS to control frame rate.
     setFrameRate(frame_rate);

    ROS_INFO("Stereo Camera Set Resolution %d, width %f, height %f", resolution, camera_->get(WIDTH_ID),
             camera_->get(HEIGHT_ID));
  }

  ~StereoCamera()
  {
    // std::cout << "Destroy the pointer" << std::endl;
    delete camera_;
  }

  /**
   * @brief      Sets the resolution.
   *
   * @param[in]  type  The type
   */
  void setResolution(int type)
  {
    switch (type)
    {
      case 0:
        width_ = 4416;
        height_ = 1242;
        break;
      case 1:
        width_ = 3840;
        height_ = 1080;
        break;
      case 2:
        width_ = 2560;
        height_ = 720;
        break;
      case 3:
        width_ = 1344;
        height_ = 376;
        break;
      default:
        ROS_FATAL("Unknow resolution passed to camera: %d", type);
    }

    camera_->set(WIDTH_ID, width_);
    camera_->set(HEIGHT_ID, height_);
    // make sure that the number set are right from the hardware
    width_ = camera_->get(WIDTH_ID);
    height_ = camera_->get(HEIGHT_ID);
  }

  /**
   * @brief      Sets the frame rate.
   *
   * @param[in]  frame_rate  The frame rate
   */
  void setFrameRate(double frame_rate)
  {
    camera_->set(FPS_ID, frame_rate);
    frame_rate_ = camera_->get(FPS_ID);
  }

  /**
   * @brief      Gets the images.
   *
   * @param      left_image   The left image
   * @param      right_image  The right image
   *
   * @return     The images.
   */
  bool getImages(cv::Mat& left_image, cv::Mat& right_image)
  {
    cv::Mat raw;
    if (camera_->read(raw))
    {
      cv::Rect left_rect(0, 0, width_ / 2, height_);
      cv::Rect right_rect(width_ / 2, 0, width_ / 2, height_);
      left_image = raw(left_rect);
      right_image = raw(right_rect);
      cv::waitKey(5);
      return true;
    }
    else
    {
      return false;
    }
  }
  int width_;
  int height_;
private:
  cv::VideoCapture* camera_;
  double frame_rate_;
  bool cv_three_;
};

/**
 * @brief       the camera ros warpper class
 */
class ZedCameraROS
{
public:
  /**
   * @brief      { function_description }
   *
   * @param[in]  resolution  The resolution
   * @param[in]  frame_rate  The frame rate
   */
  ZedCameraROS()
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    // get ros param
    private_nh.param("resolution", resolution_, 1);
    private_nh.param("frame_rate", frame_rate_, 30.0);
    private_nh.param("config_file_location", config_file_location_, std::string(""));
    private_nh.param("left_frame_id", left_frame_id_, std::string("left_camera"));
    private_nh.param("right_frame_id", right_frame_id_, std::string("right_camera"));
    private_nh.param("show_image", show_image_, false);
    private_nh.param("use_zed_config", use_zed_config_, true);
    private_nh.param("device_name", device_name_, std::string("ZED"));
    private_nh.param("encoding", encoding_, std::string("brg8"));

    correctFramerate(resolution_, frame_rate_);

    ROS_INFO("Try to initialize the camera");
    StereoCamera zed(device_name_, resolution_, frame_rate_);
    width_ = zed.width_/2;
    height_ = zed.height_;
    ROS_INFO("Initialized the camera");

    // setup publisher stuff
    image_transport::ImageTransport it(nh);
    image_transport::Publisher left_image_pub = it.advertise("left/image_raw", 1);
    image_transport::Publisher right_image_pub = it.advertise("right/image_raw", 1);

    ros::Publisher left_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
    ros::Publisher right_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);

    sensor_msgs::CameraInfo left_info, right_info;

    ROS_INFO("Try load camera calibration files");
    if (use_zed_config_)
    {
      ROS_INFO("Loading from zed calibration files");
      // get camera info from zed
      if (!config_file_location_.empty())
      {
        try
        {
          getZedCameraInfo(config_file_location_, resolution_, left_info, right_info);
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
    }
    else
    {
      ROS_INFO("Loading from ROS calibration files");
      // here we just use camera infor manager to load info
      // get config from the left, right.yaml in config
      ros::NodeHandle left_nh("left");
      ros::NodeHandle right_nh("right");
      camera_info_manager::CameraInfoManager left_info_manager(left_nh, "camera/left",
                                                               "package://zed_cpu_ros/config/left.yaml");
      left_info = left_info_manager.getCameraInfo();

      camera_info_manager::CameraInfoManager right_info_manager(right_nh, "camera/right",
                                                                "package://zed_cpu_ros/config/right.yaml");
      right_info = right_info_manager.getCameraInfo();

      left_info.header.frame_id = left_frame_id_;
      right_info.header.frame_id = right_frame_id_;
    }
    ROS_INFO("Got camera calibration files");
    // loop to publish images;

    cv::Mat left_image_undist, right_image_undist;
    cv::Mat left_image, right_image;
    ros::Rate r(frame_rate_);

    while (nh.ok())
    {
      ros::Time now = ros::Time::now();
      if (!zed.getImages(left_image_undist, right_image_undist))
      {
        ROS_INFO_ONCE("Can't find camera");
      }
      else
      {
        cv::remap(left_image_undist, left_image, left_map1, left_map2, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        cv::remap(right_image_undist, right_image, right_map1, right_map2, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        ROS_INFO_ONCE("Success, found camera");
      }
      if (show_image_)
      {
        cv::imshow("left", left_image);
        cv::imshow("right", right_image);
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
      r.sleep();
      // since the frame rate was set inside the camera, no need to do a ros sleep
    }
  }

  /**
   * @brief      Gets the camera information From Zed config.
   *
   * @param[in]  config_file         The configuration file
   * @param[in]  resolution          The resolution
   * @param[in]  left_cam_info_msg   The left camera information message
   * @param[in]  right_cam_info_msg  The right camera information message
   */
  void getZedCameraInfo(std::string config_file, int resolution, sensor_msgs::CameraInfo& left_info,
                        sensor_msgs::CameraInfo& right_info)
  {
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);
    std::string left_str = "LEFT_CAM_";
    std::string right_str = "RIGHT_CAM_";
    std::string reso_str = "";

    switch (resolution)
    {
      case 0:
        reso_str = "2K";
        break;
      case 1:
        reso_str = "FHD";
        break;
      case 2:
        reso_str = "HD";
        break;
      case 3:
        reso_str = "VGA";
        break;
    }
    // left value
    double l_cx = pt.get<double>(left_str + reso_str + ".cx");
    double l_cy = pt.get<double>(left_str + reso_str + ".cy");
    double l_fx = pt.get<double>(left_str + reso_str + ".fx");
    double l_fy = pt.get<double>(left_str + reso_str + ".fy");
    double l_new_cx = pt.get<double>(left_str + reso_str + ".new_cx");
    double l_new_cy = pt.get<double>(left_str + reso_str + ".new_cy");
    double l_new_fx = pt.get<double>(left_str + reso_str + ".new_fx");
    double l_new_fy = pt.get<double>(left_str + reso_str + ".new_fy");
    double l_k1 = pt.get<double>(left_str + reso_str + ".k1");
    double l_k2 = pt.get<double>(left_str + reso_str + ".k2");
    // right value
    double r_cx = pt.get<double>(right_str + reso_str + ".cx");
    double r_cy = pt.get<double>(right_str + reso_str + ".cy");
    double r_fx = pt.get<double>(right_str + reso_str + ".fx");
    double r_fy = pt.get<double>(right_str + reso_str + ".fy");
    double r_new_cx = pt.get<double>(right_str + reso_str + ".new_cx");
    double r_new_cy = pt.get<double>(right_str + reso_str + ".new_cy");
    double r_new_fx = pt.get<double>(right_str + reso_str + ".new_fx");
    double r_new_fy = pt.get<double>(right_str + reso_str + ".new_fy");
    double r_k1 = pt.get<double>(right_str + reso_str + ".k1");
    double r_k2 = pt.get<double>(right_str + reso_str + ".k2");

    // get baseline and convert mm to m
    boost::optional<double> baselineCheck;
    double baseline = 0.0;
    // some config files have "Baseline" instead of "BaseLine", check accordingly...
    if (baselineCheck = pt.get_optional<double>("STEREO.BaseLine"))
    {
      baseline = pt.get<double>("STEREO.BaseLine") * 0.001;
    }
    else if (baselineCheck = pt.get_optional<double>("STEREO.Baseline"))
    {
      baseline = pt.get<double>("STEREO.Baseline") * 0.001;
    }
    else
    {
      throw std::runtime_error("baseline parameter not found");
    }

    // get Rx and Rz
    double rx = pt.get<double>("STEREO.RX_" + reso_str);
    double rz = pt.get<double>("STEREO.RZ_" + reso_str);
    double ry = pt.get<double>("STEREO.CV_" + reso_str);

    // assume zeros, maybe not right
    double p1 = 0, p2 = 0, k3 = 0;

    left_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    right_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // TODO(dizeng) verify loading default zed config is still working

    // distortion parameters
    // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    left_D = cv::Mat::zeros(5,1,CV_64F);
    left_D.at<double>(0,0)=l_k1;
    left_D.at<double>(1,0)=l_k2;
    left_D.at<double>(2,0)=k3;
    left_D.at<double>(3,0)=p1;
    left_D.at<double>(4,0)=p2;
    left_info.D.resize(5);
    left_info.D[0] = 0;
    left_info.D[1] = 0;
    left_info.D[2] = k3;
    left_info.D[3] = p1;
    left_info.D[4] = p2;

    right_D = cv::Mat::zeros(5,1,CV_64F);
    right_D.at<double>(0,0)=r_k1;
    right_D.at<double>(1,0)=r_k2;
    right_D.at<double>(2,0)=k3;
    right_D.at<double>(3,0)=p1;
    right_D.at<double>(4,0)=p2;
    right_info.D.resize(5);
    right_info.D[0] = 0;
    right_info.D[1] = 0;
    right_info.D[2] = k3;
    right_info.D[3] = p1;
    right_info.D[4] = p2;

    // Intrinsic camera matrix
    // 	[fx  0 cx]
    // K =  [ 0 fy cy]
    //	[ 0  0  1]
    left_K = cv::Mat::zeros(3,3,CV_64F);
    left_K.at<double>(0,0)=l_fx;
    left_K.at<double>(0,2)=l_cx;
    left_K.at<double>(1,1)=l_fy;
    left_K.at<double>(1,2)=l_cy;
    left_K.at<double>(2,2)=1.0;
    left_info.K.fill(0.0);
    left_info.K[0] = l_new_fx;
    left_info.K[2] = l_new_cx;
    left_info.K[4] = l_new_fy;
    left_info.K[5] = l_new_cy;
    left_info.K[8] = 1.0;
    new_left_K = cv::Mat::zeros(3,3,CV_64F);
    new_left_K.at<double>(0,0)=l_new_fx;
    new_left_K.at<double>(0,2)=l_new_cx;
    new_left_K.at<double>(1,1)=l_new_fy;
    new_left_K.at<double>(1,2)=l_new_cy;
    new_left_K.at<double>(2,2)=1.0;

    right_K = cv::Mat::zeros(3,3,CV_64F);
    right_K.at<double>(0,0)=l_fx;
    right_K.at<double>(0,2)=l_cx;
    right_K.at<double>(1,1)=l_fy;
    right_K.at<double>(1,2)=l_cy;
    right_K.at<double>(2,2)=1.0;
    right_info.K.fill(0.0);
    right_info.K[0] = r_new_fx;
    right_info.K[2] = r_new_cx;
    right_info.K[4] = r_new_fy;
    right_info.K[5] = r_new_cy;
    right_info.K[8] = 1.0;
    new_right_K = cv::Mat::zeros(3,3,CV_64F);
    new_right_K.at<double>(0,0)=r_new_fx;
    new_right_K.at<double>(0,2)=r_new_cx;
    new_right_K.at<double>(1,1)=r_new_fy;
    new_right_K.at<double>(1,2)=r_new_cy;
    new_right_K.at<double>(2,2)=1.0;
    std::cout<<new_left_K<<std::endl;
    std::cout<<new_right_K<<std::endl;

    // rectification matrix
    // Rl = R_rect, R_r = R * R_rect
    // since R is identity, Rl = Rr;
    left_info.R.fill(0.0);
    right_info.R.fill(0.0);
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << rx, ry, rz);
    cv::Mat rmat(3, 3, CV_64F);
    cv::Rodrigues(rvec, rmat);
    int id = 0;
    cv::MatIterator_<double> it, end;
    for (it = rmat.begin<double>(); it != rmat.end<double>(); ++it, id++)
    {
      left_info.R[id] = *it;
      right_info.R[id] = *it;
    }

    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    left_info.P.fill(0.0);
    left_info.P[0] = l_new_fx;
    left_info.P[2] = l_new_cx;
    left_info.P[5] = l_new_fy;
    left_info.P[6] = l_new_cy;
    left_info.P[10] = 1.0;

    right_info.P.fill(0.0);
    right_info.P[0] = r_new_fx;
    right_info.P[2] = r_new_cx;
    right_info.P[3] = (-1 * l_new_fx * baseline);
    right_info.P[5] = r_new_fy;
    right_info.P[6] = r_new_cy;
    right_info.P[10] = 1.0;

    left_info.width = right_info.width = width_;
    left_info.height = right_info.height = height_;

    left_info.header.frame_id = left_frame_id_;
    right_info.header.frame_id = right_frame_id_;

    cv::Size image_size(width_,height_);

    cv::initUndistortRectifyMap(left_K, left_D, cv::Mat(),new_left_K,image_size, 0, left_map1, left_map2);
    cv::initUndistortRectifyMap(right_K, right_D, cv::Mat(), new_right_K,image_size, 0, right_map1, right_map2);
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
  /**
   * @brief      Correct frame rate according to resolution
   *
   * @param[in]  resolution          The resolution
   * @param      frame_rate   			 The camera frame rate
   */
  void correctFramerate(int resolution, double& frame_rate)
  {
    double max_frame_rate;
    std::string reso_str = "";
    switch (resolution)
    {
      case 0:
        max_frame_rate = 15;
        reso_str = "2K";
        break;
      case 1:
        max_frame_rate = 30;
        reso_str = "FHD";
        break;
      case 2:
        max_frame_rate = 60;
        reso_str = "HD";
        break;
      case 3:
        max_frame_rate = 100;
        reso_str = "VGA";
        break;
      default:
        ROS_FATAL("Unknow resolution passed");
        return;
    }
    if (frame_rate > max_frame_rate){
      ROS_WARN("frame_rate(%fHz) too high for resolution(%s), downgraded to %fHz", frame_rate, reso_str.c_str(),
               max_frame_rate);
      frame_rate = max_frame_rate;
    }
  }

private:
  int resolution_;
  double frame_rate_;
  bool show_image_, use_zed_config_;
  double width_, height_;
  std::string left_frame_id_, right_frame_id_,device_name_;
  std::string config_file_location_;
  std::string encoding_;

  cv::Mat left_K,left_D;
  cv::Mat right_K,right_D;
  cv::Mat new_left_K,new_right_K;
  cv::Mat left_map1,left_map2;
  cv::Mat right_map1,right_map2;
};
}

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "zed_camera");
    arti::ZedCameraROS zed_ros;
    return EXIT_SUCCESS;
  }
  catch (std::runtime_error& e)
  {
    ros::shutdown();
    return EXIT_FAILURE;
  }
}
