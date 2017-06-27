#include <ros/ros.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <geometry_msgs/Twist.h>

#include <bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateFlyingStateChanged.h>
#include <bebop_msgs/Ardrone3CameraStateOrientation.h>
#include <bebop_msgs/Ardrone3PilotingStateSpeedChanged.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <angles/angles.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <bebop_hri/debug.h>
#include <opencv2/highgui/highgui.hpp>

#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint3D.h>
#include <visp/vpServo.h>
#include <visp/vpSimulatorCamera.h>
#include <visp_bridge/camera.h>
#include <visp_bridge/image.h>
#include <visp/vpColVector.h>
#include <visp/vpServoDisplay.h>

#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGTK.h>

#include <boost/thread/lock_guard.hpp>
#include <cmath>
#include <cassert>

#include <control_toolbox/pid.h>

#define CLAMP(x, l, h) (((x) > (h)) ? (h) : (((x) < (l)) ? (l) : (x)))

/* Generic 1st order non-linear model
 *
 * d_v = -C * v + G * tan(tilt)
 *
 * For Bebop:
 *
 * Cx: -0.576335778073963
 * Cy: -0.584975281133000
 * Gx: 9.81
 * Gy: -9.81
 *
 * */
class BebopVelocityTiltModel
{
private:
  double c_;
  double g_;
  double vel_m_;
  double tilt_rad_;
public:
  BebopVelocityTiltModel(const double& c, const double& g)
    : c_(c), g_(g), vel_m_(0.0), tilt_rad_(0.0)
  {}

  BebopVelocityTiltModel(const double &c, const double &g,
                         const double &vel_init_m, const double& tilt_init_rad)
    : c_(c), g_(g), vel_m_(vel_init_m), tilt_rad_(tilt_init_rad)
  {}

  inline double GetVel() const {return vel_m_;}
  inline double GetTilt() const {return tilt_rad_;}

  void Reset(const double &vel_init_m, const double& tilt_init_rad)
  {
    vel_m_ = vel_init_m;
    tilt_rad_ = tilt_init_rad;
  }

  void Reset()
  {
    Reset(0.0, 0.0);
  }

  /* Simulate the dynamic system for one time step dt
   * from current tilt angle and current velocity
   * */
  inline void Simulate(const double& dt_s)
  {
    assert( (dt_s > 0.0) && (fabs(dt_s) > 1e-6) );
    const double dv = (-c_ * vel_m_) + (g_ * tan(tilt_rad_));
    vel_m_ += (dv * dt_s);
  }

  /* Simulate the dynamic system assuming a fixed input (tilt_rad_)
   * for t seconds with dt steps
   *
   * Instead of discrizing the cont-time model, we perform
   * peice-wise integeration of the first order system
   *
   * */
  void Simulate(const double& duration_s, const double& dt_s)
  {
    if (duration_s < 0.0)
    {
      throw std::runtime_error("Invalid duration" + boost::lexical_cast<std::string>(duration_s));
    }
    double t = 0.0;
    while (t < duration_s)
    {
      Simulate(dt_s);
      t += dt_s;
    }
  }

  /* Overloaded function, also sets the initial conditions */
  void Simulate(const double &duration_s, const double &dt_s,
                const double &vel_init_m, const double& tilt_init_rad)
  {
    Reset(vel_init_m, tilt_init_rad);
    Simulate(duration_s, dt_s);
  }
};

const std::string& vicon_target_topic = "vicon/ar_box/ar_box";
const std::string& vicon_bebop_topic = "vicon/bebop_blue/bebop_blue";

const double height_target_m = 0.5;
const double distground_target_m = 0.75;
double sum_error = 0.0;

bool cinfo_recv;
ros::Time roi_recv_time(0);
ros::Time bebop_recv_time(0);
double d_truth = -1.0;
double cam_tilt_rad = 0.0;

double fov_x = 0.0;
double fov_y = 0.0;
image_geometry::PinholeCameraModel cam_model;
cv_bridge::CvImagePtr cv_img_ptr;
sensor_msgs::RegionOfInterest roi;

bebop_hri::debug debug_msg;
ros::Publisher debug_pub;

bool servo_inited = false;
vpServo task;
vpFeaturePoint pd[4];
vpFeaturePoint p[4];
vpImage<vpRGBa> I;
vpDisplayOpenCV display;

//vpFeaturePoint s_x, s_xd;
//vpFeatureDepth s_Z, s_Zd;
vpCameraParameters cam;
double depth;
double Z, Zd;
double lambda;
double t_start_loop;
double tinit;
vpColVector v;
vpColVector v_beb;
vpColVector vi;
double mu;
vpAdaptiveGain lambda_adapt;

geometry_msgs::Twist cmd_vel;
ros::Publisher cmd_vel_pub;

image_transport::Publisher debug_img_pub;

// Velocity controller
double beb_roll_rad;
double beb_pitch_rad;
double beb_yaw_rad;
double beb_yaw_ref_rad;
double beb_att_m;
bool beb_param_recv;
double beb_maxtilt_rad;
double beb_max_speed_vert_m;
double beb_max_speed_rot_rad;
double beb_vx_m;
double beb_vy_m;
double beb_vz_m;
double beb_vyaw_rad;
control_toolbox::Pid pid_vx;
control_toolbox::Pid pid_vy;
control_toolbox::Pid pid_yaw;

#define GRAV_CST 9.81 // g

boost::shared_ptr<BebopVelocityTiltModel> velx_model; // pitch
boost::shared_ptr<BebopVelocityTiltModel> vely_model; // roll

ros::Time last_pred_time;
const double time_delay_s = 0.262137; // seconds
const double Cx = -0.576335778073963;
const double Cy = -0.584975281133000;
double beb_vx_pred_m = 0.0;
double beb_vy_pred_m = 0.0;


//class ASyncSub
//{
//private:
//  typedef boost::function<void (const boost::shared_ptr<T const>& data)> callback_t;

//  ros::NodeHandle nh;
//  bool active_;
//  ros::Time last_updated_;
//  std::string topic_;
//  std::size_t queue_size_;
//  callback_t user_callback_;
//  ros::Subscriber sub_;
//  boost::shared_ptr<T const> msg_cptr_;
//  mutable boost::mutex mutex_;

//  void cb(const boost::shared_ptr<T const> &msg_cptr)
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    active_ = true;
//    last_updated_ = ros::Time::now();
//    msg_cptr_ = msg_cptr;
//    if (user_callback_) user_callback_(msg_cptr_);
//  }

//public:
//  ASyncSub(ros::NodeHandle& nh,
//           const std::string& topic,
//           const std::size_t queue_size,
//           callback_t user_callback = 0)
//    : nh(nh), active_(false), last_updated_(0), topic_(topic), user_callback_(user_callback)
//  {
//    sub_ = nh.subscribe<T>(topic_, queue_size, boost::bind(&ASyncSub<T>::cb, this, _1));
//  }

//  T GetMsgCopy() const
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    return *msg_cptr_;
//  }

//  // Not thread safe
//  const boost::shared_ptr<T const>& GetMsgConstPtr() const
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    return msg_cptr_;
//  }

//  // T operator ()() const {return GetMsgCopy();}

//  // Not thread safe?
//  const boost::shared_ptr<T const>& operator()() const
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    return GetMsgConstPtr();
//  }

//  void Deactivate()
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    active_ = false;
//  }

//  void DeactivateIfOlderThan(const double seconds)
//  {
//    if (!IsActive()) return;
//    if (GetFreshness().toSec() > seconds)
//    {
//      ROS_WARN("Information on topic (%s) is older than (%4.2lf) seconds. Deactivating.", topic_.c_str(), seconds);
//      Deactivate();
//    }
//  }

//  bool IsActive() const
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    return active_;
//  }

//  const ros::Time GetLastUpdated() const
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    return last_updated_;
//  }

//  const ros::Duration GetFreshness() const
//  {
//    boost::lock_guard<boost::mutex> lock(mutex_);
//    return ros::Time::now() - last_updated_;
//  }
//};

void ResetCmdVel(geometry_msgs::Twist& v)
{
  v.linear.x = 0.0;
  v.linear.y = 0.0;
  v.linear.z = 0.0;
  v.angular.x = 0.0;
  v.angular.y = 0.0;
  v.angular.z = 0.0;
}

template<typename T>
bool GetParam(const::std::string& key, T& val)
{
  if (ros::param::get(key, val))
  {
    ROS_INFO_STREAM(">>>>>>>>> " << key << " : " << val);
    return true;
  }
  ROS_WARN_STREAM("Key " << key << " not found in ");
  return false;
}

void CameraOrientationCallback(const bebop_msgs::Ardrone3CameraStateOrientationConstPtr& cam_ori_ptr)
{
  cam_tilt_rad = -angles::from_degrees(cam_ori_ptr->tilt);
}

void CameraCallback(const sensor_msgs::ImageConstPtr& img_ptr, const sensor_msgs::CameraInfoConstPtr& cinfo_msg_ptr)
{

  cam_model.fromCameraInfo(cinfo_msg_ptr);
  fov_x = 2.0 * atan2(cam_model.fullResolution().width / 2.0, cam_model.fx());
  fov_y = 2.0 * atan2(cam_model.fullResolution().height/ 2.0, cam_model.fy());

  ROS_INFO_STREAM_ONCE("FOV " << angles::to_degrees(fov_x) << " " << angles::to_degrees(fov_y));
  cinfo_recv = true;

  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
    for (uint32_t i = 0; i < 4; i++)
    {
      cv::Point2d proj_curr = cam_model.project3dToPixel(cv::Point3d(p[i].get_x(), p[i].get_y(), 1.0));
      cv::Point2d proj_d = cam_model.project3dToPixel(cv::Point3d(pd[i].get_x(), pd[i].get_y(), 1.0));
      cv::circle(cv_ptr->image, proj_curr, 2, CV_RGB(255, 0, 0), -1);
      cv::circle(cv_ptr->image, proj_d, 2, CV_RGB(0, 255, 0), -1);
    }
//    cv::Point2d proj_curr = cam_model.project3dToPixel(cv::Point3d(s_x.get_x(), s_x.get_y(), s_x.get_Z()));
//    cv::Point2d proj_d = cam_model.project3dToPixel(cv::Point3d(s_xd.get_x(), s_xd.get_y(), s_xd.get_Z()));
//    cv::circle(cv_ptr->image, proj_curr, 5, CV_RGB(255, 0, 0), -1);
//    cv::circle(cv_ptr->image, proj_d, 5, CV_RGB(0, 255, 0), -1);
    cv::rectangle(cv_ptr->image, cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height),
                  CV_RGB(0, 0, 255), 2);
    debug_img_pub.publish(cv_ptr->toImageMsg());

    I = visp_bridge::toVispImageRGBa(*img_ptr);
//    if (servo_inited)
//    {
//      vpDisplay::display(I);
//      vpDisplay::flush(I);
//    }

//    vpImage<vpRGBa> vp_img;
//    vpImageConvert::convert(cv_ptr->image, vp_img);
  }
  catch (cv::Exception &e)
  {
    ROS_ERROR_STREAM("E: " << e.what());
  }

  if ((!servo_inited) && ((ros::Time::now() - roi_recv_time).toSec() > 1.0))
  {
    cam = visp_bridge::toVispCameraParameters(*cinfo_msg_ptr);
    cam.printParameters();
    lambda_adapt.initStandard(4.0, 0.4, 40.0);

    // TODO

    vpFeatureBuilder::create(p[0], cam, vpImagePoint(roi.y_offset, roi.x_offset));
    vpFeatureBuilder::create(p[1], cam, vpImagePoint(roi.y_offset, roi.x_offset + roi.width));
    vpFeatureBuilder::create(p[2], cam, vpImagePoint(roi.y_offset + roi.height, roi.x_offset + roi.width));
    vpFeatureBuilder::create(p[3], cam, vpImagePoint(roi.y_offset + roi.height, roi.x_offset));

    vpPoint point[4];
    point[0].setWorldCoordinates(-0.25, -0.25, 0);
    point[1].setWorldCoordinates( 0.25, -0.25, 0);
    point[2].setWorldCoordinates( 0.25,  0.25, 0);
    point[3].setWorldCoordinates(-0.25,  0.25, 0);

    vpHomogeneousMatrix cMo;
    vpTranslationVector cto(0, 0, depth);
    vpRxyzVector cro(vpMath::rad(0.0), vpMath::rad(0.0), vpMath::rad(0.0));
    vpRotationMatrix cRo(cro);
    cMo.buildFrom(cto, cRo);

    for (uint32_t i = 0; i < 4; i++)
    {
      vpColVector cP, p_img;
      point[i].changeFrame(cMo, cP);
      point[i].projection(cP, p_img);

      pd[i].set_xyZ(p_img[0], p_img[1], cP[2]);
    }

    for (uint32_t i = 0; i < 4; i++)
    {
      task.addFeature(p[i], pd[i]);
    }

    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::MEAN, vpServo::PSEUDO_INVERSE);
    //task.setLambda(lambda_adapt);
    task.setLambda(lambda);
    task.print();

//    display.init(I, 640, 480, "image");
//    vpDisplay::display(I);
//    vpDisplay::flush(I);


//    vpFeatureBuilder::create(s_x, cam, vpImagePoint(cam_model.fullResolution().height/ 2.0,
//                                                    cam_model.fullResolution().width / 2.0));

//    vpFeatureBuilder::create(s_xd, cam, vpImagePoint(cam_model.fullResolution().height/ 2.0,
//                                                    cam_model.fullResolution().width / 2.0));

//    s_x.set_Z(1.0);
//    s_xd.set_Z(1.0);
    //s_xd.set_Z(Zd);

//    task.addFeature(s_x, s_xd);

//    s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, 0);
//    s_Zd.buildFrom(s_x.get_x(), s_x.get_y(), Zd, 0);

//    task.addFeature(s_Z, s_Zd);
    servo_inited = true;
  }

  if (!servo_inited)
  {
    ROS_WARN("Wating for first roi ...");
  }

}

void BebopSyncCallback(const bebop_msgs::Ardrone3PilotingStateAltitudeChangedConstPtr& alt_ptr,
                       const bebop_msgs::Ardrone3PilotingStateAttitudeChangedConstPtr& att_ptr,
                       const bebop_msgs::Ardrone3PilotingStateSpeedChangedConstPtr& speed_ptr)
{
  if (!cinfo_recv) return;
  bebop_recv_time = ros::Time::now();

  if (!beb_param_recv)
  {
    // This is sketchy, I need to find a way to get these params
    if (!GetParam("/bebop/bebop_nodelet/PilotingSettingsMaxTiltCurrent", beb_maxtilt_rad))
    {
      return;
    }
    beb_maxtilt_rad = angles::from_degrees(beb_maxtilt_rad);
    if (!GetParam("/bebop/bebop_nodelet/SpeedSettingsMaxVerticalSpeedCurrent", beb_max_speed_vert_m))
    {
      return;
    }
    if (!GetParam("/bebop/bebop_nodelet/SpeedSettingsMaxRotationSpeedCurrent", beb_max_speed_rot_rad))
    {
      return;
    }
    beb_max_speed_rot_rad = angles::from_degrees(beb_max_speed_rot_rad);

    // use first yaw as ref point
    beb_yaw_ref_rad = -att_ptr->yaw;
    beb_param_recv = true;
  }

  beb_roll_rad = att_ptr->roll;
  beb_pitch_rad = -att_ptr->pitch;
  beb_yaw_rad = -att_ptr->yaw;
  beb_att_m = alt_ptr->altitude;
  ROS_INFO_STREAM("Current Bebop State: RPY & ALT: "
                  << angles::to_degrees(beb_roll_rad) << " "
                  << angles::to_degrees(beb_pitch_rad) << " "
                  << angles::to_degrees(beb_yaw_rad) << " "
                  << beb_att_m << " ");

  // Bebop Speed is in world coordinates (ESD coords (we convert it to ENU))
  // yaw is already converted (with respect to magnetic north)
  const double vx_enu = speed_ptr->speedX;
  const double vy_enu = -speed_ptr->speedY;
  const double vz_enu = -speed_ptr->speedZ;

  beb_vx_m = cos(beb_yaw_rad) * vx_enu + sin(beb_yaw_rad) * vy_enu;
  beb_vy_m = -sin(beb_yaw_rad) * vx_enu + cos(beb_yaw_rad) * vy_enu;

  ROS_INFO_STREAM("Current Bebop Velcoities: XYZ: "
                  << beb_vx_m << " "
                  << beb_vy_m << " "
                  << beb_vz_m << " ");
  beb_vyaw_rad = 0.0; // this is unknown


  velx_model->Simulate(time_delay_s, 0.05, beb_vx_m, beb_pitch_rad);
  vely_model->Simulate(time_delay_s, 0.05, beb_vy_m, beb_roll_rad);

  ROS_INFO_STREAM("Simulated Bebop Velocities: XY: "
                  << velx_model->GetVel() << " " << vely_model->GetVel());
  ROS_WARN_STREAM("Last predicted bebop vels: XY:"
                  << beb_vx_pred_m << " " << beb_vy_pred_m);

  beb_vx_pred_m = beb_vx_m;
  beb_vy_pred_m = beb_vy_m;
}

void ViconSyncCallback(const geometry_msgs::TransformStampedConstPtr& target_msg_ptr,
                       const geometry_msgs::TransformStampedConstPtr& bebop_msg_ptr)
{
  tf2::Transform target, bebop;
  tf2::fromMsg(target_msg_ptr->transform, target);
  tf2::fromMsg(bebop_msg_ptr->transform, bebop);

  d_truth = target.getOrigin().distance(bebop.getOrigin());
}


//void CameraCallback(const sensor_msgs::ImageConstPtr& img_msg_ptr,
//                    const sensor_msgs::CameraInfoConstPtr& cinfo_msg_ptr)
//{
//  try
//  {
//    cv_img_ptr = cv_bridge::toCvCopy(img_msg_ptr, "bgr8");
//    cv::rectangle(cv_img_ptr->image, cv::Rect(roi.x_offset,
//                                              roi.y_offset,
//                                              roi.width,
//                                              roi.height),
//                  CV_RGB(255, 0, 0), 2);

//    cv::imshow("view", cv_img_ptr->image);
//    cv::waitKey(5);

//  }
//  catch (cv_bridge::Exception& e)
//  {
//    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg_ptr->encoding.c_str());
//  }
//}

void RoiCallback(const sensor_msgs::RegionOfInterestConstPtr& roi_msg_ptr)
{
  roi = *roi_msg_ptr;
  roi_recv_time = ros::Time::now();
}


int main(int argc, char* argv[])
{
  /* Visual Servo */
  depth = 2.5;
  lambda = 0.4;
  Z = Zd = depth;
  v.resize(6);
  vi.resize(6);
  v_beb.resize(6);
  v = 0;
  vi = 0;
  mu = 4;
  t_start_loop = 0.0;
  tinit = 0.0;
  /* ---- */
  cinfo_recv = false;
  beb_param_recv = false;


  ros::init(argc, argv, "depth_eval_node");


  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

//  image_transport::Subscriber image_sub = it.subscribe("bebop/image_raw", 1, ImageCallback);
  message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateAltitudeChanged> bebop_alt_sub(nh, "bebop/states/ardrone3/PilotingState/AltitudeChanged", 1);
  message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateAttitudeChanged> bebop_att_sub(nh, "bebop/states/ardrone3/PilotingState/AttitudeChanged", 1);
  message_filters::Subscriber<bebop_msgs::Ardrone3PilotingStateSpeedChanged> bebop_speed_sub(nh, "bebop/states/ardrone3/PilotingState/SpeedChanged", 1);

  message_filters::Subscriber<geometry_msgs::TransformStamped> vicon_target_sub(nh, vicon_target_topic, 1);
  message_filters::Subscriber<geometry_msgs::TransformStamped> vicon_bebop_sub(nh, vicon_bebop_topic, 1);

//  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "bebop/image_raw", 1);
//  message_filters::Subscriber<sensor_msgs::RegionOfInterest> roi_sub(nh, "roi", 1);

  typedef message_filters::sync_policies::ApproximateTime<
      bebop_msgs::Ardrone3PilotingStateAltitudeChanged,
      bebop_msgs::Ardrone3PilotingStateAttitudeChanged,
      bebop_msgs::Ardrone3PilotingStateSpeedChanged> BebopSyncPolicy_t;

  typedef message_filters::sync_policies::ApproximateTime<
      geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> ViconSyncPolicy_t;

  message_filters::Synchronizer<BebopSyncPolicy_t>
      bebop_sync(BebopSyncPolicy_t(10), bebop_alt_sub, bebop_att_sub, bebop_speed_sub);

  message_filters::Synchronizer<ViconSyncPolicy_t>
      vicon_sync(ViconSyncPolicy_t(10), vicon_target_sub, vicon_bebop_sub);

  bebop_sync.registerCallback(boost::bind(&BebopSyncCallback, _1, _2, _3));
  vicon_sync.registerCallback(boost::bind(&ViconSyncCallback, _1, _2));

//  image_transport::Subscriber camera_sub = it.subscribe("bebop", 1, CameraCallback);
  //ros::Subscriber caminfo_sub = nh.subscribe("bebop/camera_info", 10, CameraInfoCallback);
  image_transport::CameraSubscriber camera_sub = it.subscribeCamera("bebop/image_raw", 1, CameraCallback);
  debug_img_pub = it.advertise("vs/debug_img", 10);

  // ROI is not syncable at the moment since there is no stamp
  ros::Subscriber roi_sub = nh.subscribe("roi", 10, RoiCallback);

  // Camera pitch changes rarely, no need to sync
  ros::Subscriber cam_orientation_sub = nh.subscribe("bebop/states/ARDrone3/CameraState/Orientation", 10, CameraOrientationCallback);

  debug_pub = nh.advertise<bebop_hri::debug>("vs/debug", 10, true);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("bebop_hri/cmd_vel", 10);

//  cv::namedWindow("view");
//  cv::startWindowThread();

  ResetCmdVel(cmd_vel);

  /* vel ctrl */
//  pid_vx.initPid(1.043, 2.1454, 0.0, 10.0, -10.0, nh);
//  pid_vy.initPid(-1.043, -2.1551, 0.0, 10.0, -10.0, nh);

  ros::NodeHandle nh_pid_vx(nh, "pid_forward");
  ros::NodeHandle nh_pid_vy(nh, "pid_left");
  ros::NodeHandle nh_pid_yaw(nh, "pid_yaw");

  nh_pid_vx.setParam("p", 0.15);
  nh_pid_vx.setParam("i", 0.4);
  nh_pid_vx.setParam("d", 0.0);
  nh_pid_vx.setParam("i_clamp", 0.02);

  nh_pid_vy.setParam("p", 0.15);
  nh_pid_vy.setParam("i", 0.4);
  nh_pid_vy.setParam("d", 0.0);
  nh_pid_vy.setParam("i_clamp", 0.02);

  nh_pid_yaw.setParam("p", 0.5);

  pid_vx.init(nh_pid_vx);
  pid_vy.init(nh_pid_vy);
  pid_yaw.init(nh_pid_yaw);

//  pid_vx.initPid(1.043, 0.0, 0.0, 10.0, -10.0, nh);
//  pid_vy.initPid(-0.5, 0.0, 0.0, 10.0, -10.0, nh);

  velx_model = boost::make_shared<BebopVelocityTiltModel>(Cx, GRAV_CST);
  vely_model = boost::make_shared<BebopVelocityTiltModel>(Cy, -GRAV_CST);

  ros::Rate rate(30.0);
  ros::Time pid_last_time = ros::Time::now();
  while (ros::ok())
  {
    if ((ros::Time::now() - roi_recv_time).toSec() > 1.0)
    {
      ROS_WARN("ROI is too old, skipping");
      ResetCmdVel(cmd_vel);
    }
    else if ((ros::Time::now() - bebop_recv_time).toSec() > 1.0)
    {
      ROS_WARN("Bebop is too old, skipping");
      ResetCmdVel(cmd_vel);
    }
    else if (roi.height < 5 || roi.height > cam_model.fullResolution().height)
    {
      ROS_WARN_STREAM("ROI is too small or invalid, skipping " << roi.height);
      ResetCmdVel(cmd_vel);
    }
    else
    {
      const double im_width_px = static_cast<double>(cam_model.fullResolution().width);
      const double im_height_px = static_cast<double>(cam_model.fullResolution().height);
      //double pitch_rad = -att_ptr->pitch; // Nose down increase pitch
      double sigma_1 = static_cast<double>(roi.y_offset) / im_height_px * fov_y;
      double sigma_2 = (im_height_px - static_cast<double>(roi.y_offset + roi.height)) / im_height_px * fov_y;

//      double alt_m = alt_ptr->altitude;
      double roi_height_px = roi.height;
      double beta_rad = (roi_height_px / im_height_px) * fov_y;

      double d1_m = height_target_m * sin(M_PI_2 - fov_y/2.0 + sigma_2 - cam_tilt_rad) / sin(beta_rad);
      double z1_m = d1_m * sin(M_PI_2 + fov_y/2.0  - cam_tilt_rad - sigma_1);

//      double d2_m = (alt_m + 0.1  - (height_target_m + distground_target_m)) / sin(fov_y/2.0 + cam_tilt_rad - sigma_2 - beta_rad);

//      sum_error += (d1_m - d_truth) * (d1_m - d_truth);
      try
      {
    //    t_start_loop = vpTime::measureTimeMs();
    //    s_x.set_xyZ((roi.x_offset + roi.width / 2.0) - im_width_px/2.0 ,
    //                (roi.y_offset + roi.height / 2.0) - im_height_px/2.0,
    //                Z);

    //    Z = d1_m;
        // vpImagePoint i:y j:x
    //    vpFeatureBuilder::create(s_x, cam,
    //                             vpImagePoint(
    //                               (roi.y_offset + roi.height / 2.0),
    //                               (roi.x_offset + roi.width  / 2.0)));

    //    s_x.set_Z(Z);
    //    s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z/Zd));

        vpFeatureBuilder::create(p[0], cam, vpImagePoint(roi.y_offset, roi.x_offset));
        vpFeatureBuilder::create(p[1], cam, vpImagePoint(roi.y_offset, roi.x_offset + roi.width));
        vpFeatureBuilder::create(p[2], cam, vpImagePoint(roi.y_offset + roi.height, roi.x_offset + roi.width));
        vpFeatureBuilder::create(p[3], cam, vpImagePoint(roi.y_offset + roi.height, roi.x_offset));

        for (uint32_t i = 0; i < 4; i++)
        {
           p[i].set_Z(z1_m);
        }

        v = task.computeControlLaw();
//        vpDisplay::display(I);
//        vpServoDisplay::display(task,cam,I) ;
//        vpDisplay::flush(I);

        task.print();
        ROS_WARN_STREAM("V_SERVO\n" << v);


//        ROS_ERROR_STREAM("Control Error: " << task.getError());


        // Servo:           z: forward, x: right, y: down
        // Maps to bebop:   z: x_b    , x: -y_b , y: -z_b

        v_beb[3] = 0.0;
        v_beb[4] = 0.0;

        // lin.z
        v_beb[2] = -CLAMP(v[1], -0.8, 0.8);
        if (fabs(v_beb[2]) < 0.05) v_beb[2] = 0.0;
        v_beb[2] /= beb_max_speed_vert_m;

        // ang.z
        v_beb[5] = -CLAMP(v[4], -0.2, 0.2);
        if (fabs(v_beb[5]) < 0.01) v_beb[5] = 0.0;
        v_beb[5] /= beb_max_speed_rot_rad;

        // Use PID for vel control
        // The outout of the model is pitch and roll angles
        const ros::Time& t_now = ros::Time::now();
        const ros::Duration& dt = t_now - pid_last_time;
        // err = desired - current
        // problem here is beb_v?_m 's update rate is 5 Hz
        // TODO: Use prediction based on max roll

//        // Use prediction

//        const double d_beb_vx_m = dt.toSec() * (-Cx * beb_vx_pred_m + GRAV_CST * tan(beb_pitch_rad));
//        const double d_beb_vy_m = dt.toSec() * (-Cy * beb_vy_pred_m - GRAV_CST * tan(beb_roll_rad));

//        if ((t_now - bebop_recv_time).toSec() > dt)
//        {
//          beb_vx_pred_m += d_beb_vx_m * (1.0 + time_delay_s);
//          beb_vx_pred_m += d_beb_vy_m * (1.0 + time_delay_s);
//        }

//        v_beb[0] = pid_vx.computeCommand(v[2] - beb_vx_m, dt) / beb_maxtilt_rad;
//        v_beb[1] = pid_vy.computeCommand(-v[0] - beb_vy_m, dt) / beb_maxtilt_rad;

        // max linear vel of 1 m/s
        // min 0.1 m/s
        v[2] = CLAMP(v[2], -1.0, 1.0);
        if (fabs(v[2]) < 0.1) v[2] = 0.0;

        v[0] = CLAMP(v[0], -1.0, 1.0);
        if (fabs(v[0]) < 0.1) v[0] = 0.0;

        const double pitch_ref = pid_vx.computeCommand(v[2] - velx_model->GetVel(), dt);
        const double roll_ref = pid_vy.computeCommand(-v[0] - vely_model->GetVel(), dt);

        v_beb[0] =  pitch_ref / beb_maxtilt_rad;
        v_beb[1] =  roll_ref / beb_maxtilt_rad;

        const double vyaw_ref = pid_yaw.computeCommand(angles::normalize_angle(beb_yaw_ref_rad - beb_yaw_rad), dt);

        // ang.z (fixed ref tracking, not from visual servo)
        v_beb[5] = CLAMP(vyaw_ref, -0.4, 0.4);
        if (fabs(v_beb[5]) < 0.01) v_beb[5] = 0.0;
        v_beb[5] /= beb_max_speed_rot_rad;
        pid_last_time = t_now;

        // Simulate Bebop's velocity given the reference tilt to update (predict) feedback
        // this to compensate for low frequency velocity feedback
        velx_model->Reset(velx_model->GetVel(), CLAMP(pitch_ref, -beb_maxtilt_rad, beb_maxtilt_rad));
        velx_model->Simulate(dt.toSec());

        vely_model->Reset(vely_model->GetVel(), CLAMP(roll_ref, -beb_maxtilt_rad, beb_maxtilt_rad));
        vely_model->Simulate(dt.toSec());


        ROS_WARN_STREAM("V_BEB (RAW)\n" << v_beb);

        // This is for safety only, the max-vel should have already been applied
        vpColVector v_max(6);
        v_max[0] = 1.0;
        v_max[1] = 1.0;
        v_max[2] = 1.0;
        v_max[3] = 1.0;
        v_max[4] = 1.0;
        v_max[5] = 1.0;

        vpColVector v_sat = vpRobot::saturateVelocities(v_beb, v_max);

        for (uint32_t i = 0; i < 6; i++)
        {
          if (fabs(v_sat[i]) < 0.01) v_sat[i] = 0.0;
        }

        cmd_vel.linear.x  = v_sat[0];  // normalized pitch angle
        cmd_vel.linear.y  = v_sat[1];  // normalized roll angle
        cmd_vel.linear.z  = v_sat[2];  // normalized desired vertical velocity
        cmd_vel.angular.z = v_sat[5];  // normalized desired rotation verlocity

        if ((task.getError().euclideanNorm() / static_cast<double>(task.getError().getCols()) < 0.5) &&
            (fabs(z1_m - depth) < 0.5))
        {
          ROS_WARN("################################## AT TARGET!");
          ResetCmdVel(cmd_vel);
        }

        ROS_ERROR_STREAM("CMD_VEL for: " << cmd_vel.linear.x <<
                         " left: " << cmd_vel.linear.y <<
                         " up: " << cmd_vel.linear.z <<
                         " cw: " << cmd_vel.angular.z);

      }
      catch (const vpException& e)
      {
        ResetCmdVel(cmd_vel);
        ROS_ERROR_STREAM("VP Exception: " << e.what());
      }

      /* ---- */
      ROS_INFO_STREAM("Sanity: " << angles::to_degrees(sigma_1 + sigma_2 + beta_rad));
//      ROS_INFO_STREAM("Altitude (m): " << alt_m);
//      ROS_INFO_STREAM("Pitch (deg): " << angles::to_degrees(pitch_rad));
      ROS_INFO_STREAM("Camera Tilt (deg): " << angles::to_degrees(cam_tilt_rad));
      ROS_INFO_STREAM("sig1 (deg): " << angles::to_degrees(sigma_1));
      ROS_INFO_STREAM("sig2 (deg): " << angles::to_degrees(sigma_2));
      ROS_INFO_STREAM("Beta (deg): " << angles::to_degrees(beta_rad));
      ROS_INFO_STREAM("ROI height (px): " << roi_height_px);
      ROS_WARN_STREAM("D1  (m): " << z1_m);
//      ROS_WARN_STREAM("D2  (m): " << d2_m);
      ROS_WARN_STREAM("DGT (m): " << d_truth);
      ROS_WARN_STREAM("RSE (m):" << sqrt(sum_error));

      debug_msg.header.stamp = ros::Time::now();
      debug_msg.header.frame_id = "mani";
      debug_msg.bb_height = roi_height_px;
//      debug_msg.bebop_alt = alt_m;
      debug_msg.d_groundtruth = d_truth;
      debug_msg.cam_tilt = cam_tilt_rad;
      debug_msg.beta = beta_rad;
      debug_msg.sigma_1 = sigma_1;
      debug_msg.sigma_2 = sigma_2;
      debug_msg.d_raw_bb = z1_m;
//      debug_msg.d_raw_alt = d2_m;
      debug_msg.target_height = height_target_m;
      debug_msg.target_grounddist = distground_target_m;

      debug_msg.vest_x = velx_model->GetVel();
      debug_msg.vest_y = vely_model->GetVel();
      debug_pub.publish(debug_msg);
    }

    cmd_vel_pub.publish(cmd_vel);
    ros::spinOnce();
    if (!rate.sleep()) ROS_WARN("Loop rate missed!");
  }

//  cv::destroyAllWindows();
  return 0;
}
