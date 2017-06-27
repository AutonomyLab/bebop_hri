#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>

#include "bebop_hri/util.h"
#include "bebop_hri/behavior_tools.h"
#include "bebop_hri/behavior.h"
#include "obzerver_ros/Init.h"

namespace bebop_hri
{

BebopBehaviorNode::BebopBehaviorNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : nh_(nh),
    priv_nh_(priv_nh),
    sub_joy_(nh_, "joy", 10),
    sub_manual_roi_(nh_, "manual_roi", 1),
    sub_periodic_tracks_(nh_, "obzerver/tracks/periodic", 10),
    sub_all_tracks_(nh_, "obzerver/tracks/all", 10),
    sub_visual_tracker_track_(nh_, "visual_tracker_track", 1),
    sub_bebop_att_(nh_, "bebop/states/ardrone3/PilotingState/AttitudeChanged", 10),
    sub_bebop_alt_(nh_, "bebop/states/ardrone3/PilotingState/AltitudeChanged", 10),
    sub_camera_info_(nh_, "bebop/camera_info", 1),
    sub_human_(nh, "human/human", 10),
    pub_cftld_tracker_reset_(nh_.advertise<std_msgs::Empty>("visual_tracker_reset", 1, true)),
    pub_cftld_tracker_init_(nh_.advertise<sensor_msgs::RegionOfInterest>("visual_tracker_init", 1, true)),
    pub_visual_servo_enable_(nh_.advertise<std_msgs::Bool>("visual_servo_enable", 1, true)),
    pub_visual_servo_target_(nh_.advertise<bebop_vservo::Target>("visual_servo_target", 1, true)),
    pub_obzerver_init_(nh_.advertise<obzerver_ros::Init>("obzerver/enable", 1, true)),
    pub_human_enable_(nh_.advertise<std_msgs::Bool>("human/enable", 1, true)),
    pub_bebop_camera_(nh_.advertise<geometry_msgs::Twist>("bebop/camera_control", 1, true)),
    pub_bebop_flip_(nh_.advertise<std_msgs::UInt8>("bebop/flip", 1, false)),
    pub_bebop_snapshot_(nh_.advertise<std_msgs::Empty>("bebop/snapshot", 1, false)),
    pub_bebop_abs_vel_(nh_.advertise<geometry_msgs::Twist>("abs_vel_ctrl/setpoint/cmd_vel", 10, false)),
    status_publisher_(nh_, "status"),
    bebop_mode_(constants::MODE_NUM),
    bebop_mode_prev_(constants::MODE_NUM),
    bebop_mode_prev_update_(constants::MODE_NUM),
    bebop_resume_mode_manual_(constants::MODE_IDLE),
    bebop_resume_mode_badvideo_(constants::MODE_IDLE),
    last_transition_time_(ros::Time::now()),
    promising_tracks(0),
    led_feedback_(nh_),
    gesture_curr_(constants::GESTURE_NONE),
    gesture_prev_(constants::GESTURE_NONE),
    flow_left_median_(0.0),
    flow_right_median_(0.0),
    gest_left_counter_(0.0),
    gest_right_counter_(0.0),
    gesture_both_counter_(0.0),
    desired_search_inited_(false)
{
  UpdateParams();
  Transition(static_cast<constants::bebop_mode_t>(param_init_mode_));
}

void BebopBehaviorNode::UpdateParams()
{
  util::get_param(priv_nh_, "update_freq", param_update_rate_, 10.0);
  util::get_param(priv_nh_, "initial_mode", param_init_mode_, 0);
  util::get_param(priv_nh_, "joy_override_buttion", param_joy_override_button_, 7);
  util::get_param(priv_nh_, "joy_override_timeout", param_joy_override_timeout_, 20.0);
  util::get_param(priv_nh_, "joy_reset_button", param_joy_reset_button_, 6);
  util::get_param(priv_nh_, "joy_setyaw_button", param_joy_setyaw_button_, 5);
  util::get_param(priv_nh_, "idle_timeout", param_idle_timeout_, 10.0);
  util::get_param(priv_nh_, "servo_desired_depth", param_servo_desired_depth_, 2.5);
  util::get_param(priv_nh_, "target_height", param_target_height_, 0.5);
  util::get_param(priv_nh_, "target_dist_ground", param_target_dist_ground_, 0.75);
  util::get_param(priv_nh_, "stale_video_timeout", param_stale_video_timeout_, 10.0);
  util::get_param(priv_nh_, "enable_camera_control", param_enable_camera_control_, true);
  util::get_param(priv_nh_, "max_camera_tilt", param_max_camera_tilt_deg_, 22.5);
  util::get_param(priv_nh_, "search_alt", param_search_alt_, 2.75);
  util::get_param(priv_nh_, "perform_search_action", param_perform_search_action_, false);

  util::get_param(priv_nh_, "flow_queue_size", param_flow_queue_size_, 15);
  util::get_param(priv_nh_, "flow_threshold", param_flow_threshold_, 6.0);
  util::get_param(priv_nh_, "flow_min_votes", param_flow_min_votes_, 15);

  ROS_ASSERT(param_search_alt_ > 0.5 && param_search_alt_ < 40.0);
}

void BebopBehaviorNode::ResetGestures()
{
  flow_left_vec_.clear();
  flow_right_vec_.clear();
  gesture_curr_ = constants::GESTURE_NONE;
  gesture_prev_ = constants::GESTURE_NONE;
  flow_left_median_ = 0.0;
  flow_right_median_ = 0.0;
  gest_left_counter_ = 0;
  gest_right_counter_ = 0;
  gesture_both_counter_ = 0;
}

void BebopBehaviorNode::ResetVisualTracker()
{
  pub_cftld_tracker_reset_.publish(msg_empty_);
}

void BebopBehaviorNode::Reset()
{
  ROS_WARN("[BEH] Behavior Reset");
  ResetVisualTracker();
  ToggleVisualServo(false);
  ToggleObzerver(false);
  ToggleAutonomyHuman(false);
  ResetGestures();
}

void BebopBehaviorNode::ToggleVisualServo(const bool enable)
{
  std_msgs::Bool bool_msg;
  bool_msg.data = enable;
  pub_visual_servo_enable_.publish(bool_msg);
}

void BebopBehaviorNode::ToggleObzerver(const bool enable,
                                       const uint32_t roi_min_height,
                                       const uint32_t roi_min_width,
                                       const uint32_t roi_max_height,
                                       const uint32_t roi_max_width)
{
  obzerver_ros::Init init_msg;
  init_msg.enable = enable;
  if (enable)
  {
    init_msg.min_roi.width = roi_min_width;
    init_msg.min_roi.height = roi_min_height;
    init_msg.max_roi.width = roi_max_width;
    init_msg.max_roi.height = roi_max_height;
  }
  pub_obzerver_init_.publish(init_msg);
}

void BebopBehaviorNode::ToggleAutonomyHuman(const bool enable)
{
  std_msgs::Bool bool_msg;
  bool_msg.data = enable;
  pub_human_enable_.publish(bool_msg);
}

void BebopBehaviorNode::MoveBebopCamera(const double &pan_deg, const double &tilt_deg)
{
  if (!param_enable_camera_control_)
  {
    ROS_ERROR_ONCE("[BEH] Camera control is disabled");
    return;
  }

  ROS_DEBUG_STREAM("[BEH] Request to move Bebop's camera to " << tilt_deg);
  geometry_msgs::Twist twist;
  twist.angular.y = tilt_deg;
  twist.angular.z = pan_deg;
  pub_bebop_camera_.publish(twist);
}

bool BebopBehaviorNode::SetDesiredYaw()
{
  if (sub_bebop_att_.IsActive())
  {
    desired_search_yaw_ = -(sub_bebop_att_()->yaw);
    desired_search_inited_ = true;
    ROS_WARN_STREAM("[BEH] Desired search yaw is set to: " << angles::to_degrees(desired_search_yaw_));
    return true;
  }
  ROS_ERROR("[BEH] SetDesiredYaw() failed.");
  return false;
}

void BebopBehaviorNode::BebopFlip(const uint8_t flip_type)
{
  return;
  ROS_WARN_STREAM("[BEH] Flip requested: " << flip_type);
  std_msgs::UInt8 ft_msg;
  // TODO: bound check
  ft_msg.data = flip_type;
  pub_bebop_flip_.publish(ft_msg);
}

void BebopBehaviorNode::BebopSnapshot()
{
  ROS_WARN("[BEH] Taking a snapshot ...");
  std_msgs::Empty empty_msg;
  pub_bebop_snapshot_.publish(empty_msg);
}

void BebopBehaviorNode::ControlBebopCamera()
{
  if (sub_bebop_alt_.IsActive())
  {
    const double& alt_target = param_target_height_/2.0 + param_target_dist_ground_;
    //const double& tilt = (sub_bebop_alt_()->altitude - alt_target) / (param_search_alt_ - alt_target);
    const double& tilt = pow((sub_bebop_alt_()->altitude - alt_target) / (param_search_alt_ - alt_target), 0.33);
    ROS_DEBUG_STREAM_THROTTLE(0.25, "[BEH] alt_target: " << alt_target
                             << " alt_bebop: " << sub_bebop_alt_()->altitude
                             << " tilt_rel: " << tilt);
    MoveBebopCamera(0.0, -CLAMP(tilt, 0.0, 1.0) * param_max_camera_tilt_deg_);
  }
}

void BebopBehaviorNode::PerformSearchAction()
{
  // The search behavior (absolute mode)
  // Prioritize yaw if the error is big
  geometry_msgs::Twist search_twist_;
  bool yaw_priority = true;
  double yaw_err;
  if (sub_bebop_att_.IsActive() && sub_bebop_alt_.IsActive())
  {
    yaw_err = angles::shortest_angular_distance(-sub_bebop_att_()->yaw, desired_search_yaw_);
    if (fabs(yaw_err) < angles::from_degrees(15)) yaw_priority = false;
  }

//  ROS_WARN_STREAM("[BEH] Current Yaw: " << angles::to_degrees(-sub_bebop_att_()->yaw) <<
//                  " desired yaw: " << angles::to_degrees(desired_search_yaw_) << " yaw err" << angles::to_degrees(yaw_err));

  search_twist_.linear.x = 0.0;
  search_twist_.linear.y = 0.0;
  search_twist_.linear.z = (yaw_priority) ? sub_bebop_alt_()->altitude : param_search_alt_;
  search_twist_.angular.x = 0.0;
  search_twist_.angular.y = 0.0;
  search_twist_.angular.z = desired_search_yaw_;
  pub_bebop_abs_vel_.publish(search_twist_);
}


bool BebopBehaviorNode::GestureUpdate()
{
    ROS_ASSERT(flow_left_vec_.size() == flow_right_vec_.size());
    // TODO: Config
    if (flow_left_vec_.size() < param_flow_queue_size_) // Not ready yet
        return false;

    flow_left_median_ = util::median<double>(flow_left_vec_);
    flow_right_median_ = util::median<double>(flow_right_vec_);

    // TODO: config
    // gesture_filter_zero: 0.1
    // gesture_filter_threshold: 3.2
    bool left = ( flow_left_median_ > param_flow_threshold_) && ( flow_right_median_ < 0.1 );
    bool right = ( flow_right_median_ > param_flow_threshold_) && ( flow_left_median_ <  0.1 );
    bool both = ( flow_left_median_ > param_flow_threshold_) &&
        ( flow_right_median_ > param_flow_threshold_) &&
        ( fabs(flow_left_median_ - flow_right_median_) < 0.5 * param_flow_threshold_);

    gest_left_counter_ = left ? gest_left_counter_+1 : 0;
    gest_right_counter_ = right ? gest_right_counter_+1 : 0;
    gesture_both_counter_ = both ? gesture_both_counter_+1 :  0;

    // TODO: Config
    gesture_prev_ = gesture_curr_;
    gesture_curr_ = constants::GESTURE_NONE;
    if ((gest_left_counter_ > param_flow_min_votes_) && (gest_right_counter_ == 0)) gesture_curr_ = constants::GESTURE_LEFT_SWING;
    if ((gest_right_counter_ > param_flow_min_votes_) && (gest_left_counter_ == 0)) gesture_curr_ = constants::GESTURE_RIGHT_SWING;
    // High Priority
    if (gesture_both_counter_ > param_flow_min_votes_) gesture_curr_ = constants::GESTURE_BOTH_SWING;

    ROS_DEBUG_THROTTLE(0.25, "[BEH] Left: (%.3f, %2d) Right: (%.3f, %2d) Both: (%2d) G: %d ",
             flow_left_median_, (int) gest_left_counter_,  flow_right_median_, (int) gest_right_counter_, (int) gesture_both_counter_, gesture_curr_ );
    return true;
}


void BebopBehaviorNode::UpdateBehavior()
{
  const bool is_transition = (bebop_mode_ != bebop_mode_prev_update_);
  if (is_transition)
  {
    ROS_WARN("[BEH] State Transitioned (%s -> %s)",
             BEBOP_MODE_STR(bebop_mode_prev_).c_str(),
             BEBOP_MODE_STR(bebop_mode_).c_str());
    last_transition_time_ = ros::Time::now();
  }

  const ros::Duration mode_duration = ros::Time::now() - last_transition_time_;

  status_publisher_ << "Current State: " << constants::STR_BEBOP_MODE_MAP[bebop_mode_].c_str() << ", ";
  status_publisher_ << " Duration: " << mode_duration.toSec() << ", ";
  status_publisher_ << " Manual ROI: " << sub_manual_roi_.IsActive() << ", ";
  status_publisher_ << " Number of periodic tracks: " << static_cast<int>(sub_periodic_tracks_.IsActive() ? sub_periodic_tracks_()->tracks.size() : 0) << ", ";
  status_publisher_ << " Visual tracker: " <<
                       ((sub_visual_tracker_track_.IsActive() && sub_visual_tracker_track_()->status == cftld_ros::Track::STATUS_TRACKING) ? "Yes" : "No")
                       << ", ";
  status_publisher_ << " Human Num Faces: " << static_cast<int>(sub_human_.IsActive() ? sub_human_()->numFaces : 0) << ", ";
  status_publisher_ << " Human Face Score: " << static_cast<int>(sub_human_.IsActive() ? sub_human_()->faceScore : 0) << ", ";
  status_publisher_ << " Gesture: " << constants::STR_GESTURE_STATES_MAP[gesture_curr_] << ", ";

  ROS_DEBUG_THROTTLE(1 , "[BEH] %s", status_publisher_.GetBuffer().str().c_str());
  status_publisher_.Publish();

  bebop_mode_prev_update_ = bebop_mode_;

  // Deactivate all async subs if they are not fresh enough
  sub_joy_.DeactivateIfOlderThan(1.0);
  sub_manual_roi_.DeactivateIfOlderThan(1.0);
  sub_periodic_tracks_.DeactivateIfOlderThan(0.5);
  sub_all_tracks_.DeactivateIfOlderThan(0.5);
  sub_visual_tracker_track_.DeactivateIfOlderThan(2.0);
  sub_bebop_att_.DeactivateIfOlderThan(1.0);
  sub_bebop_alt_.DeactivateIfOlderThan(1.0);
  sub_human_.DeactivateIfOlderThan(1.0);
  // The tolerance on camera_info_sub is lower
  sub_camera_info_.DeactivateIfOlderThan(0.5);

  // Emergency behavior is implemented way down the pipeline, in cmd_vel_mux layer
  // regardless of the current mode, is joy_override_button is pressed, we will pause execution
  if ((bebop_mode_ != constants::MODE_MANUAL) &&
      sub_joy_.IsActive() &&
      sub_joy_()->buttons.at(param_joy_override_button_) &&
      !sub_joy_()->buttons.at(param_joy_reset_button_))
  {
    ROS_WARN("[BEH] Joystick override detected");
    bebop_resume_mode_manual_ = bebop_mode_;
    Transition(constants::MODE_MANUAL);
    return;
  }

  if ((bebop_mode_ != constants::MODE_MANUAL) &&
      (bebop_mode_ != constants::MODE_BAD_VIDEO) &&
      (bebop_mode_ != constants::MODE_IDLE) &&
      (false == sub_camera_info_.IsActive()))
  {
    ROS_ERROR("[BEH] Video feed is stale");
    bebop_resume_mode_badvideo_ = bebop_mode_;
    Transition(constants::MODE_BAD_VIDEO);
    return;
  }

  switch (bebop_mode_)
  {
  case constants::MODE_IDLE:
  {
    if (is_transition)
    {
      Reset();
      led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_FAST_BLINK, "green", "cyan", 0.6);
    }
    const bool manual_reset = (mode_duration.toSec() > 1.0) &&
        sub_joy_.IsActive() &&
        !sub_joy_()->buttons[param_joy_override_button_] &&
        sub_joy_()->buttons[param_joy_reset_button_];

    if ((mode_duration.toSec() > 8.0) && !desired_search_inited_)
    {
      SetDesiredYaw();
    }

    if ((mode_duration.toSec() > param_idle_timeout_) || (manual_reset))
    {
      Transition(static_cast<constants::bebop_mode_t>(param_init_mode_));

      // Avoid deadlock, if the user has not specified a default starting state,
      // perfrom searching
      if (bebop_mode_ == constants::MODE_IDLE)
      {
        Transition(constants::MODE_SEARCHING);
      }

      ROS_INFO_STREAM("[BEH] IDLE transition because of " << (manual_reset ? "Manual Reset" : "Timeout"));
      ROS_INFO_STREAM("[BEH] transitioning to initial state: " << BEBOP_MODE_STR(bebop_mode_));
    }
    break;
  }

  case constants::MODE_MANUAL:
  {
    if (is_transition)
    {
      led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_BLINK_CLEAR, "red", "yellow", 30);
    }
    if (mode_duration.toSec() > param_joy_override_timeout_ &&
        bebop_resume_mode_manual_ != constants::MODE_IDLE)
    {
      ROS_WARN_STREAM("[BEH] Time in manual mode exceeded the `joy_override_timeout` of " << param_joy_override_timeout_);
      ROS_WARN_STREAM("[BEH] Behavior will be reset after override is over");
      bebop_resume_mode_manual_ = constants::MODE_IDLE;
    }

    // In manual mode, pressing the reset button (while holding the override button),
    // will immediately reset to IDLE mode
    if (sub_joy_.IsActive() &&
        sub_joy_()->buttons[param_joy_override_button_] &&
        sub_joy_()->buttons[param_joy_reset_button_])
    {
      ROS_WARN_STREAM("[BEH] Manual joy reset requested");
      Transition(constants::MODE_IDLE);
      break;
    }

    if (sub_joy_.IsActive() &&
        sub_joy_()->buttons[param_joy_override_button_] &&
        sub_joy_()->buttons[param_joy_setyaw_button_])
    {
      ROS_WARN_STREAM("[BEH] Manual SetYaw Requested ...");
      SetDesiredYaw();
      break;
    }

    if (sub_joy_.IsActive() && !sub_joy_()->buttons[param_joy_override_button_])
    {
      ROS_WARN_STREAM("[BEH] Joystick override ended, going back to " << BEBOP_MODE_STR(bebop_resume_mode_manual_));
      Transition(bebop_resume_mode_manual_);
    }
    break;
  }

  case constants::MODE_BAD_VIDEO:
  {
    if (is_transition)
    {
      ROS_ERROR("[BEH] Video STALE mode");
      ToggleVisualServo(false);
      led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_FAST_BLINK, "red", "yellow", 1.2);
    }

    if (sub_camera_info_.GetFreshness().toSec() < 0.05)
    {
      ROS_WARN_STREAM("[BEH] Video is good again!");
      Transition(bebop_resume_mode_badvideo_);
      break;
    }

    if ((mode_duration.toSec() > param_stale_video_timeout_) &&
        (bebop_resume_mode_badvideo_ != constants::MODE_IDLE))
    {
      ROS_WARN_STREAM("[BEH] Time in video stale mode exceeded the `stale_video_timeout` of " << param_stale_video_timeout_);
      ROS_WARN_STREAM("[BEH] Behavior will be reset after video comes back online");
      bebop_resume_mode_badvideo_ = constants::MODE_IDLE;
    }

    break;
  }
  case constants::MODE_SEARCHING:
  {
    if (is_transition)
    {
      ROS_INFO("[BEH] Searching ...");
      led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_SEARCH_1, "green", "blue", 3);

      MoveBebopCamera(0.0, -param_max_camera_tilt_deg_);
      // Experimental
      if (sub_camera_info_.IsActive() && sub_bebop_alt_.IsActive())
      {
        cam_model_.fromCameraInfo(sub_camera_info_());
        const double fov_x_ = 2.0 * atan2(cam_model_.fullResolution().width / 2.0, cam_model_.fx());
        const double fov_y_ = 2.0 * atan2(cam_model_.fullResolution().height/ 2.0, cam_model_.fy());

        const double cam_tilt = angles::from_degrees(param_max_camera_tilt_deg_);
        const double alt = (param_perform_search_action_) ? param_search_alt_ : sub_bebop_alt_()->altitude;

        ROS_INFO_STREAM_ONCE("[BEH] FOV " << angles::to_degrees(fov_x_) << " " << angles::to_degrees(fov_y_)
                             << " fx: " << cam_model_.fx() << " fy: " << cam_model_.fy()
                             << " tilt: " << angles::to_degrees(cam_tilt) << " alt: " << alt);

        // Minimum distance to the camera (for obz): 4m
        // Maximum distance to the camera (for obz): 30m
        const double H_min = 1.0;
        const double H_max = 2.0;
        const double D1 = alt / sin(cam_tilt + fov_y_ / 2.0);
        //const double D2 = alt / sin(cam_tilt - fov_y_ / 2.0);
        const double d_min = std::max(D1, 4.0);
        //const double  d_max = D2 - (H_min / sin(cam_tilt - fov_y_ / 2.0));
        const double d_max = 30.0;
        const double h_min = H_min * (cam_model_.fy() / d_max);
        const double h_max = H_max * (cam_model_.fy() / d_min);
        const double w_min = /*(cam_model_.fx() / cam_model_.fy())*/ 0.5 * h_min;
        const double w_max = /*(cam_model_.fx() / cam_model_.fy())*/ 0.5 * h_max;

        ROS_WARN_STREAM("[BEH] Obzerver ROI min: " << w_min << " x " << h_min << " max: " << w_max << " x " << h_max);
        ToggleObzerver(true, h_min, w_min, h_max, w_max);
      }
      else
      {
        ROS_ERROR("[BEH] Not enough info to initial ROI for obzerver, using default values");
        ToggleObzerver(true, 10, 5, 200, 100);
      }

      // End Experimental

      ToggleAutonomyHuman(false);
      ResetVisualTracker();

      break;
    }

    // constantly go to search_alt and search_yaw (x,y=0)
    if (param_perform_search_action_)
    {
      ROS_WARN_ONCE("[BEH] Search action is enabled.");
      PerformSearchAction();
    }

    //if (param_enable_camera_control_) ControlBebopCamera();

    // 1 second timeout for Reset() and Search action to work
    if (mode_duration.toSec() < 2.0)
    {
      break;
    }
    // User Feedback
//    if (sub_all_tracks_.IsActive() && sub_all_tracks_()->tracks.size())
//    {
//      const obzerver_ros::Tracks& all_tracks = sub_all_tracks_.GetMsgCopy();
//      uint32_t current_promising_tracks = 0;

//      for (uint32_t i = 0; i < all_tracks.tracks.size(); ++i)
//      {
//        const obzerver_ros::Track& track = all_tracks.tracks[i];
//        if ((track.displacement < 20.0) &&
//            (track.status == obzerver_ros::Track::STATUS_TRACKING)
//            /* && (track.dominant_freq > 0.0)*/)
//        {
//          current_promising_tracks++;
//        }
//      }

//      // Ask Sepehr why!
////      if ((current_promising_tracks > 0) && (promising_tracks == 0))
////      {
////        led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_FULL_BLINK, "green", "blue", 5.0);
////      }

////      if ((current_promising_tracks == 0) && (promising_tracks >0))
////      {
////        led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_SEARCH_1, "green", "blue", 3);
////      }

//      promising_tracks = current_promising_tracks;
//    }

    // TODO: Add confidence
    if (sub_visual_tracker_track_.IsActive() &&
        sub_visual_tracker_track_()->status == cftld_ros::Track::STATUS_TRACKING)
    {
      const cftld_ros::Track& t = sub_visual_tracker_track_.GetMsgCopy();
      ROS_INFO_STREAM("[BEH] Visual tracker has been initialized. Approaching her ... id: "
                      <<  t.uid << " confidence: " << t.confidence);
      ToggleObzerver(false);
      Transition(constants::MODE_APPROACHING_PERSON);
      break;
    }

    // The manual ROI has a higher priority than obzerver
    if (sub_manual_roi_.IsActive())
    {
      const sensor_msgs::RegionOfInterest& roi = sub_manual_roi_.GetMsgCopy();
      ROS_WARN_STREAM("[BEH]  Manual ROI for initialization [x, y, w, h]: "
                      << roi.x_offset << " " << roi.y_offset << " "
                      << roi.width << " " << roi.height);
      pub_cftld_tracker_init_.publish(roi);
      ToggleObzerver(false);
      break;
    }
    else if (sub_periodic_tracks_.IsActive() && sub_periodic_tracks_()->tracks.size())
    {
      ROS_WARN_STREAM("[BEH] Found " << sub_periodic_tracks_()->tracks.size() << " periodic tracks.");

      int32_t selected_id = -1;
      double max_freq = 0.0;
      const obzerver_ros::Tracks& all_per_tracks = sub_periodic_tracks_.GetMsgCopy();
      for (int32_t i = 0; i < all_per_tracks.tracks.size(); ++i)
      {
        const obzerver_ros::Track& pt = all_per_tracks.tracks[i];
        ROS_DEBUG_STREAM("[BEH] i: " << i << " disp: " << pt.displacement << " freq: " << pt.dominant_freq);
        if (pt.status == obzerver_ros::Track::STATUS_TRACKING &&
            pt.displacement < 30.0 &&
            pt.dominant_freq > max_freq)
        {
          selected_id = i;
          max_freq = pt.dominant_freq;
        }
      }

      if (selected_id != -1)
      {
        const obzerver_ros::Track& pt = all_per_tracks.tracks[selected_id];
        const int32_t img_w = sub_periodic_tracks_()->max_width;
        const int32_t img_h = sub_periodic_tracks_()->max_height;
        ROS_WARN_STREAM("[BEH]  ID: " << selected_id);
        ROS_WARN_STREAM("[BEH]  Frequency: " << pt.dominant_freq);
        ROS_WARN_STREAM("[BEH]  Displacement: " << pt.displacement);

        sensor_msgs::RegionOfInterest roi = pt.roi;
//        int32_t roi_wh = std::max(pt.roi.width, pt.roi.height);
//        if (roi.width + roi_wh > img_w) roi_wh = img_w - roi.x_offset - 1;
//        if (roi.height + roi_wh > img_h) roi_wh = img_h - roi.y_offset - 1;

        int32_t roi_wh = std::max(pt.roi.width, pt.roi.height);
        roi.x_offset -= (roi_wh - roi.width) / 2;
        roi.y_offset -= (roi_wh - roi.height) / 2;
        roi.width = roi_wh;
        roi.height = roi_wh;

        roi.x_offset = util::clamp<int32_t>(roi.x_offset, 0, cam_model_.fullResolution().width - 1);
        roi.y_offset = util::clamp<int32_t>(roi.y_offset, 0, cam_model_.fullResolution().height - 1);
        const int32_t x2 = util::clamp<int32_t>(roi.x_offset + roi.width, 0, cam_model_.fullResolution().width - 1);
        const int32_t y2 = util::clamp<int32_t>(roi.y_offset + roi.height, 0, cam_model_.fullResolution().height - 1);
        roi.width = x2 - roi.x_offset;
        roi.height = y2 - roi.y_offset;

        ROS_WARN_STREAM("[BEH] The track is stationary enough [x, y, w, h]: "
                        << roi.x_offset << " , " << roi.y_offset << " , "
                        << roi.width << " , " << roi.height);

        pub_cftld_tracker_init_.publish(roi);
        ToggleObzerver(false);
        sub_periodic_tracks_.Deactivate();
        break;
      }
      else
      {
        ROS_WARN_THROTTLE(1, "[BEH] None of periodic tracks were stationary enough.");
      }
    }

    break;
  }

  case constants::MODE_APPROACHING_PERSON:
  {
    if (is_transition)
    {
      ROS_INFO_STREAM("[BEH] Enabling visual servo and human detector...");
      ToggleVisualServo(true);
      ToggleAutonomyHuman(true);
      ResetGestures();
    }

    // Visual tracker's inactiviy is either caused by input stream's being stale or
    // a crash. The former needs a seperate recovery case since this node can also detects it.
    if (!sub_visual_tracker_track_.IsActive())
    {
      ROS_ERROR_STREAM("[BEH] Visual tracker is stale, this should never happen.");

      ToggleVisualServo(false);
      ToggleAutonomyHuman(false);
      Transition(constants::MODE_APPROACHING_LOST);
      break;
    }


    const cftld_ros::Track& t = sub_visual_tracker_track_.GetMsgCopy();

    // TODO: Add confidence
    if (t.status != cftld_ros::Track::STATUS_TRACKING)
    {
      ROS_WARN("[BEH] Tracker has lost the person");
      Transition(constants::MODE_APPROACHING_LOST);
    }
    else
    {
      // Feedback
      ROS_ASSERT(sub_camera_info_()->width != 0);
      const int view_angle_ = -180.0 * (((double(t.roi.width / 2.0)+ t.roi.x_offset)
                                        / sub_camera_info_()->width) - 0.5);

      led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_LOOK_AT,
                                 "green", "blue", 90, view_angle_);

      msg_vservo_target_.header.stamp = ros::Time::now();

      // re-initialize the servo, only when in mode transition and not from a previous override mode

      msg_vservo_target_.reinit = static_cast<bool>(is_transition &&
                                   (bebop_mode_prev_ != constants::MODE_BAD_VIDEO) &&
                                   (bebop_mode_prev_ != constants::MODE_MANUAL) &&
                                   (bebop_mode_prev_ != constants::MODE_APPROACHING_LOST)
                                   );

      msg_vservo_target_.desired_depth = param_servo_desired_depth_;
      msg_vservo_target_.target_distance_ground = param_target_dist_ground_;
      msg_vservo_target_.target_height_m = param_target_height_;
      // TODO: FIX ME
      msg_vservo_target_.target_width_m = param_target_height_;
      msg_vservo_target_.roi = t.roi;  // roi was enforced to be rect when cftld_ros was being initialized
      // vservo node will cache this value on its first call or when reinit=true
      // ignores it all other time
      msg_vservo_target_.desired_yaw_rad = -sub_bebop_att_()->yaw;
      pub_visual_servo_target_.publish(msg_vservo_target_);

      if (param_enable_camera_control_) ControlBebopCamera();

      if (sub_human_.IsActive() && (sub_human_()->status == autonomy_human::human::STATUS_TRACKING))
      {
        const autonomy_human::human& human = sub_human_.GetMsgCopy();
        const float face_roi_intersect = util::GetROIIntersectArea(t.roi, human.faceROI);
        const float face_area = static_cast<float>(human.faceROI.width * human.faceROI.height);
        if (face_area > 0)
        {
          ROS_WARN_STREAM("[BEH] Human found: Face score: " << human.faceScore << " Area: "
                          << face_area << " Intersection with roi: " << face_roi_intersect);
          if ((human.numFaces > 0) &&
              (human.faceScore > 5) &&
              ((face_roi_intersect / face_area) > 0.5))
          {
            Transition(constants::MODE_CLOSERANGE_ENGAGED);
          }
        }
      }
    }

    break;
  }

  case constants::MODE_APPROACHING_LOST:
  {
    if (is_transition)
    {
      ROS_INFO_STREAM("[BEH] Target lost during approach, waiting for a while ...");
      ToggleVisualServo(false);
      led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_SEARCH_1, "red", "magenta", 6);
    }

    if (mode_duration.toSec() > 10.0)
    {
      ROS_WARN("[BEH] Recovery failed");
      Transition(constants::MODE_IDLE);
      break;
    }

    if (sub_visual_tracker_track_.IsActive() &&
        sub_visual_tracker_track_()->status == cftld_ros::Track::STATUS_TRACKING)
    {
      ROS_INFO("[BEH] Recovery has been succesful");
      Transition(constants::MODE_APPROACHING_PERSON);
      break;
    }

    // Hack by Mani
    if (sub_human_.IsActive() &&
        sub_human_()->status == autonomy_human::human::STATUS_TRACKING &&
        sub_human_()->numFaces > 0 &&
        sub_human_()->faceScore > 5)
    {
      ROS_INFO("[BEH] Approach recovery using face has been succesfull");
      Transition(constants::MODE_CLOSERANGE_ENGAGED);
      break;
    }
    break;
  }

  case constants::MODE_CLOSERANGE_ENGAGED:
  {
    if (is_transition)
    {
      ROS_INFO_STREAM("[BEH] Close range interaction ...");
      ToggleVisualServo(true);
      ResetVisualTracker();
      ResetGestures();
    }

    // Human tracker's inactiviy is either caused by input stream's being stale or
    // a crash. The former needs a seperate recovery case since this node can also detects it.
    if (!sub_human_.IsActive())
    {
      ROS_ERROR_STREAM("[BEH] Human tracker is stale, this should never happen.");

      ToggleVisualServo(false);
      Transition(constants::MODE_CLOSERANGE_LOST);
      break;
    }

    const autonomy_human::human& h = sub_human_.GetMsgCopy();

    if ((h.status != autonomy_human::human::STATUS_TRACKING))
    {
      ROS_WARN("[BEH] Tracker has lost the human");
      Transition(constants::MODE_CLOSERANGE_LOST);
    }
    else
    {
      msg_vservo_target_.header.stamp = ros::Time::now();

      // re-initialize the servo, only when in mode transition and not from a previous override mode

      msg_vservo_target_.reinit = static_cast<bool>(is_transition &&
                                   (bebop_mode_prev_ != constants::MODE_BAD_VIDEO) &&
                                   (bebop_mode_prev_ != constants::MODE_MANUAL) &&
                                   (bebop_mode_prev_ != constants::MODE_CLOSERANGE_LOST)
                                   );

      msg_vservo_target_.desired_depth = param_servo_desired_depth_;
      msg_vservo_target_.target_distance_ground = param_target_dist_ground_;

      //https://upload.wikimedia.org/wikipedia/commons/6/61/HeadAnthropometry.JPG
      //msg_vservo_target_.target_height_m = 0.3;
      //msg_vservo_target_.target_width_m = 0.2;
      msg_vservo_target_.target_height_m = 0.2;
      msg_vservo_target_.target_width_m = 0.2;

      msg_vservo_target_.roi = h.faceROI;
      // vservo node will cache this value on its first call or when reinit=true
      // ignores it all other time
      msg_vservo_target_.desired_yaw_rad = -sub_bebop_att_()->yaw;

      ROS_DEBUG_STREAM_THROTTLE(0.5, "[BEH] CLOSE RANGE SERVO: " <<
               h.faceROI.x_offset << " " << h.faceROI.y_offset << " " << h.faceROI.width << " " << h.faceROI.height);
      pub_visual_servo_target_.publish(msg_vservo_target_);


      if (param_enable_camera_control_) ControlBebopCamera();

      // Gesture && Feedback
      flow_left_vec_.push_front(h.flowScore[0]);
      flow_right_vec_.push_front(h.flowScore[1]);
      if (flow_left_vec_.size() > param_flow_queue_size_) flow_left_vec_.pop_back();
      if (flow_right_vec_.size() > param_flow_queue_size_) flow_right_vec_.pop_back();

      ROS_ASSERT(sub_camera_info_()->width != 0);
      const int view_angle_ = -180.0 * (((double(h.faceROI.width / 2.0)+ h.faceROI.x_offset)
                                        / sub_camera_info_()->width) - 0.5);
      bool gesture_feedback = false;
      if (GestureUpdate() && gesture_curr_ != constants::GESTURE_NONE)
      {
        ROS_ERROR_STREAM("[BEH Gesture Detected: " << GESTURE_STR(gesture_curr_));
        if ((gesture_curr_ == constants::GESTURE_RIGHT_SWING) ||
            (gesture_curr_ == constants::GESTURE_LEFT_SWING))
        {
          Transition(constants::MODE_CLOSERANGE_SINGLECOMMNAD);
          break;
        }
        if (gesture_curr_ == constants::GESTURE_BOTH_SWING)
        {
          Transition(constants::MODE_CLOSERANGE_DOUBLECOMMNAD);
          break;
        }
      }
      else
      {
        if ((flow_left_median_ > 1.0 && flow_left_median_ > flow_right_median_))
        {
          led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_EYE,
                                     "green", "white", 20, view_angle_);
          gesture_feedback = true;
        }

        if ((flow_right_median_ > 1.0 && flow_right_median_ > flow_left_median_))
        {
          led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_EYE,
                                     "white", "green", 20, view_angle_);
          gesture_feedback = true;
        }

        if (flow_left_median_ > 1.0 && flow_right_median_ > 1.0 && fabs(flow_left_median_ - flow_left_median_) < 1.0)
        {
          led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_EYE,
                                     "green", "green", 20, view_angle_);
          gesture_feedback = true;
        }

//        if (flow_right_median_ > 1.0 && flow_right_median_ > flow_left_median_)
//        {
//          led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_LOOK_AT,
//                                     "white", "red", flow_right_median_ * 90, view_angle_);
//          gesture_feedback = true;
//        }
      }

      if (!gesture_feedback)
      {
        // General Feedback
        led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_EYE,
                                   "white", "white", 10, view_angle_);
      }

    }
    break;
  }

  case constants::MODE_CLOSERANGE_SINGLECOMMNAD:
  {
    if (is_transition && (bebop_mode_prev_ == constants::MODE_CLOSERANGE_ENGAGED))
    {
      ROS_INFO_STREAM("[BEH] Selfie!");
      // This feedback type is handcrafted for three seconds of duration
      led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_TIMER_SNAP,
                                 "white", "blue", 10);
    }

    if (mode_duration.toSec() > 3.0)
    {
      BebopSnapshot();
      Transition(constants::MODE_CLOSERANGE_ENGAGED);
    }

    break;
  }

  case constants::MODE_CLOSERANGE_DOUBLECOMMNAD:
  {
    if (is_transition && (bebop_mode_prev_ == constants::MODE_CLOSERANGE_ENGAGED))
    {
      ROS_INFO_STREAM("[BEH] Bye Bye!");
      led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_BYEBYE,
                                 "black", "cyan", 3.0);
    }

    if (mode_duration.toSec() > 3.0)
    {
      if (sub_bebop_att_.IsActive())
      {
        desired_search_yaw_ = angles::normalize_angle(3.14156 - sub_bebop_att_()->yaw);
      }
      else
      {
        desired_search_yaw_ = angles::normalize_angle(3.14156 + desired_search_yaw_);
      }

      ROS_WARN_STREAM("Desired yaw: " << angles::to_degrees(desired_search_yaw_));
      Transition(constants::MODE_SEARCHING);
    }

    break;
  }

  case constants::MODE_CLOSERANGE_LOST:
  {
    if (is_transition)
    {
      ROS_INFO_STREAM("[BEH] Target lost during close range interaction, waiting for a while ...");
      ToggleVisualServo(false);
      led_feedback_.SendFeedback(autonomy_leds_msgs::Feedback::TYPE_FAST_BLINK, "red", "black", 6);
    }

    if (mode_duration.toSec() > 30.0)
    {
      ROS_WARN("[BEH] Recovery failed");
      Transition(constants::MODE_IDLE);
      break;
    }

    if (sub_human_.IsActive() &&
        sub_human_()->status == autonomy_human::human::STATUS_TRACKING &&
        sub_human_()->numFaces > 0 &&
        sub_human_()->faceScore > 5)
    {
      ROS_INFO("[BEH] Close range recovery has been succesful");
      Transition(constants::MODE_CLOSERANGE_ENGAGED);
    }
    break;
  }

  case constants::MODE_NUM:
  {
    ROS_FATAL("WTF!");
  }
  }

}

void BebopBehaviorNode::Spin()
{
  ros::Rate loop_rate(param_update_rate_);

  MoveBebopCamera(0.0, 0.0);
  while (ros::ok())
  {
    try
    {
      UpdateBehavior();
      if (!loop_rate.sleep())
      {
        ROS_WARN_STREAM("[BEH] Loop frequency of " << param_update_rate_ << " missed!");
      }
      ros::spinOnce();
    }
    catch (const ros::Exception& e)
    {
      ROS_ERROR_STREAM("[BEH] ROS error: " << e.what());
    }
    catch (const std::runtime_error& e)
    {
      ROS_ERROR_STREAM("[BEH] Runtime error: " << e.what());
    }
  }

  ROS_INFO_STREAM("[BEH] Exiting the main loop");
}

}  // namespace bebop_hri

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bebop_behavior_node");

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  double update_freq;
  int32_t initial_state;

  ROS_INFO_STREAM("[BEH] Starting behavior node ...");
  bebop_hri::BebopBehaviorNode bebop_behavior(nh, priv_nh);
  bebop_behavior.Spin();
}
