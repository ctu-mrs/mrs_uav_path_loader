/* includes //{ */

#include "ros/init.h"
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>

#include <mrs_msgs/PathSrv.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/service_client_handler.h>

//}

namespace path_loader
{

/* class PathLoader //{ */

class PathLoader : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_ = false;

  std::string     _frame_id_;
  Eigen::MatrixXd _path_;
  bool            _loop_              = false;
  bool            _fly_now_           = false;
  bool            _stop_at_waypoints_ = false;
  bool            _use_heading_       = false;
  double          _stamp_;
  bool            _constraints_override_ = false;
  double          _constraints_speed_;
  double          _constraints_acceleration_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<mrs_msgs::PathSrv> sch_path_;
};

//}

/* onInit() //{ */

void PathLoader::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  // waiting for current time to be available
  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "PathLoader");

  _path_ = param_loader.loadMatrixDynamic2("path", -1, 4);

  param_loader.loadParam("fly_now", _fly_now_);
  param_loader.loadParam("frame_id", _frame_id_);
  param_loader.loadParam("use_heading", _use_heading_);
  param_loader.loadParam("stamp", _stamp_);
  param_loader.loadParam("stop_at_waypoints", _stop_at_waypoints_);
  param_loader.loadParam("loop", _loop_);
  param_loader.loadParam("constraints/override", _constraints_override_);
  param_loader.loadParam("constraints/speed", _constraints_speed_);
  param_loader.loadParam("constraints/acceleration", _constraints_acceleration_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[PathLoader]: could not load all parameters!");
    ros::shutdown();
  }

  // | --------------------- service clients -------------------- |

  sch_path_ = mrs_lib::ServiceClientHandler<mrs_msgs::PathSrv>(nh_, "path_out");

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO("[PathLoader]: initialized");

  // | ------------------- prepare the message ------------------ |

  mrs_msgs::PathSrv srv;
  srv.request.path.header.frame_id   = _frame_id_;
  srv.request.path.header.stamp      = _stamp_ == 0 ? ros::Time::now() : (ros::Time::now() + ros::Duration(_stamp_));
  srv.request.path.fly_now           = _fly_now_;
  srv.request.path.use_heading       = _use_heading_;
  srv.request.path.loop              = _loop_;
  srv.request.path.stop_at_waypoints = _stop_at_waypoints_;

  if (_constraints_override_) {
    srv.request.path.override_constraints      = _constraints_override_;
    srv.request.path.override_max_velocity     = _constraints_speed_;
    srv.request.path.override_max_acceleration = _constraints_acceleration_;
  } else {
    srv.request.path.override_constraints      = false;
    srv.request.path.override_max_velocity     = 0;
    srv.request.path.override_max_acceleration = 0;
  }

  for (int i = 0; i < _path_.rows(); i++) {

    double x       = _path_(i, 0);
    double y       = _path_(i, 1);
    double z       = _path_(i, 2);
    double heading = _path_(i, 3);

    mrs_msgs::Reference reference;
    reference.position.x = _path_(i, 0);
    reference.position.y = _path_(i, 1);
    reference.position.z = _path_(i, 2);
    reference.heading    = _path_(i, 3);

    srv.request.path.points.push_back(reference);
  }

  sch_path_.call(srv, 10, 0.1);

  if (!srv.response.success) {
    ROS_ERROR("[PathLoader]: path service call failed: %s", srv.response.message.c_str());
  } else {
    ROS_INFO("[PathLoader]: path service call succeeded");
  }

  ros::requestShutdown();
}

//}

}  // namespace path_loader

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(path_loader::PathLoader, nodelet::Nodelet)
