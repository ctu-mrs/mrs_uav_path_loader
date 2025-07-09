#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include "pl_utils/simple_param_loader.hpp"
#include "pl_utils/simple_service_client.hpp"

// #include <path_loader_interfaces/srv/path_srv.hpp>
// using PathSrv = path_loader_interfaces::srv::PathSrv;

#include <mrs_msgs/srv/path_srv.hpp>
using PathSrv = mrs_msgs::srv::PathSrv;



class PathLoaderNode : public rclcpp::Node
{
public:
  PathLoaderNode() : Node("path_loader")
  {
    pl_utils::SimpleParamLoader P(this);

    // ---- обязательная матрица path ----
    auto flat = P.load_required<std::vector<double>>("path"); 
    if (flat.empty() || flat.size() % 4)
    {
      RCLCPP_FATAL(get_logger(), "'path' must contain 4xN doubles");
      rclcpp::shutdown();
      return;
    }
    size_t rows = flat.size() / 4;
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 4, Eigen::RowMajor>> path_mtx(flat.data(), rows, 4);

    // ---- дополнительные параметры ----
    std::string frame_id   = declare_parameter("frame_id", "map");
    bool fly_now           = declare_parameter("fly_now", false);
    bool loop              = declare_parameter("loop", false);
    bool stop_wp           = declare_parameter("stop_at_waypoints", false);
    bool use_hdg           = declare_parameter("use_heading", false);
    bool relax_hdg         = declare_parameter("relax_heading", false);
    bool dont_prepend      = declare_parameter("dont_prepend_current_state", false);
    double stamp_shift     = declare_parameter("stamp", 0.0);
    double max_exec_time   = declare_parameter("max_execution_time", 0.0);
    double max_dev         = declare_parameter("max_deviation_from_path", 0.0);

    bool   constr_override = declare_parameter("constraints.override", false);
    double constr_vh       = declare_parameter("constraints.speed_horizontal",     0.0);
    double constr_vv       = declare_parameter("constraints.speed_vertical",       0.0);
    double constr_ah       = declare_parameter("constraints.acceleration_horizontal", 0.0);
    double constr_av       = declare_parameter("constraints.acceleration_vertical",   0.0);
    double constr_jh       = declare_parameter("constraints.jerk_horizontal",      0.0);
    double constr_jv       = declare_parameter("constraints.jerk_vertical",        0.0);
 
    // ---- клиент ----
    pl_utils::SimpleServiceClient<PathSrv> client(this, "path_out");

    // ---- формируем запрос ----
    auto req = std::make_shared<PathSrv::Request>();
    req->path.header.frame_id = frame_id;
    req->path.header.stamp    = (stamp_shift == 0.0)
                                  ? rclcpp::Time(0)
                                  : (now() + rclcpp::Duration::from_seconds(stamp_shift));
    req->path.fly_now                    = fly_now;
    req->path.loop                       = loop;
    req->path.stop_at_waypoints          = stop_wp;
    req->path.use_heading                = use_hdg;
    req->path.relax_heading              = relax_hdg;
    req->path.dont_prepend_current_state = dont_prepend;
    req->path.max_execution_time         = max_exec_time;
    req->path.max_deviation_from_path    = max_dev;

    req->path.override_constraints = constr_override;
    if (constr_override)
    {
      req->path.override_max_velocity_horizontal     = constr_vh;
      req->path.override_max_velocity_vertical       = constr_vv;
      req->path.override_max_acceleration_horizontal = constr_ah;
      req->path.override_max_acceleration_vertical   = constr_av;
      req->path.override_max_jerk_horizontal         = constr_jh;
      req->path.override_max_jerk_vertical           = constr_jv;
    }

    // точки
    for (size_t i = 0; i < rows; ++i)
    {
      // path_loader_interfaces::msg::Reference ref;
      // стало
      mrs_msgs::msg::Reference ref;
      ref.position.x = path_mtx(i,0);
      ref.position.y = path_mtx(i,1);
      ref.position.z = path_mtx(i,2);
      ref.heading    = path_mtx(i,3);
      req->path.points.push_back(ref);
    }

    // ---- вызов ----
    if (!client.call_and_wait(req))
    {
      RCLCPP_ERROR(get_logger(), "Service call timed out");
    }
    else if (!client.last_response()->success)
    {
      RCLCPP_ERROR(get_logger(), "Path rejected: %s",
                   client.last_response()->message.c_str());
    }
    else
    {
      // RCLCPP_INFO(get_logger(), "Path accepted");
      RCLCPP_INFO(get_logger(), "Path accepted: %s",
                   client.last_response()->message.c_str());
      RCLCPP_ERROR(get_logger(), "Succes: %d", client.last_response()->success);
    }

    // rclcpp::shutdown();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // конструктор выполняет всю работу синхронно
  auto node = std::make_shared<PathLoaderNode>();

  rclcpp::shutdown();
  return 0;
}

