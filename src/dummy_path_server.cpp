#include <rclcpp/rclcpp.hpp>
// #include <path_loader_interfaces/srv/path_srv.hpp>s
#include <mrs_msgs/srv/path_srv.hpp>
using PathSrv = mrs_msgs::srv::PathSrv;

class DummyPathServer : public rclcpp::Node
{
public:
  DummyPathServer() : Node("dummy_path_server")
  {
    srv_ = create_service<PathSrv>(
      "path_out",
      [this](
        const std::shared_ptr<PathSrv::Request>  req,
        std::shared_ptr<PathSrv::Response>       resp)
      {
        RCLCPP_INFO(get_logger(),
          "Dummy server: got %zu points (frame_id='%s', fly_now = %d, loop = %d)",
          req->path.points.size(),
          req->path.header.frame_id.c_str(),
          req->path.fly_now,
          req->path.loop);

        resp->success = true;
        resp->message = "OK";
      });
  } 
private:
  rclcpp::Service<PathSrv>::SharedPtr srv_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyPathServer>());
  rclcpp::shutdown();
  return 0;
}
