#include "panda_mujoco/cube_tf_transformer.hpp"

CubeTF::CubeTF():Node("cube_tf"),tf_buffer_(get_clock()),tf_listener_(tf_buffer_){

    sub_ = create_subscription<panda_mujoco::msg::CubeDetector>(
        "/cube_detections_3d",10,
        std::bind(&CubeTF::cube_callback,this,std::placeholders::_1)
    );

    pub_= create_publisher<panda_mujoco::msg::CubeDetector>(
        "/cube_detections_table",10);
    

    sub_table_ = create_subscription<panda_mujoco::msg::CubeDetector>(
        "/cube_detections_table",10,
        std::bind(&CubeTF::cube_table_to_base,this,std::placeholders::_1)
    );

    pub_table_ = create_publisher<panda_mujoco::msg::CubeDetector>(
        "/cube_table_to_base",10
    );
    RCLCPP_INFO(get_logger(),"cube tf on table being published");

}

void CubeTF::cube_callback(
    const panda_mujoco::msg::CubeDetector::SharedPtr msg
)
{
    geometry_msgs::msg::PointStamped cam_pt,table_pt;

    cam_pt.header = msg->header;
    cam_pt.point = msg->position;

    try{
        tf_buffer_.transform(
            cam_pt,
            table_pt,
            "table_frame",
            tf2::durationFromSec(0.1)
        );
    }
    catch (const tf2::TransformException &ex){
        RCLCPP_WARN(get_logger(),"TF failed: %s", ex.what());
        return;
    }

    panda_mujoco::msg::CubeDetector out= *msg; 
    out.header = table_pt.header;
    out.position = table_pt.point;

    pub_-> publish(out);
}

void CubeTF::cube_table_to_base(const panda_mujoco::msg::CubeDetector::SharedPtr msg)
{
  geometry_msgs::msg::PointStamped in_pt, out_pt;
  in_pt.header = msg->header;
  in_pt.point = msg->position;

  try {
    tf_buffer_.transform(
      in_pt,
      out_pt,
      "link0",   
      tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "TF failed: %s", ex.what());
    return;
  }

  auto out = *msg;
  out.header.frame_id = "link0";
  out.position = out_pt.point;
  pub_table_->publish(out);
}

int main (int argc, char **argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<CubeTF>());
    rclcpp::shutdown();
    return 0; 
    
}