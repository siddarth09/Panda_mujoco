#include "panda_mujoco/cube_detector.hpp"

CubeDetector::CubeDetector():Node("cube_detector")
{
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/top_rgb/color",10,
        std::bind(&CubeDetector::image_callback,this,std::placeholders::_1));


    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/top_rgb/depth", 10,
    std::bind(&CubeDetector::depth_callback, this, std::placeholders::_1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/top_rgb/camera_info", 10,
        std::bind(&CubeDetector::camera_info_callback, this, std::placeholders::_1));

    cube_pub_ = create_publisher<panda_mujoco::msg::CubeDetector>(
        "/cube_detections_3d", 10);

    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/cube_debug_image", 10);
    
    
    RCLCPP_INFO(get_logger(),"Cube Detector node started");

}

void CubeDetector::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    fx_= msg->k[0];
    fy_= msg->k[4];
    cx_= msg->k[2];
    cy_= msg->k[5];

    cam_ready_= true;
}

void CubeDetector::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    depth_image_= cv_bridge::toCvCopy(msg,msg->encoding)->image;

}

void CubeDetector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg){

    if (!cam_ready_ || depth_image_.empty()) return; 

    cv::Mat rgb = cv_bridge::toCvCopy(msg,"bgr8")->image;
    detectCubes(rgb);


}

//Core Idea 

void CubeDetector::detectCubes(const cv::Mat &rgb){
    cv::Mat hsv; 
    cv::cvtColor(rgb,hsv,cv::COLOR_BGR2HSV); //Convert to hsv 

    struct ColorRange{
        std::string name;
        cv::Scalar low,high;
    };


    std::vector<ColorRange> colors = {
    {"red",    {0,120,70},  {10,255,255}},
    {"green",  {40,70,70},  {80,255,255}},
    {"blue",   {100,150,70},{130,255,255}},
    {"yellow", {20,100,100},{35,255,255}},
    {"purple", {140,50,50}, {165,255,255}}
    };

    cv::Mat debug = rgb.clone();
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,{5,5}); //Noise removal

    for (const auto &c: colors){

        cv::Mat mask;
        cv::inRange(hsv,c.low,c.high,mask); //Binary mask per color 
        cv::morphologyEx(mask,mask,cv::MORPH_OPEN,kernel); //Mprphological opening removes noise
        std::vector<std::vector<cv::Point>> contours; 

        cv::findContours(mask,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE); //Extracting Contors
        for (const auto &cnt : contours){
            if (cv::contourArea(cnt)<400) continue; 
            //Pixel centroid
            cv::Moments m = cv::moments(cnt);
            int u = static_cast<int>(m.m10/m.m00);
            int v = static_cast<int>(m.m01/m.m00);

            double z = getDepthMedian(u,v);
            if (!std::isfinite(z)) continue;
            // Pixel to 3D projection 
            double X = (u - cx_) * z / fx_; 
            double Y = (v - cy_) * z / fy_;

            panda_mujoco::msg::CubeDetector det;
            det.header.stamp = now();
            det.header.frame_id = camera_frame_;
            det.color = c.name;
            det.position.x = X;
            det.position.y = Y;
            det.position.z = z;

            cube_pub_->publish(det);
            // RCLCPP_INFO(
            //     get_logger(),
            //     "Cube detected | color: %s | x: %.3f y: %.3f z: %.3f",
            //     det.color.c_str(),
            //     det.position.x,
            //     det.position.y,
            //     det.position.z
            //     );


           

            cv::circle(debug,{u,v},5,{0,255,0},-1);
            cv::putText(debug,c.name,{u+5,v-5},cv::FONT_HERSHEY_SIMPLEX,0.5,{0,255,0},1);
        }
    }

    debug_pub_->publish(
        *cv_bridge::CvImage(
            std_msgs::msg::Header(),"bgr8",debug).toImageMsg()
        );
}


double CubeDetector::getDepthMedian(int u,int v,int window)
{
    /*

    Samples a square window 
    removes NaNs in the window
    returns median depth 
    */
    int h = depth_image_.rows;
    int w = depth_image_.cols;
    int r = window/2 ; 

    int u0 = std::max(0,u-r);
    int u1 = std::min(w,u+r);
    int v0 = std::max(0,v-r);
    int v1 = std::min(h,v+r);

    std::vector<double> values; 
    for(int y = v0; y< v1; ++y){
        for(int x = u0; x<u1; ++x)
        {
            double d = depth_image_.at<float>(y,x);
            if (std::isfinite(d) && d>0.0)
            {
                values.push_back(d);
            }
        }
    }

    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();

    std::nth_element(values.begin(),
                    values.begin()+values.size()/2,
                    values.end());

    return values[values.size()/2];
}



int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<CubeDetector>());
    rclcpp::shutdown();
    return 0;
}