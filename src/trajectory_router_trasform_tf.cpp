/*
  Autoware trajectory publisher node in carla simulatior, KIAPI and K-city
  map name : Town04, KIAPI, K-city
  node name : trajectory_router_node
  e-mail : yunsh3594@gmail.com 
  made by. Seunghoe Yun
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <trajectory_router/trajectory_planner.hpp>
#include <trajectory_router/trajectory_route_handler.hpp>
#include <trajectory_router/trajectory_router_plugin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
using LongIntVec = std::vector<long int>;

class TrajectoryRouter : public rclcpp::Node
{
public:
  TrajectoryRouter() : Node("trajectory_router")
  {
    LongIntVec primitive;
    currentLaneletId = INITLANELET;
    lastLaneletId = INITLANELET;
    lastLaneletKey = INITKEY;
    pubState = false;
    
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>( "/localization/pose_estimator/pose", rclcpp::QoS{1}, std::bind(&TrajectoryRouter::callbackPose, this, std::placeholders::_1));
    sub_vector_map_ = create_subscription<HADMapBin>("/map/vector_map", rclcpp::QoS(1).transient_local(), std::bind(&TrajectoryRouter::callbackMap, this, std::placeholders::_1));
    route_pub_= create_publisher<autoware_planning_msgs::msg::LaneletRoute>("/planning/mission_planning/route", rclcpp::QoS{1}.transient_local());
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/perception/obstacle_segmentation/pointcloud", rclcpp::SensorDataQoS(),
      std::bind(&TrajectoryRouter::pointcloud_callback, this, std::placeholders::_1));

    transformed_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/transformed_pointcloud", rclcpp::SensorDataQoS());
    this->declare_parameter<LongIntVec>("available_lanelet_id", LongIntVec({}));
    available_lanelets = this->get_parameter("available_lanelet_id").as_integer_array();
    this->declare_parameter<LongIntVec>("desired_lane", LongIntVec({}));
    desiredLane = this->get_parameter("desired_lane").as_integer_array();
    this->declare_parameter<int>("lane_number", int());
    laneNum = this->get_parameter("lane_number").as_int();
    this->declare_parameter<int>("primitive_number", int());
    primitiveNum = this->get_parameter("primitive_number").as_int();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    for(int i=INITCOUNT; i<primitiveNum; i++)
    {
      this->declare_parameter<LongIntVec>("primitive_"+std::to_string(i), LongIntVec({}));
      primitive = this->get_parameter("primitive_"+std::to_string(i)).as_integer_array();
      primitive2DVector.push_back(primitive);
    }

    this->declare_parameter<std::vector<double>>("position.x", std::vector<double>({}));
    this->declare_parameter<std::vector<double>>("position.y", std::vector<double>({}));
    this->declare_parameter<double>("position.z", double());
    this->declare_parameter<double>("orientaion.x", double());
    this->declare_parameter<double>("orientaion.y", double());
    this->declare_parameter<std::vector<double>>("orientaion.z", std::vector<double>({}));
    this->declare_parameter<std::vector<double>>("orientaion.w", std::vector<double>({}));

    position_x = this->get_parameter("position.x").as_double_array();
    position2D_x = setPrimitiveVector(position_x, laneNum);
    position_y = this->get_parameter("position.y").as_double_array();
    position2D_y = setPrimitiveVector(position_y, laneNum);
    position_z = this->get_parameter("position.z").as_double();
    orientation_x = this->get_parameter("orientaion.x").as_double();
    orientation_y = this->get_parameter("orientaion.y").as_double();
    orientation_z = this->get_parameter("orientaion.z").as_double_array();
    orientation_w = this->get_parameter("orientaion.w").as_double_array();

  }
  lanelet::LaneletMapPtr TR_lanelet_map_ptr_;
  HADMapBin::ConstSharedPtr TR_map_ptr_;
  lanelet::ConstLanelets all_lanelets;
  lanelet::ConstLanelets road_lanelets_;
  autoware_planning_msgs::msg::LaneletSegment segment;
  autoware_planning_msgs::msg::LaneletSegment emptySegment;
  autoware_planning_msgs::msg::LaneletPrimitive primitive;

  std::vector<std::vector<double>> position2D_x;
  std::vector<std::vector<double>> position2D_y;
  std::vector<LongIntVec> currentPrimitive2DVector;
  std::vector<LongIntVec> primitive2DVector;
  std::vector<double> position_x;
  std::vector<double> position_y;
  std::vector<double> orientation_z;
  std::vector<double> orientation_w;
  LongIntVec available_lanelets;
  LongIntVec desiredLane;
  double position_z;
  double orientation_x;
  double orientation_y;
  long int currentLaneletId;
  long int lastLaneletId;
  long int laneletKey;
  long int lastLaneletKey;
  int laneNum;
  int primitiveNum;
  bool pubState;
  sensor_msgs::msg::PointCloud2 pointcloud_;
  geometry_msgs::msg::PoseStamped pose_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_pointcloud_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_pub_;

private:
  std::vector<LongIntVec> setPrimitiveVector(const LongIntVec& vec, size_t laneletNum)
  {
    std::vector<LongIntVec> return2DVec;
    return2DVec.reserve((vec.size() + laneletNum - 1) / laneletNum); 
    auto nowVec = vec.begin();
    while (nowVec != vec.end())
    {
      LongIntVec chunk;
      chunk.reserve(laneletNum);
      auto end = std::next(nowVec, laneletNum);
      std::copy(nowVec, end, std::back_inserter(chunk));
      return2DVec.push_back(std::move(chunk));
      nowVec = end;
    }
    return return2DVec;
  }

  std::vector<std::vector<double>> setPrimitiveVector(const std::vector<double>& vec, size_t laneletNum)
  {
    std::vector<std::vector<double>> return2DVec;
    return2DVec.reserve((vec.size() + laneletNum - 1) / laneletNum); 
    auto nowVec = vec.begin();
    while (nowVec != vec.end())
    {
      std::vector<double> chunk;
      chunk.reserve(laneletNum);
      auto end = std::next(nowVec, laneletNum);
      std::copy(nowVec, end, std::back_inserter(chunk));
      return2DVec.push_back(std::move(chunk));
      nowVec = end;
    }
    return return2DVec;
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
  {
    RCLCPP_WARN(get_logger(), "2");
    pointcloud_ = *msg;
    try_transform_pointcloud();
  }

  void callbackPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_WARN(get_logger(), "1");
    pose_ = *msg;
    try_transform_pointcloud();
    // lanelet::ConstLanelets start_lanelets;

    // if (currentLaneletId == INITLANELET) return;

    // if (!lanelet::utils::query::getCurrentLanelets(road_lanelets_, msg->pose, &start_lanelets))
    //   RCLCPP_WARN(get_logger(), "get lanelet id fault!");

    // for (const auto & st_llt : start_lanelets){
    //   currentLaneletId = st_llt.id();
    //   // std::cout << "id" << currentLaneletId << std::endl;
    // }

    // long int cnt = INITCOUNT;
    // for (const auto &lane_val : available_lanelets){
    //   laneletKey = cnt / laneNum; 
    //   if (currentLaneletId == lane_val && currentLaneletId != lastLaneletId && laneletKey != lastLaneletKey){
    //     lastLaneletId = currentLaneletId;
    //     lastLaneletKey = laneletKey;
    //     pubState = true;
    //     currentPrimitive2DVector = setPrimitiveVector(primitive2DVector[laneletKey], laneNum);
    //     break;
    //   }
    //   cnt++;
    // }

    // if(pubState)
    // {
    //   autoware_planning_msgs::msg::LaneletRoute route_msg_;
    //   route_msg_.header.stamp = this->get_clock()->now();
    //   route_msg_.header.frame_id = "map";
    //   route_msg_.start_pose = msg->pose;

    //   route_msg_.goal_pose.position.x = position2D_x[cnt/laneNum][desiredLane[laneletKey]]; 
    //   route_msg_.goal_pose.position.y = position2D_y[cnt/laneNum][desiredLane[laneletKey]]; 
    //   route_msg_.goal_pose.position.z = position_z;
        
    //   route_msg_.goal_pose.orientation.x = orientation_x;
    //   route_msg_.goal_pose.orientation.y = orientation_y;
    //   route_msg_.goal_pose.orientation.z = orientation_z[cnt/laneNum];  
    //   route_msg_.goal_pose.orientation.w = orientation_w[cnt/laneNum];

    //   for (const auto &out_lane_id : currentPrimitive2DVector)
    //   {
    //     segment = emptySegment;
    //     segment.preferred_primitive.id = out_lane_id[desiredLane[laneletKey]];
    //     segment.preferred_primitive.primitive_type = "";
    //     for (const auto &in_lane_id : out_lane_id)
    //     {
    //       if (in_lane_id == NOLANELET) continue;
    //       primitive.id = in_lane_id;
    //       primitive.primitive_type = "lane";
    //       segment.primitives.push_back(primitive);
    //     }
    //     route_msg_.segments.push_back(segment);
    //   }

    //   route_msg_.uuid.uuid = {209, 239, 15, 91, 197, 87, 68, 179, 62, 19, 3, 36, 111, 114, 35, 231};
    //   route_pub_->publish(route_msg_);
    //   pubState = false;
    //   std::cout << "pub trajectory" << std::endl;
    // }
  }

  void try_transform_pointcloud()
  {
    if (pointcloud_.header.frame_id.empty() || pose_.header.frame_id.empty())
    {
      return;
    }

    RCLCPP_WARN(get_logger(), "3");
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer_->lookupTransform(
        "map", "base_link", tf2::TimePointZero);
    }
    catch (tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 transformed_pointcloud;
    tf2::doTransform(pointcloud_, transformed_pointcloud, transform_stamped);
    transformed_pointcloud_pub_->publish(transformed_pointcloud);
    RCLCPP_WARN(get_logger(), "4");
  }

  void callbackMap(const HADMapBin::ConstSharedPtr msg)
  {
    TR_map_ptr_ = msg;
    TR_lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();

    lanelet::utils::conversion::fromBinMsg(*TR_map_ptr_, TR_lanelet_map_ptr_);
    all_lanelets = lanelet::utils::query::laneletLayer(TR_lanelet_map_ptr_);
    road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
    currentLaneletId = READYLANELET;
    RCLCPP_WARN(get_logger(), "Map load Done");
  }
};

int main(int argc, char **argv)
{    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryRouter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
