/*
  Autoware trajectory publisher node in carla simulatior
  map name : Town04
  node name : trajectory_router 
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



using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
using LongIntVec = std::vector<long int>;

class TrajectoryRouter : public rclcpp::Node
{
public:
  TrajectoryRouter() : Node("trajectory_router")
  {
    LongIntVec primitive;

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>( "/localization/pose_estimator/pose", rclcpp::QoS{1}, std::bind(&TrajectoryRouter::callbackPose, this, std::placeholders::_1));
    sub_vector_map_ = create_subscription<HADMapBin>("/map/vector_map", rclcpp::QoS(1).transient_local(), std::bind(&TrajectoryRouter::callbackMap, this, std::placeholders::_1));
    route_pub_= create_publisher<autoware_planning_msgs::msg::LaneletRoute>("/planning/mission_planning/route", rclcpp::QoS{1}.transient_local());

    currentLaneletId = INITLANELET;
    lastLaneletId = INITLANELET;
    lastLaneletKey = INITKEY;
    pubJoker = false;
    pubState = TrajectoryRouter::READY;

    this->declare_parameter<LongIntVec>("available_lanelet_id", LongIntVec({}));
    available_lanelets = this->get_parameter("available_lanelet_id").as_integer_array();
    this->declare_parameter<LongIntVec>("desired_lane", LongIntVec({}));
    desiredLane = this->get_parameter("desired_lane").as_integer_array();
    this->declare_parameter<int>("lane_number", int());
    laneNum = this->get_parameter("lane_number").as_int();
    this->declare_parameter<int>("primitive_number", int());
    primitiveNum = this->get_parameter("primitive_number").as_int();
    
    for(int i=INITCOUNT; i<primitiveNum; i++){
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

    // joker 

    this->declare_parameter<double>("joker.position.x", double());
    this->declare_parameter<double>("joker.position.y", double());
    this->declare_parameter<double>("joker.position.z", double());
    jokerPosition_x = this->get_parameter("joker.position.x").as_double();
    jokerPosition_y = this->get_parameter("joker.position.y").as_double();
    jokerPosition_z = this->get_parameter("joker.position.z").as_double();

    this->declare_parameter<double>("joker.orientaion.x", double());
    this->declare_parameter<double>("joker.orientaion.y", double());
    this->declare_parameter<double>("joker.orientaion.z", double());
    this->declare_parameter<double>("joker.orientaion.w", double());
    jokerOrientation_x = this->get_parameter("joker.orientaion.x").as_double();
    jokerOrientation_y = this->get_parameter("joker.orientaion.y").as_double();
    jokerOrientation_z = this->get_parameter("joker.orientaion.z").as_double();
    jokerOrientation_w = this->get_parameter("joker.orientaion.w").as_double();

    this->declare_parameter<LongIntVec>("joker.available_lanelet_id", LongIntVec({}));
    jokerAvailableLaneletID = this->get_parameter("joker.available_lanelet_id").as_integer_array();
    this->declare_parameter<LongIntVec>("joker.primitive", LongIntVec({}));
    jokerPrimitive = this->get_parameter("joker.primitive").as_integer_array();
  }
  int laneNum;
  int primitiveNum;
  long int currentLaneletId;
  long int lastLaneletId;
  long int laneletKey;
  long int lastLaneletKey;
  enum PubState {READY, NORMAL, JOKER};
  PubState pubState;

  // callbackMap valiable
  lanelet::LaneletMapPtr TR_lanelet_map_ptr_;
  HADMapBin::ConstSharedPtr TR_map_ptr_;
  lanelet::ConstLanelets all_lanelets;
  lanelet::ConstLanelets road_lanelets_;

  LongIntVec available_lanelets;
  LongIntVec desiredLane;
  std::vector<LongIntVec> currentPrimitive2DVector;
  std::vector<LongIntVec> primitive2DVector;

  std::vector<double> position_x;
  std::vector<std::vector<double>> position2D_x;
  std::vector<double> position_y;
  std::vector<std::vector<double>> position2D_y;
  double position_z;
  double orientation_x;
  double orientation_y;
  std::vector<double> orientation_z;
  std::vector<double> orientation_w;

  autoware_planning_msgs::msg::LaneletSegment segment;
  autoware_planning_msgs::msg::LaneletSegment emptySegment;
  autoware_planning_msgs::msg::LaneletPrimitive primitive;

  // callback function
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_pub_;

  // joker 
  bool pubJoker;
  double jokerPosition_x;
  double jokerPosition_y;
  double jokerPosition_z;
  double jokerOrientation_x;
  double jokerOrientation_y;
  double jokerOrientation_z;
  double jokerOrientation_w;
  LongIntVec jokerAvailableLaneletID;
  LongIntVec jokerPrimitive;
  
private:
  std::vector<LongIntVec> setPrimitiveVector(const LongIntVec& vec, size_t laneletNum)
  {
    std::vector<LongIntVec> return2DVec;
    return2DVec.reserve((vec.size() + laneletNum - 1) / laneletNum); 
    auto nowVec = vec.begin();
    while (nowVec != vec.end()){
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
    while (nowVec != vec.end()){
      std::vector<double> chunk;
      chunk.reserve(laneletNum);
      auto end = std::next(nowVec, laneletNum);
      std::copy(nowVec, end, std::back_inserter(chunk));
      return2DVec.push_back(std::move(chunk));
      nowVec = end;
    }
    return return2DVec;
  }

  void callbackPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    lanelet::ConstLanelets start_lanelets;
    if (currentLaneletId == INITLANELET)
      return;
    
    // ID 획득 
    // ID를 획득할 수 없는 상황에서는 정지하게 됨 >> 예외처리를 해야할 것으로 보여짐
    if (!lanelet::utils::query::getCurrentLanelets(road_lanelets_, msg->pose, &start_lanelets))
      RCLCPP_WARN(get_logger(), "get lanelet id fault!");
    for (const auto & st_llt : start_lanelets) {
      currentLaneletId = st_llt.id();
    }

    // ID를 바탕으로 key 값과 적절한 상태 결정
    long int cnt = INITCOUNT;
    for (const auto &lane_val : available_lanelets){
      laneletKey = cnt / laneNum; 
      if (currentLaneletId == lane_val && currentLaneletId != lastLaneletId && laneletKey != lastLaneletKey){
        lastLaneletId = currentLaneletId;
        lastLaneletKey = laneletKey;
        pubState = TrajectoryRouter::NORMAL;
        currentPrimitive2DVector = setPrimitiveVector(primitive2DVector[laneletKey], laneNum);
        break;
      }
      cnt++;
    }
    
    // joker 상태 결정
    for (const auto &lane_val : jokerAvailableLaneletID){
      if (currentLaneletId == lane_val && pubJoker){
        pubState = TrajectoryRouter::JOKER;
        currentPrimitive2DVector = setPrimitiveVector(jokerPrimitive, laneNum);
        break;
      }
    }

    // state 에 따른 적절한 topic 제작
    switch (pubState){
    case TrajectoryRouter::READY:{
      break;
    }
    case TrajectoryRouter::NORMAL:{
      autoware_planning_msgs::msg::LaneletRoute route_msg_;
      route_msg_.header.stamp = this->get_clock()->now();
      route_msg_.header.frame_id = "map";
      route_msg_.start_pose = msg->pose;
      
      route_msg_.goal_pose.position.x = position2D_x[cnt/laneNum][desiredLane[laneletKey]]; 
      route_msg_.goal_pose.position.y = position2D_y[cnt/laneNum][desiredLane[laneletKey]]; 
      route_msg_.goal_pose.position.z = position_z;
          
      route_msg_.goal_pose.orientation.x = orientation_x;
      route_msg_.goal_pose.orientation.y = orientation_y;
      route_msg_.goal_pose.orientation.z = orientation_z[cnt/laneNum];  
      route_msg_.goal_pose.orientation.w = orientation_w[cnt/laneNum];

      for (const auto &out_lane_id : currentPrimitive2DVector){
        segment = emptySegment;
        segment.preferred_primitive.id = out_lane_id[desiredLane[laneletKey]];
        segment.preferred_primitive.primitive_type = "";
        for (const auto &in_lane_id : out_lane_id){
          if (in_lane_id == NOLANELET) continue;
          primitive.id = in_lane_id;
          primitive.primitive_type = "lane";
          segment.primitives.push_back(primitive);
        }
        route_msg_.segments.push_back(segment);
      }
      route_msg_.uuid.uuid = {209, 239, 15, 91, 197, 87, 68, 179, 62, 19, 3, 36, 111, 114, 35, 231};
      route_pub_->publish(route_msg_);
      pubState = TrajectoryRouter::READY;      
      std::cout << "NORMAL pub trajectory" << std::endl;
      break;
    }
    case TrajectoryRouter::JOKER:{
      autoware_planning_msgs::msg::LaneletRoute route_msg_;
      route_msg_.header.stamp = this->get_clock()->now();
      route_msg_.header.frame_id = "map";
      route_msg_.start_pose = msg->pose;
      
      route_msg_.goal_pose.position.x = jokerPosition_x; 
      route_msg_.goal_pose.position.y = jokerPosition_y; 
      route_msg_.goal_pose.position.z = jokerPosition_z; 
          
      route_msg_.goal_pose.orientation.x = jokerOrientation_x;
      route_msg_.goal_pose.orientation.y = jokerOrientation_y;
      route_msg_.goal_pose.orientation.z = jokerOrientation_z;  
      route_msg_.goal_pose.orientation.w = jokerOrientation_w;

      for (const auto &out_lane_id : currentPrimitive2DVector){
        segment = emptySegment;
        segment.preferred_primitive.id = out_lane_id[3]; // town04 조커랩 전용 하드코딩 값
        segment.preferred_primitive.primitive_type = "";
        for (const auto &in_lane_id : out_lane_id){
          if (in_lane_id == NOLANELET) continue;
          primitive.id = in_lane_id;
          primitive.primitive_type = "lane";
          segment.primitives.push_back(primitive);
        }
        route_msg_.segments.push_back(segment);
      }
      route_msg_.uuid.uuid = {209, 239, 15, 91, 197, 87, 68, 179, 62, 19, 3, 36, 111, 114, 35, 231};
      route_pub_->publish(route_msg_);
      pubState = TrajectoryRouter::READY;      
      pubJoker = false;
      std::cout << "JOKER pub trajectory" << std::endl;
      break;
    }
    default:{
      std::cout << "Unknown pubState" << std::endl;
      break;
    }
    }
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