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
    curve = false;
    curveCnt = INITCOUNT;
    curveNum = INITCOUNT;
    pubState = TrajectoryRouter::READY;
    canState = TrajectoryRouter::READY;
    // CANSignal = TrajectoryRouter::READY;

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>( "/localization/pose_estimator/pose", rclcpp::QoS{1}, std::bind(&TrajectoryRouter::callbackPose, this, std::placeholders::_1));
    sub_vector_map_ = create_subscription<HADMapBin>("/map/vector_map", rclcpp::QoS(1).transient_local(), std::bind(&TrajectoryRouter::callbackMap, this, std::placeholders::_1));
    route_pub_= create_publisher<autoware_planning_msgs::msg::LaneletRoute>("/planning/mission_planning/route", rclcpp::QoS{1}.transient_local());

    this->declare_parameter<LongIntVec>("available_lanelet_id", LongIntVec({}));
    available_lanelets = this->get_parameter("available_lanelet_id").as_integer_array();
    this->declare_parameter<LongIntVec>("desired_lane", LongIntVec({}));
    desiredLane = this->get_parameter("desired_lane").as_integer_array();
    this->declare_parameter<int>("lane_number", int());
    laneNum = this->get_parameter("lane_number").as_int();
    this->declare_parameter<int>("primitive_number", int());
    primitiveNum = this->get_parameter("primitive_number").as_int();
    
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
    
    this->declare_parameter<LongIntVec>("curve_primitive_1", LongIntVec({}));
    curvePrimitives_1 = this->get_parameter("curve_primitive_1").as_integer_array();
    curvePrimitive2DVector_1 = setPrimitiveVector(curvePrimitives_1, laneNum);
    this->declare_parameter<LongIntVec>("curve_primitive_2", LongIntVec({}));
    curvePrimitives_2 = this->get_parameter("curve_primitive_2").as_integer_array();
    curvePrimitive2DVector_2 = setPrimitiveVector(curvePrimitives_2, laneNum);

    this->declare_parameter<std::vector<double>>("curve_1.position.x", std::vector<double>({}));
    curve1_position_x = this->get_parameter("curve_1.position.x").as_double_array();
    this->declare_parameter<std::vector<double>>("curve_1.position.y", std::vector<double>({}));
    curve1_position_y = this->get_parameter("curve_1.position.y").as_double_array();
    this->declare_parameter<std::vector<double>>("curve_2.position.x", std::vector<double>({}));
    curve2_position_x = this->get_parameter("curve_2.position.x").as_double_array();
    this->declare_parameter<std::vector<double>>("curve_2.position.y", std::vector<double>({}));
    curve2_position_y = this->get_parameter("curve_2.position.y").as_double_array();
    this->declare_parameter<double>("curve.position.z", double());
    curve_position_z = this->get_parameter("curve.position.z").as_double();

    this->declare_parameter<double>("curve.orientaion.x", double());
    curve_orientation_x = this->get_parameter("curve.orientaion.x").as_double();
    this->declare_parameter<double>("curve.orientaion.y", double());
    curve_orientation_y = this->get_parameter("curve.orientaion.y").as_double();
    this->declare_parameter<double>("curve_1.orientaion.z", double());
    curve1_orientation_z = this->get_parameter("curve_1.orientaion.z").as_double();
    this->declare_parameter<double>("curve_1.orientaion.w", double());
    curve1_orientation_w = this->get_parameter("curve_1.orientaion.w").as_double();
    this->declare_parameter<double>("curve_2.orientaion.z", double());
    curve2_orientation_z = this->get_parameter("curve_2.orientaion.z").as_double();
    this->declare_parameter<double>("curve_2.orientaion.w", double());
    curve2_orientation_w = this->get_parameter("curve_2.orientaion.w").as_double();

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
  std::vector<LongIntVec> curvePrimitive2DVector_1;
  std::vector<LongIntVec> curvePrimitive2DVector_2;
  std::vector<double> position_x;
  std::vector<double> position_y;
  std::vector<double> orientation_z;
  std::vector<double> orientation_w;
  std::vector<double> curve1_position_x;
  std::vector<double> curve1_position_y;
  std::vector<double> curve2_position_x;
  std::vector<double> curve2_position_y;
  LongIntVec available_lanelets;
  LongIntVec desiredLane;
  LongIntVec curvePrimitives_1;
  LongIntVec curvePrimitives_2;
  double position_z;
  double orientation_x;
  double orientation_y;
  double curve_position_z;
  double curve_orientation_x;
  double curve_orientation_y;
  double curve1_orientation_z;
  double curve1_orientation_w;
  double curve2_orientation_z;
  double curve2_orientation_w;
  long int currentLaneletId;
  long int lastLaneletId;
  long int laneletKey;
  long int lastLaneletKey;
  long int curveCnt;
  long int curveNum;
  int laneNum;
  int primitiveNum;
  enum PubState {READY, NORMAL, GO, STOP, PITSTOP, SLOWON, SLOWOFF};
  PubState pubState;

  // enum CANState {READY, GO, SLOWON, SLOWOFF, STOP, PITSTOP}
  PubState canState;
  bool curve;
  // PubState CANSignal;
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_pub_;
private:
  // long int 타입의 1차원 vector를 2차원 vector로 변환
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
  // double 타입의 1차원 vector를 2차원 vector로 변환
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

    if (currentLaneletId == INITLANELET || pubState == TrajectoryRouter::STOP) return;

    if (!lanelet::utils::query::getCurrentLanelets(road_lanelets_, msg->pose, &start_lanelets))
      RCLCPP_WARN(get_logger(), "get lanelet id fault!");

    for (const auto & st_llt : start_lanelets)
      currentLaneletId = st_llt.id();

    // Lanelet이 2개인 구간에서 디버깅 필요 
    // std::cout << "current lanelet ID: " << currentLaneletId << std::endl; 

    if (canState == TrajectoryRouter::STOP || canState == TrajectoryRouter::SLOWON)
    {
      curveCnt = INITCOUNT;
      curveNum = INITCOUNT;
      pubState = canState;
      for (const auto &lane : curvePrimitives_1)
      {
        if (currentLaneletId == lane)
        {
          // topic publish 
          // 추가로 뱅크 구간이 2개이기 때문에 key 같은 것이 필요함 
          // curveCnt // 차선 수  : 
          // curveCnt % 차선수 : 
          curve = true;
          curveNum = FIRSTLANE;
          break;
        }
        curveCnt++;
      }
      if(curveNum != FIRSTLANE) 
      {
        curveCnt = INITCOUNT;
        for (const auto &lane : curvePrimitives_2)
        {
          if (currentLaneletId == lane)
          {
            // topic publish 
            curve = true;
            curveNum = SECONDLANE;
            break;
          }
          curveCnt++;
        }
      }
    }

    long int cnt = INITCOUNT;
    for (const auto &lane_val : available_lanelets)
    {
      laneletKey = cnt / laneNum; 
      if (currentLaneletId == lane_val && currentLaneletId != lastLaneletId && laneletKey != lastLaneletKey)
      {
        lastLaneletId = currentLaneletId;
        lastLaneletKey = laneletKey;
        pubState = TrajectoryRouter::NORMAL;
        currentPrimitive2DVector = setPrimitiveVector(primitive2DVector[laneletKey], laneNum);
        break;
      }
      cnt++;
    }

    switch (pubState)
    {
    case TrajectoryRouter::READY: break;
    // NORMAL 상태는 계속 주행하는 상태
    // NORMAL이 끝나면 READY 상태가 됨
    case TrajectoryRouter::NORMAL:
    {
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

      for (const auto &out_lane_id : currentPrimitive2DVector)
      {
        segment = emptySegment;
        segment.preferred_primitive.id = out_lane_id[desiredLane[laneletKey]];
        segment.preferred_primitive.primitive_type = "";
        for (const auto &in_lane_id : out_lane_id)
        {
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
      std::cout << "NORMAL trajectory publish" << std::endl;
      break;
    }
    case TrajectoryRouter::GO:
      break;
    // STOP 상태는 ego 위치를 이용해 경로를 생성하지 않는 상태 
    // STOP 상태를 계속 유지
    // STOP인 경우에는 뱅크 끝까지 경로 생성 해야 함
    case TrajectoryRouter::STOP:
    {
      autoware_planning_msgs::msg::LaneletRoute route_msg_;
      route_msg_.header.stamp = this->get_clock()->now();
      route_msg_.header.frame_id = "map";
      route_msg_.start_pose = msg->pose;
      if (curve)
      {
        switch (curveNum)
        {
        case FIRSTLANE:
          route_msg_.goal_pose.position.x = curve1_position_x[curveCnt%laneNum]; 
          route_msg_.goal_pose.position.y = curve1_position_y[curveCnt%laneNum]; 
          route_msg_.goal_pose.position.z = curve_position_z;
            
          route_msg_.goal_pose.orientation.x = curve_orientation_x;
          route_msg_.goal_pose.orientation.y = curve_orientation_y;
          route_msg_.goal_pose.orientation.z = curve1_orientation_z;  
          route_msg_.goal_pose.orientation.w = curve1_orientation_w;
          
          for (const auto &out_lane_id : curvePrimitive2DVector_1)
          {
            segment = emptySegment;
            segment.preferred_primitive.id = out_lane_id[curveCnt%laneNum];
            segment.preferred_primitive.primitive_type = "";
            for (const auto &in_lane_id : out_lane_id)
            {
              if (in_lane_id == NOLANELET) continue;
              primitive.id = in_lane_id;
              primitive.primitive_type = "lane";
              segment.primitives.push_back(primitive);
            }
            route_msg_.segments.push_back(segment);
          }
          break;
        case SECONDLANE:
          route_msg_.goal_pose.position.x = curve2_position_x[curveCnt%laneNum]; 
          route_msg_.goal_pose.position.y = curve2_position_y[curveCnt%laneNum]; 
          route_msg_.goal_pose.position.z = curve_position_z;
            
          route_msg_.goal_pose.orientation.x = curve_orientation_x;
          route_msg_.goal_pose.orientation.y = curve_orientation_y;
          route_msg_.goal_pose.orientation.z = curve2_orientation_z;  
          route_msg_.goal_pose.orientation.w = curve2_orientation_w;
          
          for (const auto &out_lane_id : curvePrimitive2DVector_2)
          {
            segment = emptySegment;
            segment.preferred_primitive.id = out_lane_id[curveCnt%laneNum];
            segment.preferred_primitive.primitive_type = "";
            for (const auto &in_lane_id : out_lane_id)
            {
              if (in_lane_id == NOLANELET) continue;
              primitive.id = in_lane_id;
              primitive.primitive_type = "lane";
              segment.primitives.push_back(primitive);
            }
            route_msg_.segments.push_back(segment);
          }
          break;
        default:
          RCLCPP_WARN(get_logger(), "Stop trajectort route falut!");
          break;
        }
      }
      else
      {
        route_msg_.goal_pose = msg->pose;
        segment = emptySegment;
        segment.preferred_primitive.id = currentLaneletId;
        segment.preferred_primitive.primitive_type = "";
        primitive.id = currentLaneletId;
        primitive.primitive_type = "lane";
        segment.primitives.push_back(primitive);
        route_msg_.segments.push_back(segment);
      }
      route_msg_.uuid.uuid = {209, 239, 15, 91, 197, 87, 68, 179, 62, 19, 3, 36, 111, 114, 35, 231};
      route_pub_->publish(route_msg_);
      std::cout << "STOP trajectory publish" << std::endl;
      break;
    }
    case TrajectoryRouter::PITSTOP:
      break;
    default:
      break;
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
