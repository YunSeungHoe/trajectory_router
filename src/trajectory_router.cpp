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
#include <kiapi_msg/msg/planner_flag.hpp>
#include <avante_msgs/msg/deceleration_zone.hpp>
#include <mutex>


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
    pitstopFlag = false;
    curveState = false;
    lastCurveState = true;
    curveCnt = INITCOUNT;
    curveNum = INITCOUNT;
    pubState = TrajectoryRouter::READY;
    canState = TrajectoryRouter::READY;
    // CANSignal = TrajectoryRouter::READY;

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>( "/localization/pose_estimator/pose", rclcpp::QoS{1}, std::bind(&TrajectoryRouter::callbackPose, this, std::placeholders::_1));
    signal_sub_ = create_subscription<kiapi_msg::msg::PlannerFlag>( "/kiapi_flag_status", rclcpp::QoS{1}, std::bind(&TrajectoryRouter::callbackSignal, this, std::placeholders::_1));
    sub_vector_map_ = create_subscription<HADMapBin>("/map/vector_map", rclcpp::QoS(1).transient_local(), std::bind(&TrajectoryRouter::callbackMap, this, std::placeholders::_1));
    route_pub_= create_publisher<autoware_planning_msgs::msg::LaneletRoute>("/planning/mission_planning/route", rclcpp::QoS{1}.transient_local());
    deceleration_pub_= create_publisher<avante_msgs::msg::DecelerationZone>("/planning/deceleration_zone", rclcpp::QoS{1});

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
    this->declare_parameter<double>("orientation.x", double());
    this->declare_parameter<double>("orientation.y", double());
    this->declare_parameter<std::vector<double>>("orientation.z", std::vector<double>({}));
    this->declare_parameter<std::vector<double>>("orientation.w", std::vector<double>({}));

    position_x = this->get_parameter("position.x").as_double_array();
    position2D_x = setPrimitiveVector(position_x, laneNum);
    position_y = this->get_parameter("position.y").as_double_array();
    position2D_y = setPrimitiveVector(position_y, laneNum);
    position_z = this->get_parameter("position.z").as_double();
    orientation_x = this->get_parameter("orientation.x").as_double();
    orientation_y = this->get_parameter("orientation.y").as_double();
    orientation_z = this->get_parameter("orientation.z").as_double_array();
    orientation_w = this->get_parameter("orientation.w").as_double_array();
    
    this->declare_parameter<LongIntVec>("curve_primitive", LongIntVec({}));
    curvePrimitives = this->get_parameter("curve_primitive").as_integer_array();
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

    this->declare_parameter<double>("curve.orientation.x", double());
    curve_orientation_x = this->get_parameter("curve.orientation.x").as_double();
    this->declare_parameter<double>("curve.orientation.y", double());
    curve_orientation_y = this->get_parameter("curve.orientation.y").as_double();
    this->declare_parameter<double>("curve_1.orientation.z", double());
    curve1_orientation_z = this->get_parameter("curve_1.orientation.z").as_double();
    this->declare_parameter<double>("curve_1.orientation.w", double());
    curve1_orientation_w = this->get_parameter("curve_1.orientation.w").as_double();
    this->declare_parameter<double>("curve_2.orientation.z", double());
    curve2_orientation_z = this->get_parameter("curve_2.orientation.z").as_double();
    this->declare_parameter<double>("curve_2.orientation.w", double());
    curve2_orientation_w = this->get_parameter("curve_2.orientation.w").as_double();

    // go 
    this->declare_parameter<LongIntVec>("go_primitive", LongIntVec({}));
    goPrimitives = this->get_parameter("go_primitive").as_integer_array();

    this->declare_parameter<double>("go.position.x", double());
    go_position_x = this->get_parameter("go.position.x").as_double();
    this->declare_parameter<double>("go.position.y", double());
    go_position_y = this->get_parameter("go.position.y").as_double();
    this->declare_parameter<double>("go.position.z", double());
    go_position_z = this->get_parameter("go.position.z").as_double();
    this->declare_parameter<double>("go.orientation.x", double());
    go_orientation_x = this->get_parameter("go.orientation.x").as_double();
    this->declare_parameter<double>("go.orientation.y", double());
    go_orientation_y = this->get_parameter("go.orientation.y").as_double();
    this->declare_parameter<double>("go.orientation.z", double());
    go_orientation_z = this->get_parameter("go.orientation.z").as_double();
    this->declare_parameter<double>("go.orientation.w", double());
    go_orientation_w = this->get_parameter("go.orientation.w").as_double();

    //pit stop
    this->declare_parameter<LongIntVec>("pitstop_primitive", LongIntVec({}));
    pitstopPrimitives = this->get_parameter("pitstop_primitive").as_integer_array();
    pitstopPrimitive2DVector = setPrimitiveVector(pitstopPrimitives, laneNum);

    this->declare_parameter<std::vector<double>>("pitstop.position.x", std::vector<double>({}));
    pitstop_position_x = this->get_parameter("pitstop.position.x").as_double_array();
    this->declare_parameter<std::vector<double>>("pitstop.position.y", std::vector<double>({}));
    pitstop_position_y = this->get_parameter("pitstop.position.y").as_double_array();
    this->declare_parameter<double>("pitstop.position.z", double());
    pitstop_position_z = this->get_parameter("pitstop.position.z").as_double();

    this->declare_parameter<double>("pitstop.orientation.x", double());
    pitstop_orientation_x = this->get_parameter("pitstop.orientation.x").as_double();
    this->declare_parameter<double>("pitstop.orientation.y", double());
    pitstop_orientation_y = this->get_parameter("pitstop.orientation.y").as_double();
    this->declare_parameter<double>("pitstop.orientation.z", double());
    pitstop_orientation_z = this->get_parameter("pitstop.orientation.z").as_double();
    this->declare_parameter<double>("pitstop.orientation.w", double());
    pitstop_orientation_w = this->get_parameter("pitstop.orientation.w").as_double();
  }
  lanelet::LaneletMapPtr TR_lanelet_map_ptr_;
  HADMapBin::ConstSharedPtr TR_map_ptr_;
  lanelet::ConstLanelets all_lanelets;
  lanelet::ConstLanelets road_lanelets_;
  autoware_planning_msgs::msg::LaneletSegment segment;
  autoware_planning_msgs::msg::LaneletSegment emptySegment;
  autoware_planning_msgs::msg::LaneletPrimitive primitive;

  std::mutex signalMutex;

  std::vector<std::vector<double>> position2D_x;
  std::vector<std::vector<double>> position2D_y;
  std::vector<LongIntVec> currentPrimitive2DVector;
  std::vector<LongIntVec> primitive2DVector;
  std::vector<LongIntVec> curvePrimitive2DVector_1;
  std::vector<LongIntVec> curvePrimitive2DVector_2;
  std::vector<LongIntVec> pitstopPrimitive2DVector;
  std::vector<double> position_x;
  std::vector<double> position_y;
  std::vector<double> orientation_z;
  std::vector<double> orientation_w;
  std::vector<double> curve1_position_x;
  std::vector<double> curve1_position_y;
  std::vector<double> curve2_position_x;
  std::vector<double> curve2_position_y;
  std::vector<double> pitstop_position_x;
  std::vector<double> pitstop_position_y;
  LongIntVec available_lanelets;
  LongIntVec desiredLane;
  LongIntVec curvePrimitives;
  LongIntVec curvePrimitives_1;
  LongIntVec curvePrimitives_2;
  LongIntVec goPrimitives;
  LongIntVec pitstopPrimitives;
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
  double go_position_x;
  double go_position_y;
  double go_position_z;
  double go_orientation_x;
  double go_orientation_y;
  double go_orientation_z;
  double go_orientation_w;
  double pitstop_position_z;
  double pitstop_orientation_x;
  double pitstop_orientation_y;
  double pitstop_orientation_z;
  double pitstop_orientation_w;
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
  bool pitstopFlag;

  bool curveState;
  bool lastCurveState;
  // PubState CANSignal;
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<kiapi_msg::msg::PlannerFlag>::SharedPtr signal_sub_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_pub_;
  rclcpp::Publisher<avante_msgs::msg::DecelerationZone>::SharedPtr deceleration_pub_;
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

  void callbackSignal(const kiapi_msg::msg::PlannerFlag msg)
  {
    signalMutex.lock();
    switch (msg.flag)
    {
    case kiapi_msg::msg::PlannerFlag::READY:
      canState = TrajectoryRouter::READY;
      // std::cout << "READY" << std::endl;
      break;
    case kiapi_msg::msg::PlannerFlag::GO:
      canState = TrajectoryRouter::GO;
      std::cout << "GO" << std::endl;
      break;    
    case kiapi_msg::msg::PlannerFlag::SLOW_ON:
      canState = TrajectoryRouter::SLOWON;
      std::cout << "SLOW_ON" << std::endl;
      break;    
    case kiapi_msg::msg::PlannerFlag::SLOW_OFF:
      std::cout << "SLOW_OFF" << std::endl;
      break;    
    case kiapi_msg::msg::PlannerFlag::STOP:
      canState = TrajectoryRouter::STOP;
      std::cout << "STOP" << std::endl;
      break;    
    case kiapi_msg::msg::PlannerFlag::PIT_STOP:
      canState = TrajectoryRouter::PITSTOP;
      std::cout << "PIT_STOP" << std::endl;
      break;    
    default:
      std::cout << "error" << std::endl;
      break;
    }
    signalMutex.unlock();
  }

  void callbackPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    lanelet::ConstLanelets start_lanelets;

    if (currentLaneletId == INITLANELET || pubState == TrajectoryRouter::STOP) return;

    if (!lanelet::utils::query::getCurrentLanelets(road_lanelets_, msg->pose, &start_lanelets))
      RCLCPP_WARN(get_logger(), "get lanelet id fault!");

    // 주도로의 lanelet id를 작게해서 획득할 필요가 있음
    for (const auto & st_llt : start_lanelets)
    {
      currentLaneletId = st_llt.id();
      // std::cout << currentLaneletId << std::endl;
    }
    // Lanelet이 2개인 구간에서 디버깅 필요 
    // std::cout << "current lanelet ID: " << currentLaneletId << std::endl; 
    signalMutex.lock();
    if (canState == TrajectoryRouter::STOP || canState == TrajectoryRouter::SLOWON)
    {
      curveCnt = INITCOUNT;
      curveNum = INITCOUNT;
      pubState = canState;
      if (pubState == TrajectoryRouter::SLOWON) pubState = TrajectoryRouter::READY;
      for (const auto &lane : curvePrimitives_1)
      {
        if (currentLaneletId == lane)
        {
          // topic publish 
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
    if (canState == TrajectoryRouter::GO)
    {
      pubState = TrajectoryRouter::GO;
      canState = TrajectoryRouter::READY;
    }
    if (canState == TrajectoryRouter::PITSTOP && !pitstopFlag)
    {
      pitstopFlag = true;
      canState = TrajectoryRouter::READY;
    }
    signalMutex.unlock();

    // 이걸 찾는 조건문도 switch 사용해야 하나? 
    long int cnt = INITCOUNT;
    for (const auto &lane_val : available_lanelets)
    {
      laneletKey = cnt / laneNum; 
      if (currentLaneletId == lane_val && currentLaneletId != lastLaneletId && laneletKey != lastLaneletKey)
      {
        lastLaneletId = currentLaneletId;
        lastLaneletKey = laneletKey;
        if (pitstopFlag && laneletKey == 0)
        {
          pubState = TrajectoryRouter::PITSTOP;
          std::cout << "pubState is stop " << std::endl;
        } 
        else pubState = TrajectoryRouter::NORMAL;
        currentPrimitive2DVector = setPrimitiveVector(primitive2DVector[laneletKey], laneNum);
        break;
      }
      cnt++;
    }

    // curve 구간 진입 -> false / 나올때 -> true
    for (const auto &curve_lane : curvePrimitives)
    {
      if(currentLaneletId == curve_lane)
      {
        curveState = true;
        break;
      }
      else curveState = false;
    }

    if (lastCurveState != curveState)
    {
      avante_msgs::msg::DecelerationZone deceleration_msg_;
      deceleration_msg_.deceleration_zone = !curveState;
      deceleration_pub_ -> publish(deceleration_msg_);
      lastCurveState = curveState;
    }

    // 상태에 따른 적절한 경로 생성 
    switch (pubState)
    {
    case TrajectoryRouter::READY: break;
    case TrajectoryRouter::NORMAL:
    {
      std::cout << "NORMAL Start" << std::endl;
      long int row;
      long int col;
      long int thirdlane;
      thirdlane = 2;
      
      row = cnt/laneNum;
      if (pitstopFlag)
      {
        col = thirdlane;
      }
      else
      {
        col = desiredLane[laneletKey];
        if (desiredLane[laneletKey] == thirdlane)
          desiredLane[laneletKey] = desiredLane[laneletKey] - 1; 
      }
      autoware_planning_msgs::msg::LaneletRoute route_msg_;
      route_msg_.header.stamp = this->get_clock()->now();
      route_msg_.header.frame_id = "map";
      route_msg_.start_pose = msg->pose;

      route_msg_.goal_pose.position.x = position2D_x[row][col]; 
      route_msg_.goal_pose.position.y = position2D_y[row][col]; 
      route_msg_.goal_pose.position.z = position_z;
      
      route_msg_.goal_pose.orientation.x = orientation_x;
      route_msg_.goal_pose.orientation.y = orientation_y;
      route_msg_.goal_pose.orientation.z = orientation_z[row];  
      route_msg_.goal_pose.orientation.w = orientation_w[row];

      for (const auto &out_lane_id : currentPrimitive2DVector)
      {
        segment = emptySegment;
        segment.preferred_primitive.id = out_lane_id[col];
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
    {
      autoware_planning_msgs::msg::LaneletRoute route_msg_;
      route_msg_.header.stamp = this->get_clock()->now();
      route_msg_.header.frame_id = "map";
      route_msg_.start_pose = msg->pose;
      
      route_msg_.goal_pose.position.x = go_position_x; 
      route_msg_.goal_pose.position.y = go_position_y; 
      route_msg_.goal_pose.position.z = go_position_z;
      
      std::cout << go_orientation_z << std::endl;
      std::cout << go_orientation_w << std::endl;
      route_msg_.goal_pose.orientation.x = go_orientation_x;
      route_msg_.goal_pose.orientation.y = go_orientation_y;
      route_msg_.goal_pose.orientation.z = go_orientation_z;  
      route_msg_.goal_pose.orientation.w = go_orientation_w;
      // 벡터 맨 앞에 요소 추가 
      goPrimitives.insert(goPrimitives.begin(), currentLaneletId);

      for (const auto &lane : goPrimitives)
      {
        segment = emptySegment;
        segment.preferred_primitive.id = lane;
        segment.preferred_primitive.primitive_type = "";
        primitive.id = lane;
        primitive.primitive_type = "lane";
        segment.primitives.push_back(primitive);
        route_msg_.segments.push_back(segment);
      }
      route_msg_.uuid.uuid = {209, 239, 15, 91, 197, 87, 68, 179, 62, 19, 3, 36, 111, 114, 35, 231};
      route_pub_->publish(route_msg_);

      pubState = TrajectoryRouter::READY;
      std::cout << "GO trajectory publish" << std::endl;
      break;
    }
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
    {
      if(!pitstopFlag) break;
      autoware_planning_msgs::msg::LaneletRoute route_msg_;
      route_msg_.header.stamp = this->get_clock()->now();
      route_msg_.header.frame_id = "map";
      route_msg_.start_pose = msg->pose;

      route_msg_.goal_pose.position.x = pitstop_position_x[1]; 
      route_msg_.goal_pose.position.y = pitstop_position_y[1]; 
      route_msg_.goal_pose.position.z = pitstop_position_z;
            
      route_msg_.goal_pose.orientation.x = pitstop_orientation_x;
      route_msg_.goal_pose.orientation.y = pitstop_orientation_y;
      route_msg_.goal_pose.orientation.z = pitstop_orientation_z;  
      route_msg_.goal_pose.orientation.w = pitstop_orientation_w;
      
      for (const auto &out_lane_id : pitstopPrimitive2DVector)
      {
        segment = emptySegment;
        segment.preferred_primitive.id = out_lane_id[1];
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
      pitstopFlag = false;
      std::cout << "PIT STOP trajectory publish" << std::endl;
      break;
    }
    default: break;
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
