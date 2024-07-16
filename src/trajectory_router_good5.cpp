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
// using VecLongInt = std::vector<long int>;
class TrajectoryRouter : public rclcpp::Node
{
public:
  TrajectoryRouter() : Node("trajectory_router")
  {
    // 지역 변수 primitive
    std::vector<long int> primitive;

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>( "/localization/pose_estimator/pose", rclcpp::QoS{1}, std::bind(&TrajectoryRouter::callbackPose, this, std::placeholders::_1));
    sub_vector_map_ = create_subscription<HADMapBin>("/map/vector_map", rclcpp::QoS(1).transient_local(), std::bind(&TrajectoryRouter::callbackMap, this, std::placeholders::_1));
    route_pub_= create_publisher<autoware_planning_msgs::msg::LaneletRoute>("/planning/mission_planning/route", rclcpp::QoS{1}.transient_local());

    // 초기화 
    currentLaneletId = INITLANELET;
    lastLaneletId = INITLANELET;
    pubState = false;


    // declare param
    this->declare_parameter<std::vector<long int>>("available_lanelet_id", std::vector<long int>({}));
    this->declare_parameter<std::vector<long int>>("primitive_0", std::vector<long int>({}));
    this->declare_parameter<std::vector<long int>>("primitive_1", std::vector<long int>({}));
    this->declare_parameter<std::vector<long int>>("primitive_2", std::vector<long int>({}));
    this->declare_parameter<std::vector<long int>>("primitive_3", std::vector<long int>({}));
    this->declare_parameter<std::vector<long int>>("primitive_4", std::vector<long int>({}));
    this->declare_parameter<std::vector<long int>>("primitive_5", std::vector<long int>({}));

    // get param  -> onmapcallback ?
    available_lanelets = this->get_parameter("available_lanelet_id").as_integer_array();
    
    this->declare_parameter<int>("lane_number", int());
    laneNum =  this->get_parameter("lane_number").as_int();

    primitive = this->get_parameter("primitive_0").as_integer_array(); // primitive # 으로 for loop 
    primitive2DVector_0 = setPrimitiveVector(primitive, laneNum); 

    primitive = this->get_parameter("primitive_1").as_integer_array();
    primitive2DVector_1 = setPrimitiveVector(primitive, laneNum); 
    primitive = this->get_parameter("primitive_2").as_integer_array();
    primitive2DVector_2 = setPrimitiveVector(primitive, laneNum);
    primitive = this->get_parameter("primitive_3").as_integer_array();
    primitive2DVector_3 = setPrimitiveVector(primitive, laneNum); 
    primitive = this->get_parameter("primitive_4").as_integer_array();
    primitive2DVector_4 = setPrimitiveVector(primitive, laneNum); 
    primitive = this->get_parameter("primitive_5").as_integer_array();
    primitive2DVector_5 = setPrimitiveVector(primitive, laneNum); 

    // primitive = this->get_parameter("primitive_2").as_integer_array();
    

    // 0705
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
  int laneNum;
  // lanelet ID check varilable 
  long int currentLaneletId;
  long int lastLaneletId;
  long int laneletKey;
  bool pubState;

  // map variable
  lanelet::LaneletMapPtr TR_lanelet_map_ptr_;
  HADMapBin::ConstSharedPtr TR_map_ptr_;
  
  // lanelets variable
  lanelet::ConstLanelets all_lanelets;
  lanelet::ConstLanelets road_lanelets_;

  // trajectory  table
  std::vector<long int> available_lanelets;

  std::vector<std::vector<long int>> currentPrimitive2DVector;
  std::vector<std::vector<long int>> primitive2DVector_0;
  std::vector<std::vector<long int>> primitive2DVector_1;
  std::vector<std::vector<long int>> primitive2DVector_2;
  std::vector<std::vector<long int>> primitive2DVector_3;
  std::vector<std::vector<long int>> primitive2DVector_4;
  std::vector<std::vector<long int>> primitive2DVector_5;

  // goal pose 
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
private:
  std::vector<std::vector<long int>> setPrimitiveVector(const std::vector<long int>& vec, size_t laneletNum)
  {
    std::vector<std::vector<long int>> return2DVec;
    return2DVec.reserve((vec.size() + laneletNum - 1) / laneletNum); 
    auto nowVec = vec.begin();
    while (nowVec != vec.end()){
      std::vector<long int> chunk;
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
    RCLCPP_WARN(get_logger(), "CallbackPose Start !! ");
    lanelet::ConstLanelets start_lanelets;

    if (currentLaneletId == INITLANELET)
      return;

    if (!lanelet::utils::query::getCurrentLanelets(road_lanelets_, msg->pose, &start_lanelets))
      RCLCPP_WARN(get_logger(), "get lanelet id fault!");

    for (const auto & st_llt : start_lanelets) {
      currentLaneletId = st_llt.id();
    }

    // lanelet check currentLaneletId in available_lanelets
    // 차선 변경해야하는 lanelet 위에 차량이 존재하는 경우
    long int cnt = 0;
    for (const auto &lane_val : available_lanelets){
      if (currentLaneletId == lane_val){
        if(currentLaneletId != lastLaneletId){ // 조건을 더 추가해서 같은 key 값을 가질때 차선변경하는 경우도 처리해야 한다. 같은 key에서 한번만 발행 할 수 있도록 설계해야한다. 
          lastLaneletId = currentLaneletId;
          pubState = true;
        }
        laneletKey = cnt / laneNum; 
        switch (laneletKey){
          case 0: currentPrimitive2DVector = primitive2DVector_0; break;
          case 1: currentPrimitive2DVector = primitive2DVector_1; break;
          case 2: currentPrimitive2DVector = primitive2DVector_2; break;
          case 3: currentPrimitive2DVector = primitive2DVector_3; break;
          case 4: currentPrimitive2DVector = primitive2DVector_4; break;
          case 5: currentPrimitive2DVector = primitive2DVector_5; break;
        }
        // RCLCPP_WARN(get_logger(), "key = %ld, first id = %ld", laneletKey, currentPrimitive2DVector[0][0]);
        break;
      }
      cnt++;
    }

    // std::cout << "start laneler id 2d loop" << std::endl;
    // for (const auto &out_lane_id : currentPrimitive2DVector){
    //   for (const auto &in_lane_id : out_lane_id){
    //     std::cout << in_lane_id << std::endl;
    //   }
    // }
    // std::cout << "== 2d loop Done == " << std::endl;
    // debug 
    // for (const auto &i : currentPrimitive2DVector){
    //   for (const auto &j : i){
    //     RCLCPP_WARN(get_logger(), "primitive id -> %ld", j);
    //   }
    // }

    // topic 채우는 부분 
    if(pubState){
      autoware_planning_msgs::msg::LaneletRoute route_msg_;
      route_msg_.header.stamp = this->get_clock()->now();
      route_msg_.header.frame_id = "map";
      route_msg_.start_pose = msg->pose;

        // map<>을 이용해 goal_pose를 정의해 놓을 수 있을 것으로 보임 
        // goal_pose를 map에 저장하는 과정은 map callback 함수에서 하는 것이 적절해 보임
        // yaml 파일을 생성해서 바로 접근 할 수 있도록 셋업해야할 것으로 보임
      std::cout << cnt << std::endl;
      route_msg_.goal_pose.position.x = position2D_x[cnt/laneNum][0]; // 2차원으로 만들어야 한다. cnt -> [cnt / 4][원하는 차선]
      route_msg_.goal_pose.position.y = position2D_y[cnt/laneNum][0]; // 일단은 1차선으로 갈수 있도록 설계 
      route_msg_.goal_pose.position.z = position_z;
        
      route_msg_.goal_pose.orientation.x = orientation_x;
      route_msg_.goal_pose.orientation.y = orientation_y;
      route_msg_.goal_pose.orientation.z = orientation_z[cnt/laneNum]; // 얘는 그대로 
      route_msg_.goal_pose.orientation.w = orientation_w[cnt/laneNum];

        // 2중 for 문을 이용해서 채워야 할 것으로 보임
        // 적절한 경로를 큰 루프 후보 경로를 작은 루프 
        // 해당 lanelet id 역시 yaml 파일 이용
      for (const auto &out_lane_id : currentPrimitive2DVector){
        segment = emptySegment;
        segment.preferred_primitive.id = out_lane_id[0]; // goal 차선이랑 동일한 차선의 id 필요
        segment.preferred_primitive.primitive_type = "";
        for (const auto &in_lane_id : out_lane_id){
          if (in_lane_id == NOLANELET) continue;
          primitive.id = in_lane_id;
          primitive.primitive_type = "lane";
          segment.primitives.push_back(primitive);
        }
        route_msg_.segments.push_back(segment);
      }

      // uuid는 적절한 걸로 생성
      route_msg_.uuid.uuid = {209, 239, 15, 91, 197, 87, 68, 179, 62, 19, 3, 36, 111, 114, 35, 231};
      route_pub_->publish(route_msg_);
      pubState = false;
      std::cout << "pub trajectory" << std::endl;
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
