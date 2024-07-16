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

class TrajectoryRouter : public rclcpp::Node
{
public:
  TrajectoryRouter() : Node("trajectory_router")
  {
    // subscription topic 
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>( "/localization/pose_estimator/pose", rclcpp::QoS{1}, std::bind(&TrajectoryRouter::callbackPose, this, std::placeholders::_1));
    sub_vector_map_ = create_subscription<HADMapBin>("/map/vector_map", rclcpp::QoS(1).transient_local(), std::bind(&TrajectoryRouter::callbackMap, this, std::placeholders::_1));

    // publisher topic 
    route_pub_= create_publisher<autoware_planning_msgs::msg::LaneletRoute>("/planning/mission_planning/route", rclcpp::QoS{1}.transient_local());

    // init
    currentLaneletId = INITLANELET;
    laneletKey = currentLaneletId;
    laneletState = false;
    // declare param
    this->declare_parameter<std::vector<long int>>("available_lanelet_id", std::vector<long int>({}));

    // get param  -> onmapcallback ?
    available_lanelets = this->get_parameter("available_lanelet_id").as_integer_array();
  }
  // lanelet ID check varilable 
  long int currentLaneletId;
  long int laneletKey;
  bool laneletState;

  // map variable
  lanelet::LaneletMapPtr TR_lanelet_map_ptr_;
  HADMapBin::ConstSharedPtr TR_map_ptr_;
  
  // lanelets variable
  lanelet::ConstLanelets all_lanelets;
  lanelet::ConstLanelets road_lanelets_;

  // trajectory  table
  std::vector<long int> available_lanelets;

  // callback function 
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_vector_map_;

  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_pub_;
private:
  void callbackPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_WARN(get_logger(), "start callbackPose !! ");
    lanelet::ConstLanelets start_lanelets;

    if (currentLaneletId == INITLANELET)
      return;

    if (!lanelet::utils::query::getCurrentLanelets(road_lanelets_, msg->pose, &start_lanelets))
      RCLCPP_WARN(get_logger(), "get lanelet id fault!");

    for (const auto & st_llt : start_lanelets) {
      RCLCPP_WARN(get_logger(), "get current lanelet id -> %ld", st_llt.id());
      currentLaneletId = st_llt.id();
    }

    // lanelet check currentLaneletId in available_lanelets
    // 차선 변경해야하는 lanelet 위에 차량이 존재하는 경우
    for (const auto &lane_val : available_lanelets){
      if (currentLaneletId == lane_val){
        RCLCPP_WARN(get_logger(), "Ready trajectory");
        laneletKey = currentLaneletId;
        laneletState = true;
        break;
      }
    }

    if(laneletState){
      if(currentLaneletId == 13694){
        autoware_planning_msgs::msg::LaneletRoute route_msg_;
        route_msg_.header.stamp = this->get_clock()->now();
        route_msg_.header.frame_id = "map";
        route_msg_.start_pose = msg->pose;

        // map<>을 이용해 goal_pose를 정의해 놓을 수 있을 것으로 보임 
        // goal_pose를 map에 저장하는 과정은 map callback 함수에서 하는 것이 적절해 보임
        // yaml 파일을 생성해서 바로 접근 할 수 있도록 셋업해야할 것으로 보임
        route_msg_.goal_pose.position.x = 286.3343811035156;
        route_msg_.goal_pose.position.y = 366.3512878417969;
        route_msg_.goal_pose.position.z = 0.0;
        
        route_msg_.goal_pose.orientation.x = 0.0;
        route_msg_.goal_pose.orientation.y = 0.0;
        route_msg_.goal_pose.orientation.z = -0.1296844144431665;
        route_msg_.goal_pose.orientation.w = 0.9915553200152439;

        // 2중 for 문을 이용해서 채워야 할 것으로 보임
        // 적절한 경로를 큰 루프 후보 경로를 작은 루프 
        // 해당 lanelet id 역시 yaml 파일 이용
        autoware_planning_msgs::msg::LaneletSegment segment;
        segment.preferred_primitive.id = currentLaneletId;
        segment.preferred_primitive.primitive_type = "";

        autoware_planning_msgs::msg::LaneletPrimitive primitive;
        primitive.id = currentLaneletId;
        primitive.primitive_type = "lane";
        segment.primitives.push_back(primitive);
        route_msg_.segments.push_back(segment);


        segment.preferred_primitive.id = 28967;
        segment.preferred_primitive.primitive_type = "";
        segment.primitives.pop_back();
        primitive.id = 28967;
        primitive.primitive_type = "lane";
        segment.primitives.push_back(primitive);
        route_msg_.segments.push_back(segment);


        segment.preferred_primitive.id = 13826;
        segment.preferred_primitive.primitive_type = "";
        segment.primitives.pop_back();
        primitive.id = 13826;
        primitive.primitive_type = "lane";
        segment.primitives.push_back(primitive);
        route_msg_.segments.push_back(segment);

        segment.preferred_primitive.id = 9688;
        segment.preferred_primitive.primitive_type = "";
        segment.primitives.pop_back();
        primitive.id = 9688;
        primitive.primitive_type = "lane";
        segment.primitives.push_back(primitive);
        route_msg_.segments.push_back(segment);

        // uuid는 적절한 걸로 생성
        route_msg_.uuid.uuid = {209, 239, 15, 91, 197, 87, 68, 179, 62, 19, 3, 36, 111, 114, 35, 231};
      }
      route_pub_->publish(route_msg_);
      laneletState = false;
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
