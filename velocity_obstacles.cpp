#include <cmath>
#include <iostream>
#include <fstream>
#include <string>

#include <velocity_obstacles/velocity_obstacles.hpp>
#include "rclcpp/rclcpp.hpp"
#include <motion_common/motion_common.hpp>



#include <tf2/convert.h>
#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>

#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

using Point = geometry_msgs::msg::Point;
using ColorRGBA = std_msgs::msg::ColorRGBA;

using motion::motion_common::to_angle;
using motion::motion_common::from_angle;
using namespace std;

struct circle {//NLVO Circles

  float x_position;
  float y_position;
  float radius;

  size_t obstacl_num;
  float time;
  size_t label;
  BoundingBox box;
};

struct optional_velocitys{
  TrajectoryPoint velocity;
  bool optional_nlvo = true; // True - Speed is allowed in terms of collision with an obstacle
  bool optional_border = true; //True - Speed is allowed in terms of collision at the curb
  float min_time_horizon = 100; // dont tuch
  float cost;
  int acceleration_size = 0; //Represents the amount of acceleration required to reach this speed. 1-max, 0-non, -1 min.
  int maneuvering_side = 2; //To the right =1, to the left =-1, no maneuver =0
  float branch_cost;
};

//Centralizes all the necessary parameters in the algorithm.
struct parameters{
  float acceleration = 10; //mps^2
  float braking = -10;
  float max_braking = 25; //m/s^2
  float dt_controller = 0.04; //sec, 25hz
  float border_collision_time = 1; //sec
  float car_radius = 3.0;//4; // radius of neighboring cars = radius_ego + radius_cars = 2+2 =4 (meters)
  float dt = 0.04; //sec
  // float min_time_horizon = 0.2; // sec
  float min_time_horizon = 0.1;    // sec
  // float max_time_horizon = 3;   // sec
  float max_time_horizon = 2;      // sec
  int destination_index = 90;

  float d_teta = 0.01; //The resolution of the angle of velocity in the controller in radians
  int velocity_num_in_controler_accel = 3;// An odd number is required
  float friction_coefficient = 1.2; //friction coefficient of the wheel
  float max_velocity = 100;// m/s
  int num_of_points_controller = 2; //The number of speeds in the controller
  
  


  //pure pursit param
  float L = 5.0F;//3.0F; //vehicle length
  float min_lookahead_dist = 5.0F;// minimal lookahead distance (l_d). (5.0F = 5 meter as float type)
  float max_lookahead_dist = 50.0F;// maximal lookahead distance
  float max_steer_angle = 1.0F;//0.33F;//limit of steering angle [rad]
  float k = 0.5F; // l_d = velocity*k


  //option tree
  //5 velociteis in each branch
  int max_tree_level = 5;
  int multi = 3;
  float step_time = dt * multi;
  //Percentage difference between last point in the industry and target point
  float percent_diff_branch_cost = 0.01;
  //Number of speeds / branches at one level. Plus one of the speed selected by the controller
  int num_velocity_in_level = 2;

};

/*
sudo chmod 666 /var/run/docker.sock
cd adehome/AutowareAuto
ade --rc .aderc-amd64-foxy start --update --enter

ade enter
cd AutowareAuto
source install/setup.bash

colcon build --packages-select velocity_obstacles
colcon build --packages-select file_trajectory_planner
colcon build --packages-select minimal_simulator
colcon build --packages-select trajectories_predictor

choose one of the following:
ros2 launch minimal_simulator multi_simulator.launch.py   optional: auto_sync_mode:=True
ros2 launch minimal_simulator two_vehicles_simulator.launch.py
ros2 launch minimal_simulator two_vehicles_simulator.launch_scenario1.py
ros2 launch minimal_simulator two_vehicles_simulator.launch_scenario2.py
ros2 launch minimal_simulator four_vehicles_simulator.launch.py

ros2 launch velocity_obstacles velocity_obstacles.launch.py

ros2 launch velocity_obstacles velocity_obstacles_visual.launch.py

// one move
ros2 topic pub --once /dds_done_reply std_msgs/ByteMultiArray 

// 0.1 sec for move in row
ros2 topic pub /dds_done_reply std_msgs/ByteMultiArray 

// auto run
ros2 launch simulation_sync simulation_sync.launch.py 


////////////////////////
rqt

sudo apt update

sudo apt install ~nros-foxy-rqt*

rqt --force-discover 

*/

// Printing points in rviz
Marker create_points_marker(const std::string type_name, Trajectory points,float z,float r, float g, float b, uint32_t type = Marker::POINTS, float x = 0.3){
    Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = points.header.stamp;
    marker.ns = type_name;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.scale.x = x;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    ColorRGBA c;
    c.a = 1.0;
    c.b = b;
    c.r = r;
    c.g = g;

    for(size_t i = 0;i<points.points.size(); i++){
      Point p;
      p.x = points.points[i].x;
      p.y = points.points[i].y;
      p.z = z;
      marker.points.push_back(p);
      marker.colors.push_back(c);
    }

    return marker;
  }
// Printing shapes in rviz
void add_to_marker_array(MarkerArray & marker_array, Header & header, const std::string type_name, uint32_t type,
     float x_pos,float y_pos,float z_pos, float heading_angle, float r, float g, float b,
     std::string frame_id = std::string("/map"), 
     float sx = 1.0, float sy = 1.0, float sz = 1.0, size_t id = 0,float a = 1, uint32_t action = Marker::ADD)//bool frame_locked = false, 
     {
             
        Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = header.stamp;
        marker.ns = type_name;
        marker.id = (int)id;
        marker.type = type;
        marker.action = action;
        marker.pose.position.x = x_pos;
        marker.pose.position.y = y_pos;
        marker.pose.position.z = z_pos;
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0, 0, heading_angle);
        marker.pose.orientation = tf2::toMsg(quat_tf);
        marker.scale.x = sx;
        marker.scale.y = sy;
        marker.scale.z = sz;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;

        // marker.lifetime = rclcpp::Duration(0.1);
        marker.frame_locked = false;//frame_locked
        marker.text = "";
        //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        marker.mesh_use_embedded_materials = false;

        marker_array.markers.push_back(marker);
     }

void print_header(Header & header){
    printf("header:\n ");
    printf("frame_id: %s \n", header.frame_id.c_str());
    printf("stamp:\n \t seconds: %d \n \t nanoseconds: %u \n", header.stamp.sec,header.stamp.nanosec);
  }
void print_trajectory_point(TrajectoryPoint & tp){
    RCLCPP_INFO(rclcpp::get_logger("logger"),
    "x: %f [m], y: %f [m]\n"
    "acceleration: %f [mps2]\n"
    "front_wheel_angle: %f [rad]\n"
    "heading_rate: %f [rps]\n"
    "lateral_velocity: %f[mps] \n"
    "longitudinal_velocity: %f[mps] \n"
    "heading: %f[rad] \n",
    tp.x,tp.y,tp.acceleration_mps2,tp.front_wheel_angle_rad,tp.heading_rate_rps,tp.lateral_velocity_mps,
    tp.longitudinal_velocity_mps,to_angle(tp.heading));
  }
void print_state(KinematicState & state){
  printf("--------------state---------------\n");
  print_header(state.header);
  print_trajectory_point(state.state);
}
// Finding the color of the obstacle for coloring nlvo
float* get_color(BoundingBox box){
  static float color[3];
  switch (box.vehicle_label) {
    case BoundingBox::NO_LABEL:    // white: non labeled
      color[0] = 1.0F;
      color[1] = 1.0F;
      color[2] = 1.0F;
      break;
    case BoundingBox::CAR:    // yellow: car
      color[0] = 1.0F;
      color[1] = 1.0F;
      color[2] = 0.0F;
      break;
    case BoundingBox::PEDESTRIAN:    // blue: pedestrian
      color[0] = 0.0F;
      color[1] = 0.0F;
      color[2] = 1.0F;
      break;
    case BoundingBox::CYCLIST:    // orange: cyclist
      color[0] = 1.0F;
      color[1] = 0.647F;
      color[2] = 0.0F;
      break;
    case BoundingBox::MOTORCYCLE:    // green: motorcycle
      color[0] = 0.0F;
      color[1] = 1.0F;
      color[2] = 0.0F;
      break;
    default:    // black: other labels
      color[0] = 0.0F;
      color[1] = 0.0F;
      color[2] = 0.0F;
      break;
  }
  return color;
}
float distance(TrajectoryPoint & s1,TrajectoryPoint & s2)
{
  return float(sqrt((s1.x - s2.x) * (s1.x - s2.x) + 
  (s1.y - s2.y) * (s1.y - s2.y)));
}
float get_angle(TrajectoryPoint & s1,TrajectoryPoint & s2)
{
  // printf("\n\ns2y: %f,s1y: %f,s2x: %f,s1x: %f, \n\n",s2.y,s1.y,s2.x,s1.x);
  return float(atan2(s2.y - s1.y, s2.x - s1.x ));
}
float sqr_distance(TrajectoryPoint & s1,TrajectoryPoint & s2)
{
  return (s1.x - s2.x) * (s1.x - s2.x) + 
  (s1.y - s2.y) * (s1.y - s2.y);
}
size_t find_target_index(KinematicState & state, Trajectory & trajectory, float lookahead_dist)
{
  float sqr_lookahead_dist = lookahead_dist*lookahead_dist;
  if(trajectory.points.size() == 0){
      printf("\n168\n");
      return 1;
    }
  for (size_t i = 0; i < trajectory.points.size();i++ ){
    if (sqr_distance(trajectory.points.at(i),state.state) > sqr_lookahead_dist){
      return i;
    }
  }
  return trajectory.points.size() - 1;//last point
}
TrajectoryPoint convert_axis_x_y(TrajectoryPoint & point, TrajectoryPoint & state){
  TrajectoryPoint tp;
  tp.x = point.x * cos(to_angle(state.heading)) + point.y * sin(3.1416f + to_angle(state.heading));
  tp.y = point.x * sin(to_angle(state.heading)) + point.y * cos(3.1416f + to_angle(state.heading));

  point.x = tp.x;
  point.y = tp.y;

  return point;
}

//Creating a trajectory by time-based angular velocity, where time starts from a minimum time horizon in dt jumps
//When the speed is less than 0.001 it creates a straight line track. Otherwise, by angular velocity.
// input: Initial position of the obstacle: x, y, angle, velocity, angular velocity.
//                                     Time: minimum, maximum, dt.
Trajectory create_trajectory_by_heading_rate(float x_pos,float y_pos ,float heading, float velocity ,float dt, float min_time_horizon, float max_time_horizon, float heading_rate = 0){
  Trajectory trajectory;
  float radius = 0;
  float teta_0 = heading;
  if(abs(heading_rate) > 0.001){
    radius = abs(velocity / heading_rate);
    if(heading_rate > 0){
      teta_0 = heading - M_PI_2;
    }
    else if (heading_rate < 0){
      teta_0 = heading + M_PI_2;
    }
  }
  float x_0 = radius * cos(teta_0);
  float y_0 = radius * sin(teta_0);
  TrajectoryPoint tp;
  for(float time = min_time_horizon, count = 0; time < max_time_horizon && count < 100; (time = time + dt), count++){
    float shift_x = radius * cos(teta_0 + heading_rate * time) - x_0;
    float shift_y = radius * sin(teta_0 + heading_rate * time) - y_0;

    if(abs(heading_rate) < 0.001){
      shift_x = velocity * cos(teta_0) * time;
      shift_y = velocity * sin(teta_0) * time;
    }
    tp.x = x_pos + shift_x;
    tp.y = y_pos + shift_y;
    tp.rear_wheel_angle_rad = time; // Keeping time of that point in the trajectory
    trajectory.points.push_back(tp);
  }
  return trajectory;
}

//Calculating a time-based trajectory for ego, based on a distance-based trajectory obtained from another node.
Trajectory get_time_based_trajectory(Trajectory trajectory, float dt, float min_time_horizon, float max_time_horizon){
  Trajectory traj;
  float time = min_time_horizon;
  float previous_point_time = 0;
  float angel = 0;
  for(size_t point = 0; (point < trajectory.points.size()) && (time <= max_time_horizon);){
    TrajectoryPoint this_point = trajectory.points.at(point);
    float point_velocity = this_point.longitudinal_velocity_mps;
    if(point == trajectory.points.size()-1){
      
      angel = get_angle(trajectory.points.at(point-1), this_point);
      Trajectory rest_of_traj = create_trajectory_by_heading_rate(this_point.x ,this_point.y ,angel , point_velocity , dt, time - previous_point_time, max_time_horizon - time);

      traj.points.insert(traj.points.end(), rest_of_traj.points.begin(), rest_of_traj.points.end());
      break;
    }
    TrajectoryPoint next_point = trajectory.points.at(point+1);

    angel = get_angle(this_point, next_point);
    
    float dis = distance(this_point, next_point);
    float next_point_time = previous_point_time + dis / point_velocity;

    if(time > next_point_time){
      previous_point_time = next_point_time;
      point++;
      continue;
    }
    else{
      TrajectoryPoint tp;

      tp.x = this_point.x + point_velocity * cos(angel) * (time - previous_point_time);
      tp.y = this_point.y + point_velocity * sin(angel) * (time - previous_point_time);
      tp.longitudinal_velocity_mps = point_velocity;
      tp.rear_wheel_angle_rad = time;
      traj.points.push_back(tp);
      time = time + dt;
    }
  }
  return traj;
}

//Checks whether the point of intersection of the straight lines is on a particular line; return: true, else-false
bool cutting_point_on_line(float x_cutting_point, float y_cutting_point ,TrajectoryPoint & point2, TrajectoryPoint & point1){
  float x1, y1, x2, y2;
  x1 = point1.x;
  y1 = point1.y;
  x2 = point2.x;
  y2 = point2.y;
  if(x1 <= x_cutting_point && x_cutting_point <= x2){
      if((y1 <= y_cutting_point && y_cutting_point <= y2) || (y2 <= y_cutting_point && y_cutting_point <= y1)){
        return true; //The point of intersection is on the straight line
      }
    }
  else if (x2 <= x_cutting_point && x_cutting_point <= x1)
  {
    if((y1 <= y_cutting_point && y_cutting_point <= y2) || (y2 <= y_cutting_point && y_cutting_point <= y1)){
      return true; //The point of intersection is on the straight line
    }
  }
  return false;
}

// constructor - called at the first time
VelocityObstacles::VelocityObstacles()
{
  count = 0;//initialize a counter
}

// Calculating the location of the ego in the next step - (tree speeds)
KinematicState get_next_pos_ego(KinematicState & ego, TrajectoryPoint & velocity, float step_time){
  KinematicState new_pos_ego;
  new_pos_ego.state.x = ego.state.x + velocity.longitudinal_velocity_mps * step_time;
  new_pos_ego.state.y = ego.state.y + velocity.lateral_velocity_mps * step_time;
  new_pos_ego.state.longitudinal_velocity_mps = sqrt(pow(velocity.longitudinal_velocity_mps,2) + pow(velocity.lateral_velocity_mps,2));
  // RCLCPP_INFO(rclcpp::get_logger("logger"),"\n ego.state.longitudinal = %f,ego.state.lateral %f \n", ego.state.longitudinal_velocity_mps,ego.state.lateral_velocity_mps);
  new_pos_ego.state.heading = from_angle(atan2(velocity.lateral_velocity_mps,velocity.longitudinal_velocity_mps));//velocity.heading + ego.state.heading;
  // RCLCPP_INFO(rclcpp::get_logger("logger"),"\n velocity.heading= %f, ego %f chack: %f\n", to_angle(velocity.heading),to_angle(ego.state.heading),atan2(velocity.lateral_velocity_mps,velocity.longitudinal_velocity_mps));
  float time = ego.state.time_from_start.sec + /*(ego.state.time_from_start.nanosec / 1e9) +*/ step_time;
  // RCLCPP_INFO(rclcpp::get_logger("logger"),"\n sec %f nanosec %f n %f\n", ego.state.time_from_start.sec,ego.state.time_from_start.nanosec,ego.state.time_from_start.nanosec / 1e9);
  new_pos_ego.state.time_from_start.sec = int(time);
  new_pos_ego.state.time_from_start.nanosec = uint32_t((time - floor(time))* 1e9);
  return new_pos_ego;
}

class obstacle{
  public: BoundingBox box;
  public: Trajectory route;

  //Update the barrier data after the barrier is created.
  void update(BoundingBox & b, Trajectory & trajectory,float min_time_horizon, float max_time_horizon)
  {
    box = b;
    route = limit_horizon(trajectory, min_time_horizon, max_time_horizon);
  }
  //Produces a new trajectory by minimum and maximum time horizon
  Trajectory limit_horizon(Trajectory & trajectory,float min_time_horizon, float max_time_horizon)
  {
    Trajectory limited_trajectory;
    for(auto point:trajectory.points){
      rclcpp::Duration t = point.time_from_start;
      if(t.seconds() > max_time_horizon){
        break;
      }
      if(t.seconds() > min_time_horizon){
        limited_trajectory.points.push_back(point);
      }
    }
    // for (auto point:limited_trajectory.points){
    //   rclcpp::Duration t = point.time_from_start;
    //     RCLCPP_INFO(rclcpp::get_logger("logger"),"time from start %f\n",t.seconds());

    // }
    return limited_trajectory;
  }
};

class velocity_obstacle{

    public: TrajectoryPoint car_state; //Vehicle location information
    Header header;
    MarkerArray* marker_array;
    vector <circle> nlvo = vector <circle>(0); //A vector that contains within it all the nlvo circles created from all the obstacles
    vector <optional_velocitys> velocitys = vector <optional_velocitys>(0); //Vector containing the 9 possible velocitys
    vector <optional_velocitys> velocity_branches = vector <optional_velocitys>(0);//A vector containing the velocities for which an option tree will be built
    Trajectory left_border;
    Trajectory right_border;
    optional_velocitys chosen_velocity;
    parameters param;
    
    float acceleration; //Maximum possible acceleration of the vehicle
    float braking; //Maximum possible braking of the vehicle
    float dt_controller; // The time when the acceleration will act to change velocity.
    float border_collision_time; //Minimum time to exit the roadside
    float car_radius;

    // dinumic controller
    float max_angle = M_PI; // Need to use a function that calculates in relation to the given speed
    float angel_interval; // Resolution speeds in the controller
    float max_velocity; //

    bool vo_reliability = true;

  //Initialize an object with parameters
  velocity_obstacle(KinematicState & state, float radius, float accel, float brake, float accel_time, float border_coll_time ,Trajectory & left_b,
  Trajectory & right_b , float d_teta, float friction, float v_max, MarkerArray* marker){
    car_state = state.state;
    header = state.header;
    marker_array = marker;
    acceleration = accel;
    braking = brake;
    dt_controller = accel_time;
    border_collision_time = border_coll_time;
    left_border = compute_border_vo(left_b);
    right_border = compute_border_vo(right_b);
    car_radius = radius;
    max_velocity = v_max;
    angel_interval = d_teta;
    
    create_dinamic_controler();
    // create_all_velocitys_dinamic_model();
    // create_all_velocitys();
    
    velocitys_outside_border();
  }

  //Converts the speeds to the axis in the direction of the destination point of the vehicle, for the use of a cost function
  vector <optional_velocitys> shifted_velocity(float angle)
  {
    vector <optional_velocitys> new_velocitys = vector <optional_velocitys>(0);
    optional_velocitys tp;

    for (size_t i = 0; i < velocitys.size();i++){
      float velocity_angle = atan2(velocitys.at(i).velocity.lateral_velocity_mps ,velocitys.at(i).velocity.longitudinal_velocity_mps);
      tp.velocity.lateral_velocity_mps = -1 * velocitys.at(i).velocity.longitudinal_velocity_mps * sin(angle)
      + velocitys.at(i).velocity.lateral_velocity_mps * cos(angle);
      tp.velocity.longitudinal_velocity_mps = velocitys.at(i).velocity.longitudinal_velocity_mps * cos(angle)
      + velocitys.at(i).velocity.lateral_velocity_mps * sin(angle);
      
      new_velocitys.push_back(tp);
    }
    return new_velocitys;
  }

  //Calculate the optimal velocity for bringing the ego to the destination. Returns the index of velocity.
  void selected_speed(TrajectoryPoint & lookahed){
    float angle = get_angle(car_state , lookahed);
    vector <optional_velocitys> new_velocitys = shifted_velocity(angle);
    float car_speed = car_state.longitudinal_velocity_mps * cos(to_angle(car_state.heading) - angle); //shift car velocity to new axis. x_axis
    float delta = distance(car_state, lookahed); 
    float min = -1;
    float temp;
    size_t index = 0;
    bool flag = true;

    for (size_t i = 0; i < velocitys.size(); i++){
      if (velocitys.at(i).optional_nlvo && velocitys.at(i).optional_border){
        temp = cost(delta, car_speed, new_velocitys.at(i).velocity.longitudinal_velocity_mps, acceleration);
        velocitys.at(i).cost = temp;
        if (temp < min || min == -1.0f){
          min = temp;
          index = i;
          flag = false;  
        }
      }
    }
    if (flag)
    {
      float max = 0;
      for(size_t i = 0; i < velocitys.size(); i++){
            if (max < velocitys.at(i).min_time_horizon){
          max = velocitys.at(i).min_time_horizon;
          index = i;
        }
      }
    }
    chosen_velocity = velocitys.at(index);  
  }

  //Calculate the cost of a specific speed.
  float cost(float x, float xd0, float xdf, float umax)// umax = maximum accelration, x=distans to point
  {
    float r1, c;//C is the cost and it is equal to the time to get to xf from x0.
    float  xd; 
  //the method to have the cost is by assuming bang bang acceleration profile in order to get between two longitude points with accelration of umax.
    //x = x0 - xf;// distance to trajectory point
    xd = xd0 - xdf; // speed difference
    if(x >= 0)
    {
      r1 = xd +sqrt(2*umax*x);
      if(r1 > 0)
      {
        c = (float)(xd + sqrt(4*umax*x + 2*pow(xd,2)))/umax;   
      }
      else{
        c = (float)(-xd + sqrt(-4*umax*x + 2*pow(xd,2)))/umax;   
      }
      return c;
    }
    else
    {
      r1 = xd - sqrt(-2*umax*x);

      if(r1>=0)
      {
        c = (float)(xd + sqrt(4*umax*x + 2*pow(xd,2)))/umax;   
      }
      else
      {
        c = (float)(-xd + sqrt(-4*umax*x + 2*pow(xd,2)))/umax;   
      }
      return c;
    }
  } 

  void create_singel_velocity_dinamic_model(float accel, float angle){// angle- In relation to the direction of travel
    optional_velocitys temp;
    temp.optional_nlvo = true;
    temp.optional_border =true;
    //float accel = acceleration * accel_multi;
    float global_angle = to_angle(car_state.heading) + angle; //Angle with respect to the X-axis of a world system
    float speed = car_state.longitudinal_velocity_mps + (accel * dt_controller);
    if(speed <= max_velocity){
      temp.velocity.longitudinal_velocity_mps = speed * cos(global_angle);
      temp.velocity.lateral_velocity_mps = speed * sin(global_angle);
      temp.velocity.x = speed * cos(global_angle);
      temp.velocity.y = speed * sin(global_angle);
      temp.velocity.heading = from_angle(angle); // Angle relative to Igo's X axis
      
      if(accel < 0) temp.acceleration_size = -1;
      if (accel > 0) temp.acceleration_size = 1;
      if (accel == 0) temp.acceleration_size = 0;
      // RCLCPP_INFO(rclcpp::get_logger("logger"),"acceleration_size %d\n",temp.acceleration_size);

      // print_trajectory_point(temp.velocity);
      velocitys.push_back(temp); // insert velocity to vector

    
    } 
  }

  void create_all_velocitys_dinamic_model(){
    int velocity_num_in_arc = max_angle / angel_interval;
    for(int i = -velocity_num_in_arc / 2; i <= velocity_num_in_arc / 2; i++){
        float velocity_angle = angel_interval * i;
        //  RCLCPP_INFO(rclcpp::get_logger("logger"),"j %d, velocity_angle %f, angel_interval %f i %d, max: %f\n",j,velocity_angle,angel_interval,i,max_angle);
        create_singel_velocity_dinamic_model(acceleration ,velocity_angle);
        create_singel_velocity_dinamic_model(braking ,velocity_angle);
        create_singel_velocity_dinamic_model(0 ,velocity_angle);
    }
  }
  //Create a speed controller according to the size and direction of the speed vector.
  void create_velocity_dinamic_controler(float speed, float teta){
    
    float global_angle = to_angle(car_state.heading) + teta; //Angle with respect to the X-axis of a world system
    optional_velocitys temp;
    temp.optional_nlvo = true;
    temp.optional_border =true;
    temp.velocity.longitudinal_velocity_mps = speed * cos(global_angle);
    temp.velocity.lateral_velocity_mps = speed * sin(global_angle);
    temp.velocity.x = speed * cos(global_angle);
    temp.velocity.y = speed * sin(global_angle);
    temp.velocity.heading = from_angle(global_angle); // Angle relative to Igo's X axis
    // temp.min_time_horizon = 1000;
    // print_trajectory_point(temp.velocity);

    velocitys.push_back(temp); // insert velocity to vector
  }

  //Calculate the size and angle of each speed according to the maximum possible acceleration of the vehicle before losing wheel grip.
  // The overall acceleration consists of tangential acceleration and centripetal acceleration.
  void create_dinamic_controler(){
    create_velocity_dinamic_controler(car_state.longitudinal_velocity_mps, 0);
    for(int n = -param.num_of_points_controller; n <= param.num_of_points_controller; n++){
      float x;
      if(n>=0) x = sqrt(pow(param.num_of_points_controller,2) - pow(n,2));
      else x = -sqrt(pow(param.num_of_points_controller,2) - pow(n,2));
      float ratio = (float)x / param.num_of_points_controller;
      float omega = sqrt(pow(param.max_braking,2) - pow(ratio * param.max_braking,2)) / car_state.longitudinal_velocity_mps;
      float teta  = omega * param.dt_controller;
      float speed = car_state.longitudinal_velocity_mps + ratio * param.max_braking * param.dt_controller;

      // float accel = sqrt(pow(param.max_braking,2)-pow(ratio * param.max_braking,2));
      // float speed = car_state.longitudinal_velocity_mps + accel * param.dt_controller;
      // float omega = param.max_braking / speed;
      // float teta  = omega * param.dt_controller;

      // RCLCPP_INFO(rclcpp::get_logger("logger"),"omega %f, teta %f, speed %f ratio %f, n %d, x %f\n",omega,teta,speed,ratio,n,x);
      create_velocity_dinamic_controler(speed, teta);
      create_velocity_dinamic_controler(speed, -teta);
    }
  }

  /*
  The action creates a possible velocity vector relative to the existing velocity of the vehicle
  The difference between the 9 vectors is only at an angle with respect to the current velocity vector of the vehicle,
  so the operation takes only the coefficient of the angle on the x, y axis
  */
  void create_singel_velocity(float x_multiplication, float y_multiplication){
    optional_velocitys temp;
    //temp = car_state;
    temp.optional_nlvo = true;
    temp.optional_border =true;
    // temp.y = y velocity on global axis for rviz and nlvo
    // temp.x = x velocity on global axis for rviz and nlvo

    float acclel_x = acceleration * dt_controller * x_multiplication;
    float acclel_y = acceleration * dt_controller * y_multiplication;

    // t = x * cos() - y* sin()
    // n = x * sin() + y * cos()
    temp.velocity.longitudinal_velocity_mps = (car_state.longitudinal_velocity_mps + acclel_x) * cos(to_angle(car_state.heading)) - acclel_y * sin(to_angle(car_state.heading));// x-axis
    temp.velocity.lateral_velocity_mps = (car_state.longitudinal_velocity_mps + acclel_x) * sin(to_angle(car_state.heading)) + acclel_y * cos(to_angle(car_state.heading)); // y-axis
    
    temp.velocity.x = /*car_state.x + */temp.velocity.longitudinal_velocity_mps; //vo Coordinate system
    temp.velocity.y = /*car_state.y + */temp.velocity.lateral_velocity_mps;  //vo Coordinate system
    temp.velocity.acceleration_mps2 = 0;

    velocitys.push_back(temp); // insert velocity to vector

  }

  /*
  The action produces the 9 vectors, the vectors are on a circle at the center of which is the current velocity of the vehicle
  0- No acceleration in the axis.
  1- Maximum acceleration in the axis, 
  cosine - the maximum acceleration is divided between the axes.
  */
  void create_all_velocitys(){
    float cosine =(float)cos(45*3.14/180);
    create_singel_velocity( 0, -1);//0 dont move -> "velocitys.at(0).optional = True;"
    create_singel_velocity( 0, 0);
    create_singel_velocity( 0, 1);
    create_singel_velocity( 1, 0);
    
    create_singel_velocity( -1, 0);
    create_singel_velocity( cosine, cosine);
    create_singel_velocity( cosine, -1*cosine);
    create_singel_velocity( -1*cosine, -1*cosine);
    create_singel_velocity( -1*cosine, cosine);

    float small_cos = (float)cos(3.14/8);
    float small_sin = (float)sin(3.14/8);
    create_singel_velocity(small_cos, small_sin);
    create_singel_velocity(small_cos, -small_sin);
    create_singel_velocity(small_sin, -small_cos);
    create_singel_velocity(-small_sin, -small_cos);
    create_singel_velocity(-small_cos, -small_sin);
    create_singel_velocity(-small_cos, small_sin);
    create_singel_velocity(-small_sin, small_cos);
    create_singel_velocity(small_sin, small_cos);
  }

  //Checking whether a specific velocity vector is allowed or is in the forbidden velocity range.
  bool collision_check(TrajectoryPoint & possible_velocity, circle & specific_circle)
  {
    bool collision = false;
    // Calculating the location of a velocity vector in a world system
    float velosity_x = possible_velocity.x;
    float velosity_y = possible_velocity.y;
    //Check if the velocity vector is inside the nlvo circle.
    collision = (pow(velosity_x-specific_circle.x_position,2)+pow(velosity_y-specific_circle.y_position,2) > pow(specific_circle.radius,2));
    return collision;// true= safe velocity
  }

  //The operation checks all speeds for a single circle and flag forbidden speeds.
  void possible_velocity_for_singel_circle(circle & specific_circle){
    bool status;
    for (size_t i = 0; i < velocitys.size(); i++){
      if(velocitys.at(i).optional_border){ // only velocitis that not outside the border.
        status = collision_check(velocitys.at(i).velocity, specific_circle);
        if (velocitys.at(i).optional_nlvo && !status){
          velocitys.at(i).optional_nlvo = status;//true= safe velocity
        }

        if (/*!velocitys.at(i).optional_nlvo*/!status && velocitys.at(i).min_time_horizon > specific_circle.time){
              // RCLCPP_INFO(rclcpp::get_logger("logger"),"index: %zu, possible_velocity_for_singel_circle min_time_horizon %f\n specific_circle.time: %f",i,velocitys.at(i).min_time_horizon, specific_circle.time);

          velocitys.at(i).min_time_horizon = specific_circle.time;

        }
      }
    }
  }

  //The method calculates the possible velocities for the whole space (nlvo/smvo)
  public: void compute_allowed_velocity(){
    for (auto current_circle = nlvo.begin(); current_circle != nlvo.end(); current_circle++){
      possible_velocity_for_singel_circle(*current_circle);
    }
  }

  //Calculate nlvo for a particular obstacle
  void compute_nlvo_for_obstacle(obstacle & obstacle_car, size_t index){
          // RCLCPP_INFO(rclcpp::get_logger("logger"),"\n obstacle_car.route.points.size() %zu\n\n",obstacle_car.route.points.size());
    for (size_t i = 0; i < obstacle_car.route.points.size(); i++){
        // RCLCPP_INFO(rclcpp::get_logger("logger"),"\n obstacle_car.route.points.size() %zu\n\n",obstacle_car.route.points.size());
      compute_nlvo_for_point_in_trajectory(obstacle_car.route.points.at(i), index, obstacle_car.box);
    }
  }
  /*
  The action calculates nlvo for one specific point in the path of the obstacle. 
  Inputs the nlvo (circle) directly into the nlvo vector of the object from which the action is called.
  */
  void compute_nlvo_for_point_in_trajectory(TrajectoryPoint & point, size_t index, BoundingBox box){
    circle circle;
    // float time = point.rear_wheel_angle_rad;
    rclcpp::Duration t = point.time_from_start;
    float circle_time = t.seconds();
    rclcpp::Duration t_car = car_state.time_from_start;
    float ego_time = t_car.seconds();
    float time = circle_time - ego_time;
    //Distance between point and vehicle, division by the number of seconds it will take for the obstacle to get there.
    circle.x_position =(float) ((point.x - car_state.x) / time); 
    circle.y_position =(float) ((point.y - car_state.y) / time);
    circle.radius = (float)(car_radius / time);
    circle.obstacl_num = index;
    circle.time = time;
    circle.box = box;
    nlvo.push_back(circle);
  }

  //Checking speeds off-road.
  void velocitys_outside_border(){
    for (size_t i=0; i < left_border.points.size()-1; i++){
      border_collision_check(left_border.points.at(i), left_border.points.at(i+1));
      border_collision_check(right_border.points.at(i), right_border.points.at(i+1));
    }
  }

  /*
  y=ax+b, y=cx+d
  ax+b = cx+d
  x = (d-b)/(a-b)
  */
  void border_collision_check(TrajectoryPoint & border_point_1, TrajectoryPoint & border_point_2)
  {
    float x1, y1, x2, y2;
    x1 = border_point_1.x;
    y1 = border_point_1.y;
    x2 = border_point_2.x;
    y2 = border_point_2.y;

    float border_slop = (y2 - y1) / (x2 - x1); //a

    TrajectoryPoint zero;
    zero.x = zero.y = 0;
    if (velocitys.size() == 0){
      printf("\n\n614\n\n");
      return;
    }

    for (size_t i = 0; i < velocitys.size(); i++){
      float velocity_y = velocitys.at(i).velocity.y;
      float velocity_x = velocitys.at(i).velocity.x;
      float velocity_slop = velocity_y / velocity_x;
      float x_cutting_point = (y2 - border_slop * x2)/(velocity_slop - border_slop);
      float y_cutting_point = velocity_slop * x_cutting_point;

      bool cut_of_velocity = cutting_point_on_line(x_cutting_point, y_cutting_point, velocitys.at(i).velocity, zero);
      bool cut_of_border = cutting_point_on_line(x_cutting_point, y_cutting_point, border_point_2, border_point_1);
      // When the two intersection point is on the speed vector and the boundary line, set speed is not possible
      if(cut_of_velocity && cut_of_border){
        velocitys.at(i).optional_border = false; //censel velocity
        velocitys.at(i).min_time_horizon = 0;
      }
    }
  }

  //Calculation of road boundaries in the speed space
  Trajectory compute_border_vo(Trajectory & border){
    Trajectory time_border;

    for (size_t i = 0; i < border.points.size(); i++){
      
      TrajectoryPoint point;

      point.x = (border.points.at(i).x - car_state.x) / border_collision_time; //vo Coordinate system
      point.y = (border.points.at(i).y - car_state.y) / border_collision_time; //vo Coordinate system

      time_border.points.push_back(point);
      
    }
    return time_border;
  }

  // void get_velocity_branch_by_angle(){
  //   for(auto velocity:)
  // }

  //Finding the number of speeds at the lowest cost in favor of calculating tree speeds using them.
  void get_velocity_branch_by_cost(){
    float max_cost = velocitys.at(0).cost;
    for(auto point:velocitys){
      if(point.cost > max_cost) max_cost = point.cost;
    }

    vector <optional_velocitys> velocity_branch = vector <optional_velocitys>(0);
    velocity_branch.push_back(chosen_velocity);
    optional_velocitys temp;
    float min_cost = max_cost;
    float last_min_cost = chosen_velocity.cost;
    for(int i = 0; i < param.num_velocity_in_level;i++){
      for(auto point:velocitys){
        if(!point.optional_nlvo || !point.optional_border) continue;
        if(point.cost < min_cost && point.cost > last_min_cost){
          min_cost = point.cost;
          temp = point;
        } 
      }
      // print_trajectory_point(temp.velocity);
      velocity_branch.push_back(temp);
      last_min_cost = temp.cost;
      min_cost = max_cost;
    }
    // RCLCPP_INFO(rclcpp::get_logger("logger"),"\n velocity_branch size %zu\n",velocity_branch.size());
    velocity_branches = velocity_branch;
  }

  void print_vo_to_rviz(string vo_name, MarkerArray* marker_array){
    
    // Delete all old graphics 
    for(size_t i = 0; i < 1200 + (nlvo.size()); i++){
        add_to_marker_array(*marker_array,header,vo_name ,Marker::CYLINDER,0,0,0,0,0,0,0, "/map",1,1,1, i ,1,Marker::DELETE);
    }
    size_t index = 1;

    
    Trajectory traj;
    TrajectoryPoint zero, tp;
    traj.points.push_back(zero);
    float alfa = to_angle(car_state.heading);
    tp.x = car_state.longitudinal_velocity_mps *cos(alfa);
    tp.y = car_state.longitudinal_velocity_mps *sin(alfa);
    traj.points.push_back(tp);
    marker_array-> markers.push_back(create_points_marker(vo_name + "car_velocity", traj,1,0,0,0,Marker::LINE_STRIP, 0.2)); 
    index++;
  //print border in velocity plane 
      marker_array-> markers.push_back(create_points_marker("left border" + vo_name, left_border,1,0,0,0,Marker::LINE_STRIP, 0.4));
      index++;

      marker_array-> markers.push_back(create_points_marker("right border" + vo_name, right_border,1,0,0,0,Marker::LINE_STRIP, 0.4));
      index++;
    // }
    // print velocitys to rviz
    for (size_t i = 0; i < velocitys.size(); i++){
      int r, g, b;
      if(velocitys.at(i).optional_nlvo && velocitys.at(i).optional_border){
        // blue
        r = 0;
        g = 0.0f;
        b = 1.0f;
      }
      else{
        // yellow
        r = 1;
        g = 1;
        b = 0;
      }
      add_to_marker_array(*marker_array, header, /*"velocitys"*/vo_name, Marker:: SPHERE,
                          velocitys.at(i).velocity.x ,velocitys.at(i).velocity.y, 5, 0, r, g ,b,
                          "/map",0.3f,0.3f,0.3f/*0.6f,0.6f,0.6f*/, index);

      index++;

    }

    for(size_t i = 0; i < velocity_branches.size(); i++){
      TrajectoryPoint tp;
      tp.x = velocity_branches.at(i).velocity.x;
      tp.y = velocity_branches.at(i).velocity.y;
      add_to_marker_array(*marker_array, header, vo_name /*"branch_velocity"*/, Marker:: SPHERE,
      tp.x ,tp.y ,5, 0, 1, 1, 0, "/map",0.2f,0.4f,0.3f/*1.1f,1.1f,1.1f*/, i);// yellow

      index++;
    }

    add_to_marker_array(*marker_array, header, /*"chosen_one"*/vo_name, Marker:: SPHERE,
                        chosen_velocity.velocity.x ,chosen_velocity.velocity.y ,10, 0, 0, 1, 0,
                        "/map",0.3f,0.3f,0.3f/*1.1f,1.1f,1.1f*/, index);
    index++;

    // print nlvo
    if(nlvo.size() != 0){
      for (size_t j=0; j < nlvo.size();j++){
        float z = 0.5f;
        float* color = get_color(nlvo.at(j).box);
        if(!chosen_velocity.optional_nlvo && nlvo.at(j).time >= chosen_velocity.min_time_horizon){
          color[0] = 0.67;
          color[1] = 0.5;
          color[2] = 0.2;
          z = 0.25;
        }

        add_to_marker_array(*marker_array, header,/*"nlvo"*/vo_name ,Marker::CYLINDER,nlvo.at(j).x_position,
                            nlvo.at(j).y_position,0,-1.5708f, color[0], color[1], color[2], "/map",
                            2* nlvo.at(j).radius, 2* nlvo.at(j).radius, z, index,0.5);
        index++;
      }
    }
    // RCLCPP_INFO(rclcpp::get_logger("logger"),"\n index_rviz_marker = %zu\n", index);

  }
};

//Filter obstacles that are beyond the route obtained for the vehicle
void filtering_distant_obstacles(KinematicState & state, Trajectory & trajectory, BoundingBoxArray* box_arr, TrajectoryArray* predicted_traj){
  BoundingBoxArray boxes = *box_arr;
  TrajectoryArray predicted_trajectories = *predicted_traj;
  BoundingBoxArray relevent_boxes;
  TrajectoryArray relevent_trajectory;
    for (size_t i =0; i < boxes.boxes.size();i++){
      float dis_from_ego = sqrt(pow(boxes.boxes.at(i).centroid.x - state.state.x,2)
      +pow(boxes.boxes.at(i).centroid.y - state.state.y,2));
  // RCLCPP_INFO(rclcpp::get_logger("logger"),"\n dis_from_ego = %f",dis_from_ego);
      float dis_ego_from_end_trajectory = sqrt(pow(state.state.x - trajectory.points.back().x, 2) + pow(state.state.y - trajectory.points.back().y, 2));

      if (dis_from_ego < 0.95 * dis_ego_from_end_trajectory){// depend on nom of point in trajectory
          relevent_boxes.boxes.push_back(boxes.boxes.at(i));
          relevent_trajectory.trajectories.push_back(predicted_trajectories.trajectories.at(i));
          // printf("\n\nbox.x: %f,  box.y: %f\n\n",boxes.boxes.at(i).centroid.x, boxes.boxes.at(i).centroid.y);
        }
      }
      *box_arr = relevent_boxes;
      *predicted_traj = relevent_trajectory;
}
// comput vo for spasific moment
velocity_obstacle get_vo(KinematicState & state, Trajectory  & trajectory, BoundingBoxArray & boxes, TrajectoryArray & predicted_trajectories,
  Trajectory & border_left, Trajectory & border_right, MarkerArray & marker_array, parameters & param){
  
  add_to_marker_array(marker_array, state.header,"ego_circle" ,Marker::CYLINDER,state.state.x,
                          state.state.y,3,-1.5708f, 1, 0, 0, "/map",
                          0.5, 0.5, 0.5, 0,0.8);

  velocity_obstacle vo{state, param.car_radius, param.acceleration,param.braking, param.dt_controller,param.border_collision_time, border_left, border_right,
                        param.d_teta, param.friction_coefficient, param.max_velocity, &marker_array};

  filtering_distant_obstacles(state, trajectory, &boxes, &predicted_trajectories);
  // RCLCPP_INFO(rclcpp::get_logger("logger"),"\n size boxes = %zu, size_traj %zu",boxes.boxes.size(), predicted_trajectories.trajectories.size());

  // enemy car- global coordinates
  vector <BoundingBox> obstacles = vector<BoundingBox>(2);
  vector <obstacle> cars = vector<obstacle>(0);

  //Building a vector-type structure Obstacle 
  for (size_t i = 0; i < boxes.boxes.size(); i++){
    obstacle car;
    //assuming predicted trajectories index match boxes index:
    car.update(boxes.boxes.at(i), predicted_trajectories.trajectories.at(i),param.min_time_horizon, param.max_time_horizon);

    //Add a point list to marker array. Inputs: points, z,r,g,b:
    marker_array.markers.push_back( create_points_marker("route_obstacle" + to_string(i),car.route, 3,1,0.2,0.4));

    add_to_marker_array(marker_array, state.header,"box_circle" ,Marker::CYLINDER,boxes.boxes.at(i).centroid.x,
                          boxes.boxes.at(i).centroid.y,1,-1.5708f, 0, 0, 0.2, "/map",
                          param.car_radius *2, param.car_radius *2, 0.1, i,0.8);
    cars.push_back(car);
  }

  //Calculate nlvo for each of the obstacles
  if (cars.size()){
    for (size_t i = 0; i < cars.size(); i++){
      vo.compute_nlvo_for_obstacle(cars.at(i), i);
    }
  }
  //Finding speeds that will not cause a collision with the obstacles during the given time horizon
  vo.compute_allowed_velocity();

  // the point itself at the target point index:
  TrajectoryPoint target_point = trajectory.points.at(param.destination_index);

  //Finding the optimal speed to reach the destination
  vo.selected_speed(target_point);

  //Finding speeds for which we will create a tree of options
  // vo.get_velocity_branch();
  vo.get_velocity_branch_by_cost();

  return vo;
}

bool chack_branch_cost(TrajectoryPoint & current_pos, TrajectoryPoint & destination, vector <float>* min_branch_cost_vector, int level){
  parameters param;
  float cost = distance(current_pos, destination);
  // print_trajectory_point(current_pos);
  // print_trajectory_point(destination);
  // RCLCPP_INFO(rclcpp::get_logger("logger"),"branch cost: %f\n"
  // "min_branch_cost_vector.size(): %zu"
  // ,cost,min_branch_cost_vector-> size());

  if(min_branch_cost_vector-> size()+1 <= (size_t)level){
    min_branch_cost_vector-> push_back(cost);
    // RCLCPP_INFO(rclcpp::get_logger("logger"),"push_back(cost)");

    return true;
  }
    // RCLCPP_INFO(rclcpp::get_logger("logger"),"min_branch_cost_vector-> at((size_t)(level-1)): %f",min_branch_cost_vector-> at((size_t)(level-1)));

  // RCLCPP_INFO(rclcpp::get_logger("logger"),"min_branch_cost_vector-> at(level) size:%zu, level: %d",min_branch_cost_vector-> size(),level);

  if(cost < min_branch_cost_vector-> at((size_t)(level-1))){
    min_branch_cost_vector-> at((size_t)(level-1)) = cost;
    // RCLCPP_INFO(rclcpp::get_logger("logger"),"min_branch_cost_vector.at(level): %f at(%d)",min_branch_cost_vector-> at(level-1),level-1);

    
    return true;
  }
  float percent = (cost - min_branch_cost_vector-> at(level-1))/ min_branch_cost_vector-> at(level-1);
    // RCLCPP_INFO(rclcpp::get_logger("logger")," percent: %f\n",percent);

  if(percent > param.percent_diff_branch_cost){
    // RCLCPP_INFO(rclcpp::get_logger("logger"),"\n-----------------------delete-------------------\n");

    return false;
  }
  return true;


}
void create_branch(KinematicState & ego_state, optional_velocitys & desired_velocity, Trajectory  & trajectory,
                   BoundingBoxArray & boxes, TrajectoryArray & predicted_trajectories,Trajectory & border_left,
                    Trajectory & border_right, MarkerArray* marker_array, parameters & param, int level,
                     int velocity_num, vector <Trajectory>* velocitis_tree, vector <Trajectory>* roads_tree, Trajectory & this_branch, Trajectory & this_branch_road,
                      vector <float>* min_branch_cost_vector,string str = ""){
  // TrajectoryArray tree = *velocitis_tree;
  // TrajectoryArray roads = *roads_tree;
  // vector <float> branch_cost = *branch_c;
  level++;
  str = str +" - "+ to_string(velocity_num);
  // RCLCPP_INFO(rclcpp::get_logger("logger"),str+"  level:%d",level);
  if(level > param.max_tree_level){
    
    velocitis_tree-> push_back(this_branch);
    roads_tree-> push_back(this_branch_road);
    // tree.trajectories.push_back(this_branch);
    // roads.trajectories.push_back(this_branch_road);
    // if(velocitis_tree->size()%1000 == 0){
    //   RCLCPP_INFO(rclcpp::get_logger("logger"),str+"  level:%d",level);
    //   RCLCPP_INFO(rclcpp::get_logger("logger"),"tree.trajectories.size: %zu",velocitis_tree->size());
    // }
    // *velocitis_tree = tree;
    // *roads_tree = roads;

  }
  else{
    velocity_obstacle vo = get_vo(ego_state, trajectory, boxes, predicted_trajectories, border_left, border_right, *marker_array, param);
    // vo.print_vo_to_rviz(str,marker_array);

    param.min_time_horizon += param.step_time;
    param.max_time_horizon += param.step_time;
    
    velocity_num = 0;
    for(auto desired_velocity:vo.velocity_branches){
      KinematicState new_ego_state = get_next_pos_ego(ego_state, desired_velocity.velocity, param.step_time);
      // RCLCPP_INFO(rclcpp::get_logger("logger")," min_branch_cost_vector size: %zu level: %d\n",min_branch_cost_vector-> size(),level);
      
      if(!chack_branch_cost(new_ego_state.state, trajectory.points.at(param.destination_index),
                           min_branch_cost_vector, level)) continue;
      // RCLCPP_INFO(rclcpp::get_logger("logger")," min_branch_cost_vector sssssssize: %zu level: %d\n",min_branch_cost_vector-> size(),level);
      
      // print_trajectory_point(desired_velocity.velocity);
      // RCLCPP_INFO(rclcpp::get_logger("logger"),str+" new car_state");
      // print_trajectory_point(ego_state.state);
      this_branch.points.push_back(desired_velocity.velocity);
      this_branch_road.points.push_back(new_ego_state.state);
      // RCLCPP_INFO(rclcpp::get_logger("logger"),"num of v: %zu",vo.velocity_branches.size());
      
      // add_to_marker_array(*marker_array, ego_state.header, "desired_velocity "+str + " "+to_string(velocity_num), Marker:: SPHERE,
      //                   desired_velocity.velocity.x ,desired_velocity.velocity.y, 4, 0, 0.5, 0.5 ,0.5,
      //                   "/map",1.3f,2.0f,1.1f, int(desired_velocity.velocity.x));
      create_branch(new_ego_state, desired_velocity, trajectory, boxes, predicted_trajectories, border_left, border_right,
                    marker_array, param, level, velocity_num, velocitis_tree, roads_tree, this_branch, this_branch_road, min_branch_cost_vector,str);
      velocity_num++;
      // RCLCPP_INFO(rclcpp::get_logger("logger")," go aigen\n");

      this_branch.points.pop_back();
      this_branch_road.points.pop_back();
    }
    
  }
}

//Translates the speed at which the controller chose to travel to a point for pure_pursuit, the point is at a lookahead obtained from pure_pursuit.
TrajectoryPoint pure_pursuit_target(TrajectoryPoint & car_state ,TrajectoryPoint & velocity, float lookahead_dist){
  TrajectoryPoint target;
  float alpha = atan2(velocity.lateral_velocity_mps, velocity.longitudinal_velocity_mps);
  target.x = car_state.x + lookahead_dist * cos(alpha);
  target.y = car_state.y + lookahead_dist * sin(alpha);

  target.longitudinal_velocity_mps = sqrt(pow(velocity.longitudinal_velocity_mps,2)+pow(velocity.lateral_velocity_mps,2));
  return target;

}

//----------------------------------------------------------//
//compile: colcon build --packages-select general_controller

// veolocity obstacles controller function:
// inputs: state trajectory Bounding Boxes
// output steering command acceleration command
bool VelocityObstacles::compute_commands(
  KinematicState & state, Trajectory  & trajectory, BoundingBoxArray & boxes, TrajectoryArray & predicted_trajectories,
  Trajectory & border_left, Trajectory & border_right, float & steering_command, float & velocity_command,MarkerArray & marker_array)
{
  parameters param;
  // bool velocity_obstacle_control = false;
  bool velocity_obstacle_control = true;

  if (trajectory.points.size() < 1 || border_right.points.size() < 1 || border_left.points.size() < 1){
    return true;
  }
  if(boxes.boxes.size() != predicted_trajectories.trajectories.size()){
    return true;
  }

//Printing of a system of axes in the velocity space (vo)
add_to_marker_array(marker_array,state.header, "vo_axis" ,Marker::ARROW,-90,0,0, 0,0,1,0,"/map",180,0.1,0.01,0);
add_to_marker_array(marker_array,state.header, "vo_axis" ,Marker::ARROW,0,-90,0, 1.57,0.9,0,0,"/map",180,0.1,0.01,1);

velocity_obstacle vo = get_vo(state, trajectory, boxes, predicted_trajectories,
                              border_left, border_right, marker_array, param);
vo.print_vo_to_rviz("vo",&marker_array);


/////////////////////////////////////////////////////////////////////////////////////
// Creating a tree for proper travel routes
optional_velocitys null;
vector <Trajectory> velocitis_tree = vector<Trajectory>(0);
vector <Trajectory> roads_tree = vector<Trajectory>(0);
Trajectory this_branch, this_branch_road;
vector <float> min_branch_cost_vector = vector<float>(0);
int level = 0;
int velocity_num = 0;

// create_branch(state, null, trajectory, boxes, predicted_trajectories, border_left, border_right, &marker_array,
//               param, level, velocity_num, &velocitis_tree, &roads_tree, this_branch, this_branch_road, &min_branch_cost_vector);

// RCLCPP_INFO(rclcpp::get_logger("logger")," roads_tree.trajectories. size %zu\n",roads_tree.size());

// RCLCPP_INFO(rclcpp::get_logger("logger")," roads_tree.trajectories.at(0) size %zu\n",roads_tree.at(0).points.size());
// for(auto point:roads_tree.at(0).points){
//   RCLCPP_INFO(rclcpp::get_logger("logger")," x: %f, y %f\n",point.x, point.y);
// }
// for(size_t i = 0; i < 100 && i < roads_tree.size(); i++){
  // marker_array.markers.push_back(create_points_marker("roads_tree "+to_string((int)i),roads_tree.at((int)i*roads_tree.size()/20),4,0,0,1,Marker::LINE_STRIP, 0.1));

// }
///////////////////////////////////////////////////////////////////////////////////////////////
   
  // RCLCPP_INFO(rclcpp::get_logger("logger"),
  // "\nvelocitis_tree size: %zu"
  // "\nthis_branch size: %zu"
  // "\nbranch_cost.size %zu"
  // ,velocitis_tree.trajectories.size(),this_branch.points.size(),branch_cost.size());

  // for(auto traj:velocitis_tree.trajectories){
  //   // RCLCPP_INFO(rclcpp::get_logger("logger"),"trag: %zu, size: %zu",traj,traj.points.size());
  // }




// // Trajectory time_base_ego_trajectory = get_time_based_trajectory(state.state, trajectory, dt, min_time_horizon, max_time_horizon);
// // marker_array.markers.push_back( create_points_marker("time based ogo traj", time_base_ego_trajectory, 3,1,0.2,0.4));

// //print route_obstacles

    //Add a point list to marker array. Inputs: points, z,r,g,b:
    // marker_array.markers.push_back( create_points_marker("route_obstacle" + to_string(i),car.route, 3,1,0.2,0.4));

//     cars.push_back(car);
//   }
// }


// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////

// example - a simple implementation of a pure pursuit controller:

  // float L = 5.0F;//3.0F; //vehicle length
  // float min_lookahead_dist = 5.0F;// minimal lookahead distance (l_d). (5.0F = 5 meter as float type)
  // float max_lookahead_dist = 50.0F;// maximal lookahead distance
  // float max_steer_angle = 1.0F;//0.33F;//limit of steering angle [rad]
  // float k = 0.5F; // l_d = velocity*k

  float lookahead_dist = state.state.longitudinal_velocity_mps * param.k;

  //limit lookahead distance to maximal and minimal value:
  if (lookahead_dist > param.max_lookahead_dist){
    lookahead_dist = param.max_lookahead_dist;
  }
  else if (lookahead_dist < param.min_lookahead_dist){
    lookahead_dist = param.min_lookahead_dist;
  }

  //search the closest point along the trajectory to lookahead distance:
  size_t target_index = find_target_index(state,trajectory,lookahead_dist);
  // the point itself at the target point index:
  TrajectoryPoint target_point = trajectory.points.at(param.destination_index);

  //Calculation of a destination point for pure_pursuit according to the velocity selected in vo
  target_point = pure_pursuit_target(state.state, vo.chosen_velocity.velocity, lookahead_dist);
  


//      //                                       end velocity
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(!velocity_obstacle_control){
    //search the closest point along the trajectory to lookahead distance:
  target_index = find_target_index(state,trajectory,lookahead_dist);

  // the point itself at the target point index:
  target_point = trajectory.points.at(target_index);
  }
  
  // angle between vehicle and taget point:
  float alpha = get_angle(state.state, target_point ) - to_angle(state.state.heading);

  //compute the exact lookahead distance to the target point:
  float exact_lookahead_dist = distance(state.state, target_point);

  //compute steering angle:
  steering_command =  float(atan(2.0F*param.L*float(sin(alpha))/exact_lookahead_dist));

  //limit steering angle to a maximal and minimal value:
  if (steering_command > param.max_steer_angle){
    steering_command = param.max_steer_angle;
  }
  else if (steering_command < -param.max_steer_angle){
    steering_command = -param.max_steer_angle;
  }
  //follow velocity profile of the velocity obstacle:
  size_t forward = 4;
  if (forward < trajectory.points.size()-1){
      if(velocity_obstacle_control){
        velocity_command = target_point.longitudinal_velocity_mps; // desired speed from velocity obstacle
      }
      else{
        velocity_command = trajectory.points.at(forward+ 1).longitudinal_velocity_mps;
      }
  }
  else{
    velocity_command = 0.0F;
  }

  return false;
}
