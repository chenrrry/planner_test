#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <fstream>
#include <ros/package.h>
#include <std_msgs/Bool.h>

class GoalPointPublisher
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher pub_joy_;
    ros::Publisher update_publisher_;
    ros::Subscriber sub_;

    std::vector<geometry_msgs::Point> waypoints_;
    size_t current_waypoint_index_;
    ros::Time last_waypoint_time_;
    std::ofstream outfile;
    bool  whether_update_graph_each_reach;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void publishGoalPoint(const geometry_msgs::Point& goal_point);
    std::vector<geometry_msgs::Point> readWaypointsFromFile(const std::string& file_path);

public:
    GoalPointPublisher();
    ~GoalPointPublisher();
    void publishNextGoalPoint();
    void pushWaypoint(double x, double y, double z);
    
};


GoalPointPublisher::GoalPointPublisher()
    : current_waypoint_index_(0)
{

    pub_ = nh_.advertise<geometry_msgs::PointStamped>("/goal_point", 5);
    pub_joy_ = nh_.advertise<sensor_msgs::Joy>("/joy", 5);
    sub_ = nh_.subscribe<nav_msgs::Odometry>("/state_estimation", 5, &GoalPointPublisher::odomCallback, this);
    update_publisher_ = nh_.advertise<std_msgs::Bool>( "/update_visibility_graph", 5 );
    whether_update_graph_each_reach = true;

    std::string pkg_path = ros::package::getPath("test_goalpoint_publisher");
    std::cout << "current package : " << pkg_path;
    std::string waypoints_file_path = pkg_path + "/data/tunnel_test.txt";
    std::vector<geometry_msgs::Point> waypoints_from_file = readWaypointsFromFile(waypoints_file_path);
    std::string output_file_path = pkg_path + "/data/tunnel_test_result.txt";; 
    outfile.open(output_file_path);
    
    for (const auto& waypoint : waypoints_from_file)
    {
        this->pushWaypoint(waypoint.x, waypoint.y, 0.0);
    }

    last_waypoint_time_ = ros::Time::now();
}

GoalPointPublisher::~GoalPointPublisher() 
{
    outfile.close();
}
std::vector<geometry_msgs::Point> GoalPointPublisher::readWaypointsFromFile(const std::string& file_path)
{
    std::vector<geometry_msgs::Point> waypoints;
    std::ifstream infile(file_path);
    
    if (!infile.is_open())
    {
        ROS_ERROR("Failed to open waypoints file: %s", file_path.c_str());
        return waypoints;
    }

    double x, y;
    while (infile >> x >> y)
    {
        geometry_msgs::Point waypoint;
        waypoint.x = x;
        waypoint.y = y;
        waypoint.z = 0.0;  // Assuming flat ground
        waypoints.push_back(waypoint);
    }

    infile.close();
    return waypoints;
}



void GoalPointPublisher::pushWaypoint(double x, double y, double z)
{
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    waypoints_.push_back(point);
}




void GoalPointPublisher::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    // // 使用 std::cout 打印 odom 的信息
    // std::cout << "Received odom: "
    //           << "position (x: " << odom->pose.pose.position.x
    //           << ", y: " << odom->pose.pose.position.y
    //           << ", z: " << odom->pose.pose.position.z << "), "
    //           << "orientation (x: " << odom->pose.pose.orientation.x
    //           << ", y: " << odom->pose.pose.orientation.y
    //           << ", z: " << odom->pose.pose.orientation.z
    //           << ", w: " << odom->pose.pose.orientation.w << ")"
    //           << std::endl;
    double distance_to_waypoint = std::sqrt(
        std::pow(odom->pose.pose.position.x - waypoints_[current_waypoint_index_].x, 2) +
        std::pow(odom->pose.pose.position.y - waypoints_[current_waypoint_index_].y, 2)
    );

    if (distance_to_waypoint < 0.5)  // Threshold to consider "reached", you can adjust as needed
    {
        ros::Duration travel_time = ros::Time::now() - last_waypoint_time_;

        outfile << "Reached waypoint " << current_waypoint_index_  
                << " in " << travel_time.toSec() << " seconds." << std::endl;

        current_waypoint_index_++;
        last_waypoint_time_ = ros::Time::now();

        if (current_waypoint_index_ >= waypoints_.size()) {

            outfile << "All waypoints reached." << std::endl;  
            ros::shutdown();

        }
        if(whether_update_graph_each_reach)
        {
            std_msgs::Bool msg;
            msg.data = true;
            update_publisher_.publish(msg);
        }
        
    }else{
            publishNextGoalPoint();
    }
}



void GoalPointPublisher::publishNextGoalPoint()
{
    if (current_waypoint_index_ < waypoints_.size())
    {
        publishGoalPoint(waypoints_[current_waypoint_index_]);
    }
}
void GoalPointPublisher::publishGoalPoint(const geometry_msgs::Point& goal_point)
{


    sensor_msgs::Joy joy;

    joy.axes.push_back(0);
    joy.axes.push_back(0);
    joy.axes.push_back(-1.0);
    joy.axes.push_back(0);
    joy.axes.push_back(1.0);
    joy.axes.push_back(1.0);
    joy.axes.push_back(0);
    joy.axes.push_back(0);

    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(1);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);

    joy.header.stamp = ros::Time::now();
    joy.header.frame_id = "goalpoint_tool";

    geometry_msgs::PointStamped point;
    point.header.frame_id = "map";
    point.header.stamp = ros::Time::now();
    point.point = goal_point;

    pub_.publish(point);

    usleep(10000); // small sleep before next publish
    pub_.publish(point);

    usleep(10000); // set to 1000000us (1s) on real robot
    pub_joy_.publish(joy);
    // std::cout<<"publish goal_point;\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_point_publisher_node");
    
    GoalPointPublisher goal_publisher;
    
    // Immediately publish the first waypoint
    goal_publisher.publishNextGoalPoint();

    ros::spin(); //there will continue the loop;

    return 0;
}