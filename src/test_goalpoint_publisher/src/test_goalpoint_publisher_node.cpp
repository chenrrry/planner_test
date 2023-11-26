#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <fstream>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h" 


class GoalPointPublisher
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_start_;
    ros::Publisher pub_goal_;
    ros::Subscriber sub_;

    std::vector<geometry_msgs::Point> points_;
    std::vector<geometry_msgs::PoseStamped> start_;
    std::vector<geometry_msgs::PoseStamped> goal_;
    size_t current_point_index_;
    ros::Time last_point_time_;
    std::ofstream outfile;

    void numberCallback(const std_msgs::Int32::ConstPtr& msg);
    void publishGoalPoint(const geometry_msgs::Point& start_point, const geometry_msgs::Point& goal_point);
    std::vector<geometry_msgs::Point> readPointsFromFile(const std::string& file_path);

public:
    GoalPointPublisher();
    ~GoalPointPublisher();
    void publishNextGoalPoint();
    void pushPoint(double x, double y, double z);
    
};


GoalPointPublisher::GoalPointPublisher()
    : current_point_index_(0)
{

    pub_start_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    sub_ = nh_.subscribe("/start_notification", 1, &GoalPointPublisher::numberCallback, this);

    std::string pkg_path = ros::package::getPath("test_goalpoint_publisher");
    std::cout << "current package : " << pkg_path;
    std::string points_file_path = pkg_path + "/data/tunnel_test.txt";
    std::vector<geometry_msgs::Point> points_from_file = readPointsFromFile(points_file_path);
    std::string output_file_path = pkg_path + "/data/tunnel_test_result.txt";; 
    outfile.open(output_file_path);
    
    for (const auto& point : points_from_file)
    {
        this->pushPoint(point.x, point.y, point.z);
    }

    last_point_time_ = ros::Time::now();
}

GoalPointPublisher::~GoalPointPublisher() 
{
    outfile.close();
}

std::vector<geometry_msgs::Point> GoalPointPublisher::readPointsFromFile(const std::string& file_path)
{
    std::vector<geometry_msgs::Point> points;
    std::ifstream infile(file_path);
    
    if (!infile.is_open())
    {
        ROS_ERROR("Failed to open waypoints file: %s", file_path.c_str());
        return points;
    }

    double x, y, t;
    while (infile >> x >> y >> t )
    {
        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = t;  // Assuming flat ground
        points.push_back(point);
    }

    infile.close();
    return points;
}

void GoalPointPublisher::pushPoint(double x, double y, double z)
{
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    points_.push_back(point);
}


void GoalPointPublisher::numberCallback(const std_msgs::Int32::ConstPtr& msg) //收到类型更改
{

    int number = msg->data;
    outfile<<"data "<<number<<std::endl;
    if (number==0)  // Threshold to consider "reached", you can adjust as needed
    {
        current_point_index_=0;
    }
    publishNextGoalPoint(); 
}

void GoalPointPublisher::publishNextGoalPoint()
{
    if (current_point_index_ +1< points_.size())
    {
        publishGoalPoint(points_[current_point_index_],points_[current_point_index_+1]);
    }
}
void GoalPointPublisher::publishGoalPoint(const geometry_msgs::Point& start_point, const geometry_msgs::Point& goal_point)
{
    geometry_msgs::PoseWithCovarianceStamped start;
    start.header.frame_id = "map";
    start.header.stamp = ros::Time::now();
    
    start.pose.pose.position.x=start_point.x;
    start.pose.pose.position.y=start_point.y;
    start.pose.pose.position.z=0;
    start.pose.pose.orientation.x=0;
    start.pose.pose.orientation.y=0;
    start.pose.pose.orientation.z=0;
    start.pose.pose.orientation.w=1;

    pub_start_.publish(start);

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x=goal_point.x;
    goal.pose.position.y=goal_point.y;
    goal.pose.position.z=0;
    goal.pose.orientation.x=0;
    goal.pose.orientation.y=0;
    goal.pose.orientation.z=0;
    goal.pose.orientation.w=1;

    usleep(10000); // small sleep before next publish
    pub_goal_.publish(goal);

    usleep(10000); // set to 1000000us (1s) on real robot
    // pub_joy_.publish(joy);
    outfile<<"publish start and goal";
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