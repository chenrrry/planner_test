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
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <nlohmann/json.hpp>
#include <fstream>
// 使用命名空间以简化代码
using json = nlohmann::json;

class GoalPointPublisher
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_start_;
    ros::Publisher pub_goal_;
    ros::Subscriber sub_;

    ros::Subscriber subPath;
    ros::Subscriber subPathNodes;
    ros::Subscriber subPathVehicles;
    ros::Subscriber subPath2DNodes;
    ros::Subscriber subPathBoxes;

    std::string test_file;

    std::vector<geometry_msgs::Point> points_;
    std::vector<geometry_msgs::PoseStamped> start_;
    std::vector<geometry_msgs::PoseStamped> goal_;
    size_t current_point_index_;
    ros::Time last_point_time_;
    std::ofstream outfile;
    std::ofstream outfile_viz;
    ros::Timer timerForWrite;


    void numberCallback(const std_msgs::Int32::ConstPtr& msg);
    void publishGoalPoint(const geometry_msgs::Point& start_point, const geometry_msgs::Point& goal_point);
    std::vector<geometry_msgs::Point> readPointsFromFile(const std::string& file_path);

    void pathCallback(const nav_msgs::Path::ConstPtr& msg); 
    void pathNodesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void pathVehiclesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void path2DNodesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void pathBoxesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void writeCallback(const ros::TimerEvent&) {
        outfile.flush(); 
        outfile_viz.flush(); 
        std::cout<<"已经写入文件"<<std::endl;
    }
    bool whetherPublishFirstGoalPoint = false;

public:
    GoalPointPublisher();
    ~GoalPointPublisher();
    void publishNextGoalPoint();
    void pushPoint(double x, double y, double z);
    bool subSmoothTopic = false;
    
};


GoalPointPublisher::GoalPointPublisher()
    : current_point_index_(0)
{
    pub_start_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    sub_ = nh_.subscribe("/start_notification", 5, &GoalPointPublisher::numberCallback, this);

    if(subSmoothTopic){
        subPath = nh_.subscribe("sPath", 1, &GoalPointPublisher::pathCallback, this);
        subPathNodes = nh_.subscribe("sPathNodes", 1, &GoalPointPublisher::pathNodesCallback, this);
        subPathVehicles = nh_.subscribe("sPathVehicle", 1, &GoalPointPublisher::pathVehiclesCallback, this);
        subPath2DNodes = nh_.subscribe("sPath2DNodes", 1, &GoalPointPublisher::path2DNodesCallback, this);
        subPathBoxes = nh_.subscribe("sPathBoxes", 1, &GoalPointPublisher::pathBoxesCallback, this);
        
    }else{
        subPath = nh_.subscribe("/path", 1, &GoalPointPublisher::pathCallback, this);
        subPathNodes = nh_.subscribe("/pathNodes", 1, &GoalPointPublisher::pathNodesCallback, this);
        subPathVehicles = nh_.subscribe("/pathVehicle", 1, &GoalPointPublisher::pathVehiclesCallback, this);
        subPath2DNodes = nh_.subscribe("/path2DNodes", 1, &GoalPointPublisher::path2DNodesCallback, this);
        subPathBoxes = nh_.subscribe("/pathBoxes", 1, &GoalPointPublisher::pathBoxesCallback, this);
    }
    timerForWrite = nh_.createTimer(ros::Duration(1), &GoalPointPublisher::writeCallback, this);
    std::string pkg_path = ros::package::getPath("test_goalpoint_publisher");
    std::cout << "current package : " << pkg_path;
    // 创建一个json对象
    json j;

    // 打开一个文件流来读取JSON文件
    std::ifstream file(pkg_path + "/config/base_config.json");

    // 将文件内容解析到json对象
    file >> j;  
    test_file=j["currentTest"] ;

    std::string points_file_path = pkg_path + "/data/"+test_file+".txt";
    std::vector<geometry_msgs::Point> points_from_file = readPointsFromFile(points_file_path);
    std::string output_file_path = pkg_path + "/output/"+test_file+"_result.txt";
    outfile.open(output_file_path);

    outfile<<"111 "<<output_file_path<<std::endl;
    
    for (const auto& point : points_from_file)
    {
        this->pushPoint(point.x, point.y, point.z);
    }

    last_point_time_ = ros::Time::now();
    std::cout << "初始化handler已经完成" << std::endl;
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
    outfile<<"x: "<<x<<" y: "<<y<<" z: "<<z<<std::endl;
}


void GoalPointPublisher::numberCallback(const std_msgs::Int32::ConstPtr& msg) //收到类型更改
{   
    if(whetherPublishFirstGoalPoint == true && msg->data == 0){
        return;
    }
    if(msg->data == 0){
        whetherPublishFirstGoalPoint = true;
    }
    std::cout<<"收到消息"<<std::endl;
    sleep(3); // sleep for 3 second before publishing next goal point
    int number = msg->data;
    outfile<<"data: "<<number<<std::endl;
    if(number==-1){
        std::string output_file_path_viz = ros::package::getPath("test_goalpoint_publisher") + 
            "/output/"+test_file+"_resultViz_"+std::to_string(current_point_index_/2)+".txt";; 
        outfile_viz.open(output_file_path_viz);
        outfile<<output_file_path_viz<<std::endl;
        return;
    }
    if(number == -2){
        outfile_viz.close();
        return;
    }
    if (number==0)  // Threshold to consider "reached", you can adjust as needed
    {
        current_point_index_=0;
    }else{
        ros::Duration travel_time = ros::Time::now() - last_point_time_;
        outfile << "Reached point " << current_point_index_  
        << " in " << travel_time.toSec() << " seconds." << std::endl;
        current_point_index_=number*2;
    }


    outfile<<"cp"<<current_point_index_<<"size "<<points_.size() <<std::endl;
    last_point_time_ = ros::Time::now();
    publishNextGoalPoint(); 
}

void GoalPointPublisher::publishNextGoalPoint()
{
    outfile<<"publishNextGoalPoint"<<std::endl;
    if (current_point_index_ +1< points_.size())
    {
        publishGoalPoint(points_[current_point_index_],points_[current_point_index_+1]);
    }
}

void GoalPointPublisher::publishGoalPoint(const geometry_msgs::Point& start_point, const geometry_msgs::Point& goal_point)
{
    std::cout<<"发送开始"<<std::endl;
    outfile<<current_point_index_<<std::endl;
    geometry_msgs::PoseWithCovarianceStamped start;
    start.header.frame_id = "map";
    start.header.stamp = ros::Time::now();
    
    start.pose.pose.position.x=start_point.x;
    start.pose.pose.position.y=start_point.y;
    start.pose.pose.position.z=0;

    tf::Quaternion tf_quaternion1;
    tf_quaternion1.setRPY(0,0,start_point.z); // 假设已经设置了roll, pitch, yaw
    tf::quaternionTFToMsg(tf_quaternion1, start.pose.pose.orientation);
    pub_start_.publish(start);

    outfile<<start.pose.pose.orientation.x<<start.pose.pose.orientation.y<<start.pose.pose.orientation.z<<start.pose.pose.orientation.w<<std::endl;

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x=goal_point.x;
    goal.pose.position.y=goal_point.y;
    goal.pose.position.z=0;
    tf::Quaternion tf_quaternion2;
    tf_quaternion2.setRPY(0,0,goal_point.z); // 假设已经设置了roll, pitch, yaw
    tf::quaternionTFToMsg(tf_quaternion2, goal.pose.orientation);

    usleep(10000); // small sleep before next publish
    pub_goal_.publish(goal);

    usleep(10000); // set to 1000000us (1s) on real robot

    outfile<<"publish start and goal"<<std::endl;
    std::cout<<"发送结束"<<std::endl;
}

void GoalPointPublisher::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    // std::string output_file_path_viz = ros::package::getPath("test_goalpoint_publisher") + 
    //     "/data/tunnel_test_result_"+std::to_string(current_point_index_/2)+".txt";; 
    // outfile_viz.open(output_file_path_viz);
    // outfile_viz.close();
    // 处理路径数据
    
}

void GoalPointPublisher::pathNodesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    outfile<<"1"<<std::endl;
    for (const auto& marker : msg->markers) {
        outfile_viz << "1" << std::endl;
        outfile_viz << marker.pose.position.x << " "<< marker.pose.position.y << " " << marker.pose.position.z << std::endl;
        outfile_viz << marker.scale.x << " " << marker.scale.y << " " << marker.scale.z << std::endl;
        outfile_viz << marker.color.r << " "<< marker.color.r << " " << marker.color.r << " " << marker.color.a << std::endl;
    }
    // 记录3D路径节点信息
    // Index为1，第一排为位置信息xyz，第二排为缩放信息xyz，第三排为颜色信息rgba
}

void GoalPointPublisher::pathVehiclesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    outfile<<"2"<<std::endl;
    for (const auto& marker : msg->markers) {
        if(marker.type==visualization_msgs::Marker::CUBE){
            outfile_viz << "2" << std::endl;
            outfile_viz << marker.pose.position.x << " "<< marker.pose.position.y << " " << marker.pose.position.z << std::endl;
            outfile_viz << marker.pose.orientation.x << " "<< marker.pose.orientation.y << " " << marker.pose.orientation.z << " "
                << marker.pose.orientation.w << std::endl;
            outfile_viz << marker.scale.x << " " << marker.scale.y << " " << marker.scale.z << std::endl;
            outfile_viz << marker.color.r << " "<< marker.color.r << " " << marker.color.r << " " << marker.color.a << std::endl;
            // 记录3D路径车辆数据
            // Index为2，第一排为位置信息xyz，第二排为缩放信息xyz，第三排为方向信息xyzw，第三排为颜色信息rgb
        }else{
            outfile_viz << "3" << std::endl;
            for (size_t i = 0; i < marker.points.size(); i += 2) {
                const auto& point = marker.points[i];
                outfile_viz << point.x << " " << point.y << " " << point.z << std::endl;
            }
        }
        }
    // 记录3D路径车辆数据
    // Index为2，第一排为位置信息xyz，第二排为缩放信息xyz，第三排为方向信息xyzw，第三排为颜色信息rgb
}

void GoalPointPublisher::path2DNodesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    // outfile<<"3 "<<std::endl;
    // for (const auto& marker : msg->markers) {
    //     outfile_viz << "3" << std::endl;
    //     outfile_viz << marker.pose.position.x << " "<< marker.pose.position.y << " " << marker.pose.position.z << std::endl;
    //     outfile_viz << marker.scale.x << " " << marker.scale.y << " " << marker.scale.z << std::endl;
    //     outfile_viz << marker.color.r << " "<< marker.color.r << " " << marker.color.r << " " << marker.color.a << std::endl;
    // }
    // 记录2D路径节点信息
    // Index为3，第一排为位置信息xyz，第二排为缩放信息xyz，第三排为颜色信息rgba
}

void GoalPointPublisher::pathBoxesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    // for (const auto& marker : msg->markers) {
    //     outfile_viz << "4" << std::endl;
    //     outfile_viz << marker.pose.position.x << " "<< marker.pose.position.y << " " << marker.pose.position.z << std::endl;
    //     outfile_viz << marker.scale.x << " " << marker.scale.y << " " << marker.scale.z << std::endl;
    //     outfile_viz << marker.color.r << " "<< marker.color.r << " " << marker.color.r << " " << marker.color.a << std::endl;
    // }
    // 记录2D搜索圆数据
    // Index为3，第一排为位置信息xyz，第二排为缩放信息xyz，第三排为颜色信息rgba

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_point_publisher_node");
    
    GoalPointPublisher goal_publisher;
    ros::spin(); //there will continue the loop;
    return 0;
}