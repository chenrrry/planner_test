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
#include <std_msgs/String.h>
#include <iostream>
#include <cstring>      // for memset
#include <sys/socket.h> // for socket functions
#include <netinet/in.h> // for sockaddr_in
#include <arpa/inet.h>  // for inet_pton
#include <unistd.h>     // for close
#include <boost/filesystem.hpp>
#include <algorithm>
using namespace boost::filesystem;
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
    ros::Subscriber subAlgorithm;
    std::string test_file;

    std::vector<geometry_msgs::Point> points_;
    std::vector<geometry_msgs::PoseStamped> start_;
    std::vector<geometry_msgs::PoseStamped> goal_;
    size_t current_point_index_;
    ros::Time last_point_time_;
    std::ofstream outfile;
    std::ofstream outfile_viz;
    ros::Timer timerForWrite;
    ros::Timer inactivity_timer_;

    bool has_received_msg_since_last_check_;
    ros::Publisher termination_pub_;

    void numberCallback(const std_msgs::Int32::ConstPtr& msg);
    void publishGoalPoint(const geometry_msgs::Point& start_point, const geometry_msgs::Point& goal_point);
    std::vector<geometry_msgs::Point> readPointsFromFile(const std::string& file_path);

    void pathCallback(const nav_msgs::Path::ConstPtr& msg); 
    void pathNodesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void pathVehiclesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void path2DNodesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void pathBoxesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void algorithmCallback(const std_msgs::String::ConstPtr& msg);
    void inactivityTimeout(const ros::TimerEvent&);
    void resetTimeForInactivity();
    void writeCallback(const ros::TimerEvent&) {
        outfile.flush(); 
        outfile_viz.flush(); 
        // std::cout<<"已经写入文件"<<std::endl;
    }
    bool whetherPublishFirstGoalPoint = false;
    std::string algorithm;

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
    subAlgorithm = nh_.subscribe("/algForHA",5,&GoalPointPublisher::algorithmCallback,this);
    if(subSmoothTopic){
        subPath = nh_.subscribe("sPath", 1000, &GoalPointPublisher::pathCallback, this);
        subPathNodes = nh_.subscribe("sPathNodes", 1000, &GoalPointPublisher::pathNodesCallback, this);
        subPathVehicles = nh_.subscribe("sPathVehicle", 1000, &GoalPointPublisher::pathVehiclesCallback, this);
        subPath2DNodes = nh_.subscribe("sPath2DNodes", 1000, &GoalPointPublisher::path2DNodesCallback, this);
        subPathBoxes = nh_.subscribe("sPathBoxes", 1000, &GoalPointPublisher::pathBoxesCallback, this);
        
    }else{
        subPath = nh_.subscribe("/path", 1000, &GoalPointPublisher::pathCallback, this);
        subPathNodes = nh_.subscribe("/pathNodes", 1000, &GoalPointPublisher::pathNodesCallback, this);
        subPathVehicles = nh_.subscribe("/pathVehicle", 1000, &GoalPointPublisher::pathVehiclesCallback, this);
        subPath2DNodes = nh_.subscribe("/path2DNodes", 1000, &GoalPointPublisher::path2DNodesCallback, this);
        subPathBoxes = nh_.subscribe("/pathBoxes", 1000, &GoalPointPublisher::pathBoxesCallback, this);
    }
    timerForWrite = nh_.createTimer(ros::Duration(1), &GoalPointPublisher::writeCallback, this);
    std::string pkg_path = ros::package::getPath("test_goalpoint_publisher");
    std::cout << "current package : " << pkg_path <<std::endl;
    // 创建一个json对象
    json j;

    // 打开一个文件流来读取JSON文件
    std::ifstream file(pkg_path + "/config/base_config.json");

    // 将文件内容解析到json对象
    file >> j;  
    test_file=j["currentTest"] ;

    std::string points_file_path = pkg_path + "/data/"+test_file+".txt";
    std::vector<geometry_msgs::Point> points_from_file = readPointsFromFile(points_file_path);
    std::string output_file_path = pkg_path + "/outputUseless/"+test_file+"_result.txt";
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

int findNextIndex(const std::string& directory, const std::string& prefix) {
    path dirPath(directory);
    int maxIndex = -1; // 初始最大索引值

    if (exists(dirPath) && is_directory(dirPath)) {
        for (auto& entry : directory_iterator(dirPath)) {
            path filePath = entry.path();
            if (is_regular_file(filePath)) {
                // 提取文件名并检查是否匹配所需的前缀
                std::string fileName = filePath.filename().string();
                if (fileName.find(prefix) != std::string::npos) {
                    // 解析文件名以找到索引
                    size_t lastPos = fileName.find_last_of("_"); // 假设索引前有一个下划线
                    size_t dotPos = fileName.find_last_of("."); // 找到文件扩展名的点
                    if (lastPos != std::string::npos && dotPos != std::string::npos) {
                        std::string indexStr = fileName.substr(lastPos + 1, dotPos - lastPos - 1);
                        try {
                            int index = std::stoi(indexStr);
                            maxIndex = std::max(maxIndex, index);
                        } catch (const std::invalid_argument& e) {
                            // 忽略无法转换为整数的文件名
                        }
                    }
                }
            }
        }
    }

    return maxIndex + 1; // 返回最大索引加1
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
    int number = msg->data;
    outfile<<"data: "<<number<<std::endl;
    if(number==-1){
        std::string output_directory = ros::package::getPath("test_goalpoint_publisher") + "/output/"+this->algorithm;
        std::string file_prefix = test_file+"_resultViz_"; // 根据实际情况调整前缀
        int max_file_index = findNextIndex(output_directory, file_prefix);
        std::string output_file_path_viz = ros::package::getPath("test_goalpoint_publisher") + 
            "/output/"+this->algorithm+"/"+test_file+"_resultViz_"+std::to_string(max_file_index)+".txt";
        outfile_viz.open(output_file_path_viz);
        outfile<<output_file_path_viz<<std::endl;
        return;
    }
    if(number == -2){
        // outfile_viz.flush();
        // outfile_viz.close();//发送2不能立马关闭，因为之前的可能没有来得及处理 
        std::cout << "接收到搜索完成的消息" << std::endl;
        // Initialize the timer with a 1-second duration, triggering inactivityTimeout
        inactivity_timer_ = nh_.createTimer(ros::Duration(3), &GoalPointPublisher::inactivityTimeout, this);

        // Initialize your termination publisher (if not already done)
        termination_pub_ = nh_.advertise<std_msgs::String>("/termination_topic", 1);
        std::cout << "初始化定时器" << std::endl;
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
    sleep(3); // sleep for 3 second before publishing next goal point
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
    for (const auto& marker : msg->markers) { //注意这里的z值其实是yaw
        outfile_viz << "1" << std::endl;
        outfile_viz << marker.pose.position.x << " "<< marker.pose.position.y << " " << marker.pose.position.z << std::endl;
    }
    // Index为1，第一排为位置信息xyz
    if (inactivity_timer_ !=nullptr){
        resetTimeForInactivity();
    }
}
void GoalPointPublisher::resetTimeForInactivity(){
    inactivity_timer_.stop();
    inactivity_timer_.start();
    has_received_msg_since_last_check_ = true;
}
void GoalPointPublisher::pathVehiclesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    outfile<<"2"<<std::endl;
    for (const auto& marker : msg->markers) {
        if(marker.type==visualization_msgs::Marker::CUBE){
            outfile_viz << "2" << std::endl;
            outfile_viz << marker.pose.position.x << " "<< marker.pose.position.y << std::endl;
            outfile_viz << marker.pose.orientation.x << " "<< marker.pose.orientation.y << " " << marker.pose.orientation.z << " "
                << marker.pose.orientation.w << std::endl;
            // 记录3D路径车辆数据
            // Index为2，第一排为位置信息xyz，第二排为缩放信息xyz，第三排为方向信息xyzw，第三排为颜色信息rgb
        }else{
            outfile_viz << "3" << std::endl;
            for (size_t i = 0; i < marker.points.size(); i += 2) { //正方形不需要记录z，因为全是0
                const auto& point = marker.points[i];
                outfile_viz << point.x << " " << point.y << std::endl;
            }
        }
        }
    // 记录3D路径车辆数据
    // Index为2，第一排为位置信息xyz，第二排为缩放信息xyz，第三排为方向信息xyzw，第三排为颜色信息rgb
    if (inactivity_timer_ !=nullptr){
        resetTimeForInactivity();
    }
}

void GoalPointPublisher::path2DNodesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    outfile<<"0"<<std::endl;
    for (const auto& marker : msg->markers) {
        outfile_viz << "0" << std::endl;
        outfile_viz << marker.pose.position.x << " "<< marker.pose.position.y << std::endl;
    }
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

void GoalPointPublisher::algorithmCallback(const std_msgs::String::ConstPtr& msg) {
    std::string received_data = msg->data;
    if(received_data == "contour_hybrid_astar"){
        this->algorithm = "ENHA";
        std::cout << "测试节点接收到算法" << received_data <<std::endl;
    }else if(received_data == "hybrid_astar"){
        this->algorithm = "HA";
        std::cout << "测试节点接收到算法" << received_data <<std::endl;
    }else if(received_data == "split_hybrid_astar"){
        this->algorithm = "EHHA";
        std::cout << "测试节点接收到算法" << received_data <<std::endl;
    }else if(received_data == "rrt"){
        this->algorithm = "RRT";
        std::cout << "测试节点接收到算法" << received_data <<std::endl;
    }else{
        std::cout << "非常危险，出现了不能处理的算法"<<std::endl;
    }
}

bool sendPacket(const std::string& ip, int port, const std::string& data) {
    // 创建套接字
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        std::cerr << "Error creating socket." << std::endl;
        return false;
    }

    // 构建服务器地址结构
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported." << std::endl;
        close(sockfd);
        return false;
    }

    // 连接到服务器
    if (connect(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection Failed." << std::endl;
        close(sockfd);
        return false;
    }

    // 发送数据
    send(sockfd, data.c_str(), data.length(), 0);

    // 关闭套接字
    close(sockfd);

    return true;
}


// Add a method to handle the timeout
void GoalPointPublisher::inactivityTimeout(const ros::TimerEvent&) {
    std::cout <<"执行一次定时器"<<std::endl;
    if (!has_received_msg_since_last_check_) {
        // No messages received since the last check, send termination signal
        std_msgs::String msg;
        msg.data = "terminate";
        // Assuming you have a publisher set up to communicate with the other node
        termination_pub_.publish(msg);
        ROS_INFO("Inactivity timeout reached. Sent termination signal.");
        std::cout << "terminate" << std::endl;
        sendPacket("127.0.0.1", 12345, "terminate");
    } else {
        // Reset for the next period
        has_received_msg_since_last_check_ = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_point_publisher_node");
    
    GoalPointPublisher goal_publisher;
    ros::spin(); //there will continue the loop;
    return 0;
}