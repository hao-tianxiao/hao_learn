#ifndef routing_h
#define routing_h

#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <iterator>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include </home/hao/eigen-3.4.0/Eigen/Eigen>
#include </home/hao/eigen-3.4.0/Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <ros/ros.h>


class waypoint
{
public:
    double x = 0;
    double y = 0;
    double dirAngle = 0;
    double k = 0;
    double index = 0;
};

struct vehicle_position
{
    double x;
    double y;
    double index;
};

struct vehicle_state : public vehicle_position
{
    double yaw;
    double kappa;
    double theta;
};

class emplanner
{
public:
    ros::Subscriber odom;
    ros::Subscriber qp_constrcution;
    ros::Publisher marker_pub;
    ros::Publisher marker_pub1;
    ros::Publisher marker_pub2;
    ros::Publisher marker_pub3;
    ros::Publisher global_path_publish;

    std::vector<waypoint> global_path,left_edge,right_edge,local_path;


    void nodeStart(int argc, char **argv);
    void odomCallBack(const nav_msgs::Odometry::ConstPtr odometryMsg);
    void rviz_road(ros::Publisher marker_pub, std::vector<waypoint> roads);
    double calcDistance(const double start, const double& end);
    int find_projection_point(std::vector<waypoint> global_path,nav_msgs::Odometry::ConstPtr odometryMsg);
    void definite_reference_line_range(int projection_point_index, std::vector<waypoint> global_path);
    
    void H_construction(Eigen::SparseMatrix<double> &hessian);
    void A1_construction();
    void A2_construction();
    void A3_construction();
    void h_small_construction();
    void f_construction(Eigen::VectorXd &gradient);
    void p_construction(Eigen::SparseMatrix<double> &p);
    void lb_construction(Eigen::VectorXd &lowerBound);
    void ub_construction(Eigen::VectorXd &upperBound);

    void w_init();
    
    
    int projection_point_index = 0;
    int front_point_index;
    int back_point_index;
    double min_distance;
    int flag_global;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> w_smooth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> w_length;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> w_reference;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_1;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_1_T;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_2;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_2_T;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_3;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_3_T;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_small;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_small_T;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> p;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> f_T;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> f;
    Eigen::VectorXd lb;
    Eigen::VectorXd ub;
    // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> lb;
    // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> ub;

    // Eigen::Matrix<int, 362, 362> w_smooth;
    // Eigen::Matrix<int, 362, 362> w_length;
    // Eigen::Matrix<int, 362, 362> w_reference;
    // Eigen::Matrix<int, 362, 358> A_1;
    // Eigen::Matrix<int, 362, 358> A_1_T;
    // Eigen::Matrix<int, 362, 360> A_2;
    // Eigen::Matrix<int, 362, 360> A_2_T;
    // Eigen::Matrix<int, 362, 362> A_3;
    // Eigen::Matrix<int, 362, 362> A_3_T;
    // Eigen::Matrix<int, 362, 1> h_small;
    // Eigen::Matrix<int, 362, 1> h_small_T;
    // Eigen::Matrix<int, 362, 362> H;
    // Eigen::Matrix<int, 362, 362> p;
    // Eigen::Matrix<int, 1, 362> f_T;
    // Eigen::VectorXd f;




};

void readTraje(std::vector<waypoint> &vecTraj,const std::string &fileName);
void resampleTraje(double resample_interval, std::vector<waypoint> &vecTraj);
double calcPathLength(const std::vector<waypoint> &vecTraj);
void calcCurvePara(std::vector<waypoint> &curve_points);
void resampleOnStraight(std::vector<waypoint> &vecTraj,std::vector<waypoint> &curve_points,double resample_interval);
void resampleOnCurve(std::vector<waypoint> &vecTraj,std::vector<waypoint> &curve_points,double resample_interval);
void getDirAndK(std::vector<waypoint> &vecTraj, int index);
void createEdge(const std::vector<waypoint> &global_path, std::vector<waypoint> &upper_edge, std::vector<waypoint> &low_edge);
void visualize_road_trajectories(ros::Publisher pub, std::vector<waypoint> roads);

#endif