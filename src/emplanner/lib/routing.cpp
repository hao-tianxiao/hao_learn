#include "../include/routing.h"
#include <iomanip>
#include <algorithm>
// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"
// eigen
#include <Eigen/Dense>
#include <iostream>





void readTraje(std::vector<waypoint> &vecTraj,const std::string &fileName){
    std::ifstream infile(fileName,std::ios::in);
    if(!infile.is_open())
    {
        std::cout<<"can not open fine"<<std::endl;
        return;
    }
    waypoint Point;
    std::string line;
    std::stringstream ss;
    while(getline(infile,line))
    {
        ss<<line;
        ss>>Point.x>>Point.y;
        vecTraj.push_back(Point);
        ss.clear();
    }
    infile.close();
}

void resampleTraje(double resample_interval, std::vector<waypoint> &vecTraj)
{
    std::vector<waypoint> original_trajectory(vecTraj);
    vecTraj.clear();
    vecTraj.push_back(original_trajectory.at(0));
    vecTraj.reserve(ceil(1.5*calcPathLength(original_trajectory)/resample_interval));
    for(int i = 1;i<original_trajectory.size();++i)
    {
        std::vector<waypoint> curve_points = {vecTraj.back(),
                                              original_trajectory.at(i),
                                              i<original_trajectory.size()-1? original_trajectory.at(i+1):original_trajectory.back()};
        calcCurvePara(curve_points);
        if(curve_points.at(1).k == 0)
        {
            resampleOnStraight(vecTraj,curve_points,resample_interval);
        }
        else{
            resampleOnCurve(vecTraj,curve_points,resample_interval);
        }
    }

}

double calcPathLength(const std::vector<waypoint> &vecTraj)
{
    double length = 0;
    for(int i = 0;i<vecTraj.size()-1;++i)
    {
        Eigen::Vector2d point_vector = {vecTraj.at(i+1).x - vecTraj.at(i).x,vecTraj.at(i+1).y - vecTraj.at(i).y};
        length += point_vector.norm();
    }
    return length;
}

void calcCurvePara(std::vector<waypoint> &curve_points){
    if(curve_points.size() != 3)
        return;
    waypoint &p0 = curve_points.at(0), &p1 = curve_points.at(1),&p2 = curve_points.at(2);
    double d =2*((p0.y-p2.y)*(p0.x-p1.x)-(p0.y-p1.y)*(p0.x-p2.x));
    if(fabs(d)<1e-8)
    {
        p1.k = 0;
        std::vector<double> Line = {(p1.x - p0.x),(p1.y - p0.y)};
        p1.dirAngle = atan2(Line.at(1),Line.at(0));
        return;

    }
    double a = p0.y*p0.y - p1.y*p1.y + p0.x*p0.x-p1.x*p1.x;
    double b = p0.y*p0.y - p2.y*p2.y + p0.x*p0.x-p2.x*p2.x;

    double cx = ((p0.y - p2.y)*a - (p0.y - p1.y)*b)/d;
    double cy = ((p0.x - p2.x)*a - (p0.x - p1.x)*b)/(-d);

    double dx = cx - p1.x;
    double dy = cy - p1.y;
    double R = sqrt(dx*dx+dy*dy);
    p1.k = 1/R;
    p0.k = 1/R;

    std::vector<double> differVector = {p2.x - p1.x,p2.y - p1.y};
    std::vector<double> dirVec = {-dy/R,dx/R};
    if((differVector.at(0)*dirVec.at(0)+ differVector.at(1)*dirVec.at(1))<0)
    {
        dirVec.at(0) = -dirVec.at(0);
        dirVec.at(1) = -dirVec.at(1);
    }
    p1.dirAngle = atan2(dirVec.at(1),dirVec.at(0));

    // calculate k and dir for p0
    dx = cx - p0.x;
    dy = cy - p0.y;
    differVector = {p1.x - p0.x,p1.y - p0.y};
    dirVec = {-dy/R,dx/R};
    if((differVector.at(0)*dirVec.at(0)+ differVector.at(1)*dirVec.at(1))<0)
    {
        dirVec.at(0) = -dirVec.at(0);
        dirVec.at(1) = -dirVec.at(1);
    }
    p0.dirAngle = atan2(dirVec.at(1),dirVec.at(0));
}

void resampleOnStraight(std::vector<waypoint> &vecTraj,std::vector<waypoint> &curve_points,double resample_interval){
    if(curve_points.size() != 3)
        return;
    waypoint prePoint(vecTraj.back());
    Eigen::Vector2d vecDiff = {curve_points.at(1).x - prePoint.x,curve_points.at(1).y - prePoint.y};
    double dist = vecDiff.norm();
    double coeff = resample_interval/dist;
    vecDiff[0] *= coeff;
    vecDiff[1] *= coeff;
    for(;dist>resample_interval;dist -= resample_interval)
    {
        prePoint.x += vecDiff[0];
        prePoint.y += vecDiff[1];
        vecTraj.push_back(prePoint);
    }
}

void resampleOnCurve(std::vector<waypoint> &vecTraj,std::vector<waypoint> &curve_points,double resample_interval)
{
    if(curve_points.size() != 3)
        return;
    waypoint prePoint(curve_points.at(0));
    double R = 1/curve_points.at(1).k;
    int dir = 0;

    // judge whither clockwise or anticlockwise
    Eigen::Vector2d p0 = {cos(prePoint.dirAngle), sin(prePoint.dirAngle)};
    Eigen::Vector2d p1 = {cos(curve_points.at(1).dirAngle), sin(curve_points.at(1).dirAngle)};

    double cross = p0[0]*p1[1]- p0[1]*p1[0];
    double dot = p0.dot(p1);
    double theta = acos(dot/(p0.norm()*p1.norm()));
    if(cross>0)
    {
        dir = 1;
    }
    else
    {
        dir = -1;//clockwise
    }

    double dist = fabs(theta)*R;
    double theta_diff = resample_interval*curve_points.at(1).k;
    for(;dist>resample_interval;dist -= resample_interval){

        if(vecTraj.size() == vecTraj.capacity())
            break;
        Eigen::Vector2d vec = {cos(prePoint.dirAngle+dir*theta_diff/2), sin(prePoint.dirAngle+dir*theta_diff/2)};
        vec = 2*R* sin(theta_diff/2)*vec;
        prePoint.dirAngle += dir*theta_diff;
        prePoint.x += vec[0];
        prePoint.y += vec[1];
        vecTraj.push_back(prePoint);
    }
}

void getDirAndK(std::vector<waypoint> &vecTraj, int index){
    if(index == 0 || index == vecTraj.size()-1)
    {
        return;
    }
    waypoint &p0 = vecTraj.at(index-1), &p1 = vecTraj.at(index),&p2 = vecTraj.at(index+1);
    double d =2*((p0.y-p2.y)*(p0.x-p1.x)-(p0.y-p1.y)*(p0.x-p2.x));
    if(fabs(d)<1e-8)
    {
        vecTraj.at(index).k = 0;
        double module = (p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y);
        std::vector<double> Line = {(p2.x - p1.x)/module,(p2.y - p1.y)/module};
        vecTraj.at(index).dirAngle = atan2(Line.at(1),Line.at(0));
        return;

    }
    double a = p0.y*p0.y - p1.y*p1.y + p0.x*p0.x-p1.x*p1.x;
    double b = p0.y*p0.y - p2.y*p2.y + p0.x*p0.x-p2.x*p2.x;

    double cx = ((p0.y - p2.y)*a - (p0.y - p1.y)*b)/d;
    double cy = ((p0.x - p2.x)*a - (p0.x - p1.x)*b)/(-d);

    double dx = cx - p1.x;
    double dy = cy - p1.y;
    double R = sqrt(dx*dx+dy*dy);
    vecTraj.at(index).k = 1/R;

    std::vector<double> differVector = {p2.x - p1.x,p2.y - p1.y};
    std::vector<double> dirVec = {-dy/R,dx/R};
    if((differVector.at(0)*dirVec.at(0)+ differVector.at(1)*dirVec.at(1))<0)
    {
        dirVec.at(0) = -dirVec.at(0);
        dirVec.at(1) = -dirVec.at(1);
    }


    vecTraj.at(index).dirAngle = atan2(dirVec.at(1),dirVec.at(0));
    if(index == 1)
    {
        dx = cx - p0.x;
        dy = cy - p0.y;
        R = sqrt(dx*dx+dy*dy);
        vecTraj.at(index-1).k = 1/R;
        differVector = {p1.x - p0.x,p1.y - p0.y};
        dirVec = {-dy/R,dx/R};
        if((differVector.at(0)*dirVec.at(0)+ differVector.at(1)*dirVec.at(1))<0)
        {
            dirVec.at(0) = -dirVec.at(0);
            dirVec.at(1) = -dirVec.at(1);
        }
        vecTraj.at(index-1).dirAngle = atan2(dirVec.at(1),dirVec.at(0));
    }
    if(index == vecTraj.size()-2)
    {
        dx = cx - p2.x;
        dy = cy - p2.y;
        R = sqrt(dx*dx+dy*dy);
        vecTraj.at(index+1).k = 1/R;
        differVector = {p2.x - p1.x,p2.y - p1.y};
        dirVec = {-dy/R,dx/R};
        if((differVector.at(0)*dirVec.at(0)+ differVector.at(1)*dirVec.at(1))<0)
        {
            dirVec.at(0) = -dirVec.at(0);
            dirVec.at(1) = -dirVec.at(1);
        }
        vecTraj.at(index+1).dirAngle = atan2(dirVec.at(1),dirVec.at(0));
    }
}

void createEdge(const std::vector<waypoint> &global_path, std::vector<waypoint> &upper_edge, std::vector<waypoint> &low_edge)
{
    upper_edge.reserve(global_path.size());
    low_edge.reserve(global_path.size());

    for(const auto & point : global_path)
    {
        waypoint temp_point;
        Eigen::Vector2d pos = {point.x,point.y};
        Eigen::Vector2d tor = {-sin(point.dirAngle), cos(point.dirAngle)};
        Eigen::Vector2d upper_point = pos+6*tor;
        temp_point.x = upper_point[0];
        temp_point.y = upper_point[1];
        temp_point.dirAngle = point.dirAngle;
        temp_point.k = point.k;
        upper_edge.push_back(temp_point);

        upper_point = pos-6*tor;
        temp_point.x = upper_point[0];
        temp_point.y = upper_point[1];
        temp_point.dirAngle = point.dirAngle;
        temp_point.k = point.k;
        low_edge.push_back(temp_point);
    }
}

void emplanner::rviz_road(ros::Publisher marker_pub, std::vector<waypoint> roads){
   ////可视化中间线    
    visualization_msgs::Marker points;
    points.header.frame_id = "odom";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action =  visualization_msgs::Marker::ADD;
    points.pose.orientation.w =  1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;
    // Create the vertices for the points 
    for (size_t i = 0; i < roads.size(); i++)
    {  
            geometry_msgs::Point p;
            p.x =  roads[i].x;
            // std::cout<<"///// p.x//// "<< p.x<<std::endl;
            p.y =  roads[i].y;
            // std::cout<<"///// p.y//// "<< p.y<<std::endl;
            p.z = 0;
            points.points.push_back(p);       
    }
    marker_pub.publish(points);   
}

////可视化左右边界
void visualize_road_trajectories(ros::Publisher pub, std::vector<waypoint> roads)
{
    visualization_msgs::MarkerArray v_trajectories;
    int count = 0;
    const int size =  roads.size();
    for(;count<size;count++){
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = "map";
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.color.r = 1;
        v_trajectory.color.g = 0;
        v_trajectory.color.b = 0;
        v_trajectory.color.a = 0.8;
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::ADD;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
      // ostringstream str;
      //  str<<v_trajectory.id;
     //   v_trajectory.text=str.str();
        v_trajectory.scale.x = 1;
        geometry_msgs::Point p;
        for (size_t i = 0; i < roads.size(); i++)
        {  
            geometry_msgs::Point p;
            p.x =  roads[i].x;
            // std::cout<<"///// p.x//// "<< p.x<<std::endl;
            p.y =  roads[i].y;
            // std::cout<<"///// p.y//// "<< p.y<<std::endl;
            p.z = 0;
            v_trajectory.points.push_back(p);       
        }
        v_trajectories.markers.push_back(v_trajectory);
    }
    pub.publish(v_trajectories);
}

int emplanner::find_projection_point(std::vector<waypoint> global_path, nav_msgs::Odometry::ConstPtr odometryMsg)
{
    vehicle_position vehicle_position_current;
    vehicle_position_current.x = odometryMsg->pose.pose.position.x;
    vehicle_position_current.y = odometryMsg->pose.pose.position.y;


    for(int i=0; i<global_path.size(); i++)
    {
        min_distance = calcDistance(global_path[projection_point_index].x- vehicle_position_current.x,global_path[projection_point_index].y- vehicle_position_current.y);
        // double vehicle_to_projectionPoint = sqrt(pow(global_path[i].x - vehicle_position_current.x, 2) + pow(global_path[i].y - vehicle_position_current.y, 2));
        double vehicle_to_projectionPoint = calcDistance(global_path[i].x - vehicle_position_current.x, global_path[i].y - vehicle_position_current.y);
        if(vehicle_to_projectionPoint < min_distance)
        {
            // projection_point_index = global_path[i].index;
            projection_point_index = i;
            // std::cout<<"global_path[i].index"<< global_path[i].index <<std::endl;

            // std::cout<<"i"<< i <<std::endl;
            min_distance = vehicle_to_projectionPoint;
        }

    }
    return projection_point_index;
}

void emplanner::definite_reference_line_range(int projection_point_index, std::vector<waypoint> global_path)
{
    if(projection_point_index > 30 && global_path.size() - projection_point_index > 150)
    {
        flag_global = 1;
        front_point_index = projection_point_index - 30;
        back_point_index = projection_point_index + 150;
    }else if(projection_point_index <= 30 && global_path.size() - projection_point_index > 150)
    {
        flag_global = 1;
        front_point_index = 0;
        back_point_index = 180;
    }else if(projection_point_index > 30 && global_path.size() - projection_point_index <= 150)
    {
        flag_global = 1;
        front_point_index = (global_path.size() - 181);
        back_point_index = global_path.size();
    }else
    {
        flag_global = 0;
        std::cout<<"the global waypoints is too short!"<<std::endl;
    }
}

double emplanner::calcDistance(const double start, const double& end)
{
    double x = end- start;
    double y = end- start;
    return sqrt(x*x+ y*y);
}

void emplanner::w_init()
{
    w_smooth = Eigen::MatrixXd::Zero(362, 362);
    w_length = Eigen::MatrixXd::Zero(362, 362);
    w_reference = Eigen::MatrixXd::Zero(362, 362);
    for(int i=0; i<361; i++)
    {
        w_smooth(i, i) = 1;
        w_length(i, i) = 1;
        w_reference(i, i) = 1;
    }
    // std::cout<<"w_smooth"<< w_smooth<<std::endl;
}

void emplanner::H_construction(Eigen::SparseMatrix<double> &hessian)
{
    H = Eigen::MatrixXd::Zero(362, 362);
    H = 2*(w_smooth*A_1_T*A_1+w_length*A_2_T*A_2+w_reference*A_3_T*A_3);
    for(int i=0; i<361; i++)
    {
        for(int j=0; j<361; j++)
        {
            hessian.insert(i,j)= H(i,j);
        }
    }
    // std::cout<<"H.rows"<< H.rows()<<std::endl;
    // std::cout<<"H.cols"<< H.cols()<<std::endl;

}

void emplanner::A1_construction()
{   
    A_1_T = Eigen::MatrixXd::Zero(362, 358);
    A_1 = Eigen::MatrixXd::Zero(358, 362);
    for(int i=0; i<358; i+=2)
    {
        A_1_T(i,i) = 1;
        A_1_T(i+2,i) = -2;
        A_1_T(i+4,i) = 1;
        A_1_T(i+1,i+1) = 1;
        A_1_T(i+3,i+1) = -2;
        A_1_T(i+5,i+1) = 1;
    }
    A_1 = A_1_T.transpose();
    //     std::cout << "A_1_T" << std::endl
    // << A_1_T << std::endl; //输出为m*1的向量
}

void emplanner::A2_construction()
{
    A_2_T = Eigen::MatrixXd::Zero(362, 360);
    A_2 = Eigen::MatrixXd::Zero(360, 362);
    for(int i=0; i<360; i+=2)
    {
        A_2_T(i,i) = 1;
        A_2_T(i+2,i) = -1;
        A_2_T(i+1,i+1) = 1;
        A_2_T(i+3,i+1) = -1;
    }
    A_2 = A_2_T.transpose();

}

void emplanner::A3_construction()
{
    A_3_T = Eigen::MatrixXd::Zero(362, 362);
    A_3 = Eigen::MatrixXd::Zero(362, 362);
    for(int i=0; i<361; i++)
    {
        A_3(i,i) = 1;
    }
    // A_3 = A_3_T.transpose();
}

void emplanner::h_small_construction()
{

    h_small = Eigen::MatrixXd::Zero(362, 1);
    h_small_T = Eigen::MatrixXd::Zero(1, 362);
    if(flag_global)
    {
        for(int i=0; i<362; i+=2)
        {
            h_small(i,0)=-2*global_path[front_point_index + i/2].x;
            h_small(i+1,0)=-2*global_path[front_point_index + i/2].y;
        //     std::cout << "global_path[front_point_index + i/2].x" << std::endl
        //             << global_path[front_point_index + i/2].x << std::endl//输出为m*1的向量
        //             << "global_path[front_point_index + i/2].y" << std::endl//输出为m*1的向量
        //             << global_path[front_point_index + i/2].y << std::endl//输出为m*1的向量
        //             << "ront_point_index + i/2" << std::endl//输出为m*1的向量
        //             << front_point_index + i/2 << std::endl; //输出为m*1的向量
        }
    }
    h_small_T = h_small.transpose();
}

void emplanner::f_construction(Eigen::VectorXd &gradient)
{
    f_T = Eigen::MatrixXd::Zero(1, 362);
    f = Eigen::MatrixXd::Zero(362, 1);
    f_T = w_reference*h_small_T;
    f = f_T.transpose();
    for(int i=0; i<361; i++)
    {
        gradient(i,0) = f(i,0);
    }
}
void emplanner::p_construction(Eigen::SparseMatrix<double> &p)
{
    for(int i=0; i<361; i++)
    {
        p.insert(i,i) = 1;
    }
}
void emplanner::lb_construction(Eigen::VectorXd &lowerBound)
{   
// lowerBound.fill(-1);
    for(int i=0; i<362; i+=2)
    {
        // lowerBound[i] = global_path[front_point_index + i/2].x - 0.21;
        // lowerBound[i+1] = global_path[front_point_index + i/2].y - 0.21;
        lowerBound[i] = global_path[front_point_index + i/2].x - 0.1;
        lowerBound[i+1] = global_path[front_point_index + i/2].y - 0.1;
    }

}   

void emplanner::ub_construction(Eigen::VectorXd &upperBound)
{
// upperBound.fill(1);
    for(int i=0; i<362; i+=2)
    {
        // upperBound[i] = global_path[front_point_index + i/2].x + 0.21;
        // upperBound[i+1] = global_path[front_point_index + i/2].y + 0.21;
        upperBound[i] = global_path[front_point_index + i/2].x + 0.1;
        upperBound[i+1] = global_path[front_point_index + i/2].y + 0.1;
    }
}
