#include <ros/ros.h>
// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"
// eigen
#include <Eigen/Dense>
#include <iostream>

#include "../include/routing.h"

// using namespace emplanner;
// using namespace qp_constrution;
void emplanner::odomCallBack(const nav_msgs::Odometry::ConstPtr odometryMsg)
{
    //Visual global path
    rviz_road(this->marker_pub, global_path);
    // visualize_road_trajectories(marker_pub1, left_edge);
    // visualize_road_trajectories(marker_pub2, right_edge);
    
    //reference line smoothing
    //find vehicle position's projection point on the reference line
    int nearest_index = find_projection_point(global_path, odometryMsg);
    // std::cout<<"nearest_index"<< nearest_index<<std::endl;

    //definite the range of reference line smoothing
    definite_reference_line_range(nearest_index, global_path);
    std::cout<<"nearest_index"<< nearest_index<<std::endl;
    std::cout<<"front_point_index"<< front_point_index<<std::endl;
    std::cout<<"back_point_index"<< back_point_index<<std::endl;


    //calculate related parameters

    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian(362, 362);    //P: n*n正定矩阵,必须为稀疏矩阵SparseMatrix
    Eigen::VectorXd gradient(362);                    //Q: n*1向量
    Eigen::SparseMatrix<double> linearMatrix(362, 362); //A: m*n矩阵,必须为稀疏矩阵SparseMatrix
    Eigen::VectorXd lowerBound(362);                  //L: m*1下限向量
    Eigen::VectorXd upperBound(362);                  //U: m*1上限向量
    
            //     std::cout << "lowerBound:" << std::endl
            //   << lowerBound.rows() << std::endl
            //   << lowerBound.cols() << std::endl;
    w_init();
    A1_construction();
    A2_construction();
    A3_construction();
    h_small_construction();
    f_construction(gradient);
    p_construction(linearMatrix);
    lb_construction(lowerBound);
    ub_construction(upperBound);
    H_construction(hessian);


    // // instantiate the solver
    OsqpEigen::Solver solver;

    //             std::cout << "lowerBound" << std::endl
    // << lowerBound << std::endl; //输出为m*1的向量
    //             std::cout << "upperBound" << std::endl
    // << upperBound << std::endl; //输出为m*1的向量

    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(362);   //变量数n
    solver.data()->setNumberOfConstraints(362); //约束数m
    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);

    // std::cout << "hessian" << std::endl
    // << hessian << std::endl; //输出为m*1的向量

    // // // instantiate the solver
    solver.initSolver();

    Eigen::VectorXd QPSolution;

    // solve the QP problem
    solver.solve();

    QPSolution = solver.getSolution();
    // std::cout << "QPSolution" << std::endl
    //           << QPSolution[1] << std::endl//输出为m*1的向量
    //           << QPSolution[2] << std::endl//输出为m*1的向量
    //           << QPSolution[362] << std::endl; //输出为m*1的向量

    local_path.resize(181);
    for(int i=0, j=0; i<180, j<360; i++, j+=2)
    {
        local_path[i].x = QPSolution[j];
        local_path[i].y = QPSolution[j+1];
            std::cout << "local_path[i].x" << std::endl
            << local_path[0].x << std::endl; //输出为m*1的向量
            std::cout << "local_path[i].y" << std::endl
            << local_path[0].y << std::endl; //输出为m*1的向量
    }

    // //Visual local path
    rviz_road(this->marker_pub3, local_path);
    local_path.clear();
}



void emplanner::nodeStart(int argc, char **argv)
{
    ros::init(argc, argv, "emplanner");
    ros::NodeHandle nh;

    //load global path    // 读取全局路径
    readTraje(global_path,"/home/hao/hao_learn/src/path_record/data/road_path_odom1.txt");
    // readTraje(global_path,"/home/hao/hao_learn/src/emplanner/data/waypoints.txt");
    // resampleTraje(1,global_path);//discrete the path
    // for(int i = 0;i<global_path.size();i++)
    // {
    //     getDirAndK(global_path, i);
    //     global_path[i].index = i;
    // }
    // createEdge(global_path,left_edge,right_edge);//creat the right and left edge
    std::cout<<"global_path_size"<<global_path.size()<<std::endl;


    this->marker_pub = nh.advertise<visualization_msgs::Marker>("road_rviz", 1);
    // this->marker_pub1 = nh.advertise<visualization_msgs::MarkerArray>("road_rviz2", 1);
    // this->marker_pub2 = nh.advertise<visualization_msgs::MarkerArray>("road_rviz3", 1);
    this->marker_pub3 = nh.advertise<visualization_msgs::Marker>("road_rviz4", 1);
    this->odom = nh.subscribe("/odom", 1, &emplanner::odomCallBack, this);
    // this->qp_constrcution = nh.subscribe("/rosout", 1, &emplanner::qp_callback, this);
    ros::spin();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    emplanner node;
    node.nodeStart(argc, argv);
    return 0;
}