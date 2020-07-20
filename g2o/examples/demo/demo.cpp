#include <iostream>
#include <stdint.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "types_demo.hpp"

using namespace Eigen;
using namespace std;

/* the only edge used in this example is EdgeProjectXYZ2UV, its linearizeOplus() defined in types_six_dof_expmap.cpp */
int main(int argc, const char* argv[]){

    printf("%d %s\n", argc, argv[0]);
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver; /* 6-dof state,  3-dof meas. */
    linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();


    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( /* use Levenberg */
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );
    optimizer.setAlgorithm(solver);

    Eigen::Vector3d p_actual(g2o::Sampler::gaussRand(0.0, 0.01),
                    g2o::Sampler::gaussRand(-0.5, 0.01),
                    g2o::Sampler::gaussRand(0.25, 0.01));
    Eigen::Quaterniond q_actual =  
        Eigen::AngleAxisd(g2o::Sampler::gaussRand(0.0, 0.05), Vector3d::UnitX())* 
        Eigen::AngleAxisd(g2o::Sampler::gaussRand(0.0, 0.05), Vector3d::UnitY())* 
        Eigen::AngleAxisd(g2o::Sampler::gaussRand(0.0, 0.05), Vector3d::UnitZ());
    g2o::SE3Quat T_il_actual(q_actual, p_actual);
    g2o::SE3Quat T_il_nominal(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(0,-0.5,0.25));

    std::cout << "Nominal Imu->Lidar SE3: \n" << T_il_nominal <<'\n';
    std::cout << "Actual Imu->Lidar SE3: \n" << T_il_actual <<'\n';  
    
    g2o::VertexSE3Expmap * v_se3_il = new g2o::VertexSE3Expmap();
    v_se3_il->setId(0);
    v_se3_il->setEstimate(T_il_nominal);
    optimizer.addVertex(v_se3_il);

    int vertex_id = 0; /* 0 is reserved for v_se3_il */
    /* add imu poses in world frame vertices, v_se3_wi */
    std::vector<std::pair<g2o::SE3Quat, int> > imu_poses_gt;
    for(int i = 0; i < 2; i++){
        /* radius, angle, height are ground truth */
        double radius_gt = g2o::Sampler::gaussRand( 75.0, 8.0);
        double angle_gt  = g2o::Sampler::uniformRand(-M_PI, M_PI);
        double height_gt = g2o::Sampler::gaussRand( 0.5, 0.25);
        double x_gt = radius_gt * std::cos(angle_gt);
        double y_gt = radius_gt * std::sin(angle_gt);
        Eigen::Vector3d p_gt(x_gt, y_gt, height_gt);
        /* p is "measured" position, used as vertex estimate */
        Eigen::Vector3d p_meas = p_gt + Vector3d(g2o::Sampler::gaussRand(0.0, 0.02),
        g2o::Sampler::gaussRand(0.0, 0.02),
        g2o::Sampler::gaussRand(0.0, 0.02));
        std::cout<<"Imu Ground Truth position: \n" << p_gt.transpose() << "\n\n";
        std::cout<<"Imu Measured position: \n" << p_meas.transpose() << "\n\n";
        std::cout<<"Imu position Error: \n" << p_meas.transpose() - p_gt.transpose() << '\n';


        double roll_gt  = g2o::Sampler::uniformRand(-0.10, 0.10);
        double pitch_gt = g2o::Sampler::uniformRand(-0.30, 0.30);
        double yaw_gt   = g2o::Sampler::gaussRand(angle_gt + M_PI, 0.20);
        Eigen::Quaterniond q_gt = 
            Eigen::AngleAxisd(roll_gt , Vector3d::UnitX()) * 
            Eigen::AngleAxisd(pitch_gt, Vector3d::UnitY()) * 
            Eigen::AngleAxisd(yaw_gt  , Vector3d::UnitZ());
        Eigen::Quaterniond q_meas = 
            Eigen::AngleAxisd(roll_gt  + g2o::Sampler::gaussRand(0.0, 0.05), Vector3d::UnitX()) * 
            Eigen::AngleAxisd(pitch_gt + g2o::Sampler::gaussRand(0.0, 0.05), Vector3d::UnitY()) * 
            Eigen::AngleAxisd(yaw_gt   + g2o::Sampler::gaussRand(0.0, 0.05), Vector3d::UnitZ());

        g2o::SE3Quat T_wi_gt(q_gt, p_gt);
        g2o::VertexSE3Expmap * v_se3_wi = new g2o::VertexSE3Expmap();
        v_se3_wi->setId(++vertex_id);
        v_se3_wi->setEstimate(g2o::SE3Quat(q_meas,p_meas));
        optimizer.addVertex(v_se3_wi);

        imu_poses_gt.emplace_back(T_wi_gt, vertex_id);
        std::cout << "Imu Pose Ground Truth : \n" << imu_poses_gt.back().first << '\n';
        std::cout << "Imu Pose Measurement: \n" << v_se3_wi->estimate() << '\n';
        std::cout << "Imu Pose error: \n" << imu_poses_gt.back().first.inverse()* v_se3_wi->estimate() << '\n';

    }

    /* add feature point 3d vertices, v_xyz*/
    std::vector<std::pair<Eigen::Vector3d, int> > feature_points_gt;
    for (size_t i = 0; i < 5; i++ )
    {
        Eigen::Vector3d xyz_w_gt(
            g2o::Sampler::uniformRand(-50.0, 50.0),
            g2o::Sampler::uniformRand(-50.0, 50.0),
            g2o::Sampler::uniformRand(0.0, 5.0) );
        Eigen::Vector3d xyz_w_meas = xyz_w_gt + Eigen::Vector3d(
            g2o::Sampler::gaussRand(0.0, 0.02),
            g2o::Sampler::gaussRand(0.0, 0.02),
            g2o::Sampler::gaussRand(0.0, 0.02) );        
        g2o::VertexPointXYZ * v_xyz_feat = new g2o::VertexPointXYZ();
        v_xyz_feat->setId(++vertex_id);
        v_xyz_feat->setFixed(true); /* fix the first camera pose */
        v_xyz_feat->setEstimate(xyz_w_meas); 
        optimizer.addVertex(v_xyz_feat); 
        feature_points_gt.emplace_back(xyz_w_gt, vertex_id);
    }
    /* add ternary edges */
    for(size_t i = 0; i < imu_poses_gt.size(); i++){
        for(size_t j  = 0; j < feature_points_gt.size(); j++){
            printf("--%3zu %3zu\n", i, j);
            double noise_std = 0.02;
            g2o::Edge_SE3_SE3_XYZ * e = new g2o::Edge_SE3_SE3_XYZ();
            auto v0 = dynamic_cast<g2o::VertexSE3Expmap*>(v_se3_il);
            e->setVertex(0, v0);
            auto v1 = dynamic_cast<g2o::VertexSE3Expmap*>
                (optimizer.vertices().find(imu_poses_gt[i].second)->second);
            e->setVertex(1, v1);
            auto v2 = dynamic_cast<g2o::VertexPointXYZ*>
                (optimizer.vertices().find(feature_points_gt[j].second)->second);
            e->setVertex(2, v2);
            
            cout<< "Compare feature points" << (v2->estimate() - feature_points_gt[j].first).transpose() << "\n\n";

            Eigen::Vector3d xyz_l_meas = (v1->estimate() * T_il_actual).inverse().map(v2->estimate()) + Vector3d(
                g2o::Sampler::gaussRand(0.0, 0.02),
                g2o::Sampler::gaussRand(0.0, 0.02),
                g2o::Sampler::gaussRand(0.0, 0.02) ); 
            e->setMeasurement(xyz_l_meas);
            e->information() = Matrix3d::Identity()/( noise_std * noise_std );
            optimizer.addEdge(e);
        }
    }
    optimizer.optimize(5);

}
