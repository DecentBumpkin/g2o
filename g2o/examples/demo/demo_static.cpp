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
#include "types_static.hpp"

using namespace Eigen;
using namespace std;


/* the only edge used in this example is EdgeProjectXYZ2UV, its linearizeOplus() defined in types_six_dof_expmap.cpp */
int main(int argc, const char* argv[]) {

    printf("%d %s\n", argc, argv[0]);
    // int numIteration = atoi(argv[1]);
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver; /* 6-dof state,  3-dof meas. */
    linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();


    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( /* use Levenberg */
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );
    optimizer.setAlgorithm(solver);

    Eigen::Vector3d p_guess(4,0,0);
    Eigen::Quaterniond q_guess =  
        Eigen::AngleAxisd(0, Vector3d::UnitX())* 
        Eigen::AngleAxisd(0, Vector3d::UnitY())* 
        Eigen::AngleAxisd(0.1, Vector3d::UnitZ());
    g2o::SE3Quat T_wl_guess(q_guess, p_guess);
    std::cout << "Nominal Imu->Lidar SE3: \n" << T_wl_guess <<'\n';
    
    g2o::VertexSE3Expmap * v_se3_wl = new g2o::VertexSE3Expmap();
    v_se3_wl->setId(0);
    v_se3_wl->setEstimate(T_wl_guess);
    optimizer.addVertex(v_se3_wl);

    double xyz_w_arr[7][3] = {
        {4.8682, 3.9116,  1.9851},
        {4.8819, 2.4407,  1.3232},
        {4.8967, 1.0257,  1.4942},
        {4.9001, 0.5605,  2.1421},
        {4.9218, -1.3016,  1.8082},
        {4.9380, -3.2454,  1.8852},
        {-0.3135,-1.4014,  2.4282}
    };

    double xyz_l_arr[7][3] = {
        {8.483001 , 3.995000, -0.424000},
        {8.564000 , 2.563000, -1.144000},
        {8.832001 , 1.163000, -1.074000},
        {9.044001 , 0.681000, -0.454000},
        {9.253000 , -1.157000 , -0.855000},
        {9.579000 , -3.050000, -0.892000},
        {4.310000 ,-2.008000, 0.686000} 
    }; 

    printf("bebhe");
    int vertex_id = 0; /* 0 is reserved for v_se3_il */
    for (size_t i = 0; i < 7; i++ )
    {
        g2o::VertexPointXYZ * v_xyz_w = new g2o::VertexPointXYZ();
        Eigen::Map<const Eigen::Vector3d> xyz_w_map(xyz_w_arr[i]);
        Eigen::Vector3d xyz_w_vec(xyz_w_map);
        std::cout << xyz_w_vec.transpose() << std::endl;
        
        v_xyz_w->setId(++vertex_id);
        v_xyz_w->setFixed(true); /* fix the first camera pose */
        v_xyz_w->setEstimate(xyz_w_vec); 
        optimizer.addVertex(v_xyz_w); 


        g2o::Edge_SE3_XYZ * e = new g2o::Edge_SE3_XYZ();
        e->setVertex(0, v_se3_wl);
        e->setVertex(1, v_xyz_w);
        Eigen::Map<const Eigen::Vector3d> xyz_l_map(xyz_l_arr[i]);
        Eigen::Vector3d xyz_l(xyz_l_map);
        std::cout << xyz_l.transpose() << std::endl;
        e->setMeasurement(xyz_l);
        e->information() = Matrix3d::Identity()/( 0.02 * 0.02 );
        optimizer.addEdge(e);
    }


    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(20);

    auto T_final = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertices().find(0)->second)->estimate();
    std::cout << " Initial Guess : \n" << T_wl_guess << '\n';
    std::cout << " Final Result  : \n" << T_final  << '\n';
    for(int i = 0;i < 7; i++){
        Eigen::Map<const Eigen::Vector3d> xyz_w_map(xyz_w_arr[i]);
        Eigen::Map<const Eigen::Vector3d> xyz_l_map(xyz_l_arr[i]);
        Eigen::Vector3d xyz_w(xyz_w_map);
        Eigen::Vector3d xyz_l(xyz_l_map);
        std::cout << "total station: " << xyz_w.transpose() << std::endl;
        std::cout << "livox aligned: " << T_final.map(xyz_l).transpose() << std::endl;
        printf("error: %.3lf\n", (xyz_w - T_final.map(xyz_l)).norm());

    }
}