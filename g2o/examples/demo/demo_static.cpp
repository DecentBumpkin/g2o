#include <iostream>
#include <stdint.h>
#include <unordered_map>

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

#include "rapidjson/reader.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

using namespace Eigen;
using namespace std;


/* the only edge used in this example is EdgeProjectXYZ2UV, its linearizeOplus() defined in types_six_dof_expmap.cpp */
int main(int argc, const char* argv[]) {

    FILE* fp = fopen("/home/wei/PolyExplore/LivoxStatic/build/tmptmp.json", "r"); // non-Windows use "r"
    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    
    rapidjson::Document doc;
    doc.ParseStream(is);
    fclose(fp);

    std::vector<Eigen::Vector3d> xyz_l;
    if (doc.HasMember("points") && doc["points"].IsArray())
    {
        printf("reading lvx %u picked points...\n", doc["points"].Size());
        const rapidjson::Value &array = doc["points"];
        for(auto it = array.Begin(); it != array.End(); it++){
        
            if(it->HasMember("x") && (*it)["x"].IsDouble() &&
                it->HasMember("y") && (*it)["y"].IsDouble() &&
                it->HasMember("z") && (*it)["z"].IsDouble() ){
                printf("Read point %lf %lf %lf\n",(*it)["x"].GetDouble(),(*it)["y"].GetDouble(),(*it)["z"].GetDouble());
                xyz_l.emplace_back((*it)["x"].GetDouble(),(*it)["y"].GetDouble(),(*it)["z"].GetDouble());
            }
        }

    }


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

    Eigen::Vector3d p_guess(6,0,0);
    Eigen::Quaterniond q_guess =  
        Eigen::AngleAxisd(0, Vector3d::UnitX())* 
        Eigen::AngleAxisd(0, Vector3d::UnitY())* 
        Eigen::AngleAxisd(0.6, Vector3d::UnitZ());
    g2o::SE3Quat T_wl_guess(q_guess, p_guess);
    // Eigen::Vector3d p_temp(-2.016, 3.546, 0.829);
    // Eigen::Quaterniond q_temp(Eigen::AngleAxisd(-0.6, Vector3d::UnitZ()).toRotationMatrix());
    // g2o::SE3Quat T_wl_guess(q_temp, p_temp);

    std::cout << "Nominal Imu->Lidar SE3: \n" << T_wl_guess <<'\n';
    
    g2o::VertexSE3Expmap * v_se3_wl = new g2o::VertexSE3Expmap();
    v_se3_wl->setId(0);
    v_se3_wl->setEstimate(T_wl_guess.inverse());
    optimizer.addVertex(v_se3_wl);

/*real pain to hard code*/
    double xyz_w_arr[9][3] = {
        {4.8682, 3.9116,  1.9851},
        {4.8819, 2.4407,  1.3232},
        {4.8967, 1.0257,  1.4942},
        {4.9001, 0.5605,  2.1421},
        {4.9218, -1.3016,  1.8082},
        {4.9380, -3.2454,  1.8852},
        {4.9760,   -6.2514,   1.9442},
        {3.7604,  -7.5817,   2.5301},
        {-0.3135,-1.4014,  2.4282 }
    };

    int vertex_id = 0; /* 0 is reserved for v_se3_il */
    for (size_t i = 0; i < 9; i++ )
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
        std::cout << xyz_l[i].transpose() << std::endl;
        e->setMeasurement(xyz_l[i]);
        e->information() = Matrix3d::Identity()/( 0.02 * 0.02 );
        optimizer.addEdge(e);
    }


    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(20);

    auto T_final = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertices().find(0)->second)->estimate();
    std::cout << " Initial Guess : \n" << T_wl_guess << '\n';
    std::cout << " Final Result  : \n" << T_final  << '\n';
    for(int i = 0;i < 9; i++){
        Eigen::Map<const Eigen::Vector3d> xyz_w_map(xyz_w_arr[i]);
        Eigen::Vector3d xyz_w(xyz_w_map);
        std::cout << "total station: " << xyz_w.transpose() << std::endl;
        std::cout << "livox aligned: " << T_final.map(xyz_l[i]).transpose() << std::endl;
        printf("error: %.5lf\n", (xyz_w - T_final.map(xyz_l[i])).norm());

    }

    std::cout << "Final Transformation:\n" << T_final << std::endl;
    Eigen::Vector3d lidar(-2.016, 3.546, 0.829);
    std::cout << "livox fixture: " << T_final.inverse().map(lidar).transpose() << std::endl;
    std::cout << "livox fixture: " << T_final.inverse().map(lidar).norm() << std::endl;

}