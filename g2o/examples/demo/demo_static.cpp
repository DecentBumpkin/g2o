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
#include "rapidjson/prettywriter.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"

using namespace Eigen;
using namespace std;

int readPointsJSON(const char* name, std::vector<Eigen::Vector3d>& points);
int writePointsJSON(const char* name, const std::vector<Eigen::Vector3d>& points);
void convertFromLidarToWorld(const std::vector<Eigen::Vector3d>& points_src,  
                            std::vector<Eigen::Vector3d>& points_final, 
                            const g2o::SE3Quat& T);
/* the only edge used in this example is EdgeProjectXYZ2UV, its linearizeOplus() defined in types_six_dof_expmap.cpp */
int main(int argc, const char* argv[]) {

    printf("%d %s\n", argc, argv[0]); /* TODO Parse Commandline*/
    
    std::vector<Eigen::Vector3d> xyz_l, xyz_w;
    if(readPointsJSON("/home/wei/PolyExplore/LivoxStatic/build/points_in_lidar_frame.json", xyz_l))
    {
        printf("Read lidar_frame failure!");
        return 1;
    }
    std::vector<int> indices{1,2,3,4,5,6,7,8,10};
    if(readPointsJSON("/home/wei/PolyExplore/LivoxStatic/scripts/total_station_xyz.json", xyz_w))
    {
        printf("Read world_frame failure!");
        return 1;
    }

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
    // Eigen::Quaterniond q_temp(Eigen::AngleAxisd(-0.6, Vector3d::UnitZ()).toRotationMatrix()); /* operator= not supported */
    // g2o::SE3Quat T_wl_guess(q_temp, p_temp);

    std::cout << "Nominal Imu->Lidar SE3: \n" << T_wl_guess <<'\n';
    
    g2o::VertexSE3Expmap * v_se3_wl = new g2o::VertexSE3Expmap();
    v_se3_wl->setId(0);
    v_se3_wl->setEstimate(T_wl_guess.inverse());
    optimizer.addVertex(v_se3_wl);

    int vertex_id = 0; /* 0 is reserved for v_se3_il */
    for (size_t i = 0; i < 9; i++ )
    {
        g2o::VertexPointXYZ * v_xyz_w = new g2o::VertexPointXYZ();
        std::cout << xyz_w[indices[i]].transpose() << std::endl;
        
        v_xyz_w->setId(++vertex_id);
        v_xyz_w->setFixed(true); /* fix the first camera pose */
        v_xyz_w->setEstimate(xyz_w[indices[i]]); 
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
        std::cout << "total station: " << xyz_w[indices[i]].transpose() << std::endl;
        std::cout << "livox aligned: " << T_final.map(xyz_l[i]).transpose() << std::endl;
        printf("error: %.5lf\n", (xyz_w[indices[i]] - T_final.map(xyz_l[i])).norm());
    }

    std::cout << "Final Transformation:\n" << T_final << std::endl;
    Eigen::Vector3d lidar(-2.016, 3.546, 0.829);
    std::cout << "Meausred lidar origin in lidar_frame: \n" << T_final.inverse().map(lidar).transpose() << std::endl;
    std::cout << "Meausred lidar origin in lidar_frame: " << T_final.inverse().map(lidar).norm() << std::endl;
    
    std::vector<Eigen::Vector3d> xyz_final;
    convertFromLidarToWorld(xyz_l, xyz_final, T_final);
    writePointsJSON("copy_of_world.json", xyz_final);
    
}

void convertFromLidarToWorld(const std::vector<Eigen::Vector3d>& points_src,  
                            std::vector<Eigen::Vector3d>& points_final, 
                            const g2o::SE3Quat& T){
    points_final.clear();
    for(auto& point_src : points_src){
        points_final.push_back(T.map(point_src));
    }
    assert(point_src.size() == points_final.size());
}

int writePointsJSON(const char* name, const std::vector<Eigen::Vector3d>& points){

    rapidjson::Document doc;
    doc.SetObject();
    rapidjson::Value points_js = rapidjson::Value(rapidjson::kArrayType);
    for(auto& point : points)
    {
        rapidjson::Value item(rapidjson::kObjectType);
        rapidjson::Value point_js(rapidjson::kObjectType);
        item.SetDouble(point[0]);
        point_js.AddMember("x", item, doc.GetAllocator());
        item.SetDouble(point[1]);
        point_js.AddMember("y", item, doc.GetAllocator());
        item.SetDouble(point[2]);
        point_js.AddMember("z", item, doc.GetAllocator()); /* item is moved */
        points_js.PushBack(point_js, doc.GetAllocator()); /* point is moved */
    }
    doc.AddMember("points", points_js, doc.GetAllocator() ); /* points is "moved" */
    // save to json format
    FILE* fp = fopen(name, "w"); // non-Windows use "w"
    if(!fp) return 1;
    char writeBuffer[65536];
    rapidjson::FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
    rapidjson::PrettyWriter<rapidjson::FileWriteStream> fwriter(os);
    fwriter.SetMaxDecimalPlaces(6);
    doc.Accept(fwriter);
    fclose(fp);

    return 0;
}

int readPointsJSON(const char* name, std::vector<Eigen::Vector3d>& points){
    FILE* fp = fopen(name, "r"); // non-Windows use "r"
    if(!fp) return 1;
    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    rapidjson::Document doc;
    doc.ParseStream(is);

    points.clear();
    if (doc.HasMember("points") && doc["points"].IsArray())
    {
        printf("reading lvx %u picked points...\n", doc["points"].Size());
        const rapidjson::Value &array = doc["points"];
        for(auto it = array.Begin(); it != array.End(); it++){
        
            if(it->HasMember("x") && (*it)["x"].IsDouble() &&
                it->HasMember("y") && (*it)["y"].IsDouble() &&
                it->HasMember("z") && (*it)["z"].IsDouble() ){
                printf("Read point %lf %lf %lf\n",(*it)["x"].GetDouble(),(*it)["y"].GetDouble(),(*it)["z"].GetDouble());
                points.emplace_back((*it)["x"].GetDouble(),(*it)["y"].GetDouble(),(*it)["z"].GetDouble());
            }
        }
    }
    fclose(fp);
    return 0;
}