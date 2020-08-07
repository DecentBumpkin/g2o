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
#include "g2o/stuff/command_args.h"
#include "types_static.hpp"

#include <yaml-cpp/yaml.h>
#include "lib/json_io.hpp"
using namespace Eigen;
using namespace std;


/* the only edge used in this example is EdgeProjectXYZ2UV, its linearizeOplus() defined in types_six_dof_expmap.cpp */
int main(int argc, char* argv[]) {

    //// Todo, read fix total station points and selected position from json 
    printf("%d %s\n", argc, argv[0]); /* TODO Parse Commandline*/

    int maxIterations;
    bool verbose;
    bool fixOrigin;
    g2o::CommandArgs arg;
    arg.param("i", maxIterations, 10, "perform n iterations");
    arg.param("v", verbose, false, "verbose output of the optimization process");
    arg.param("fix", fixOrigin, false, "whether fix the livox origin at measured position xyz");
    arg.parseArgs(argc, argv);

    printf("max iterations: %d\n"
            "verbose: %s\n"
            "fixOrigin: %s\n",
            maxIterations, 
            verbose?"YES":"NO",
            fixOrigin?"YES":"NO");

    std::string source_filename;
    std::string target_filename;
    std::shared_ptr<Eigen::Vector3d> fixed_origin_ptr;
    std::vector<double> guess_translation;
    std::vector<double> guess_eular_angle;
    try{
        YAML::Node config = YAML::LoadFile("../g2o/examples/demo/demo_static.yaml");
        guess_translation = config["initial_guess"]["translation"].as<std::vector<double> >();
        guess_eular_angle = config["initial_guess"]["eular_angle"].as<std::vector<double> >();
        printf("use initial guess:\n");
        printf("translation : %.3lf %.3lf %.3lf\n"
                "eular angle : %.3lf %.3lf %.3lf\n", 
                guess_translation[0], guess_translation[1], guess_translation[2],
                guess_eular_angle[0], guess_eular_angle[1], guess_eular_angle[2]);
        std::string origin_id = config["origin_id"].as<std::string>();
        double orig_x = config[origin_id]["x"].as<double>();
        double orig_y = config[origin_id]["y"].as<double>();
        double orig_z = config[origin_id]["z"].as<double>();
        printf("Use Fixed Origin id = %s\n", origin_id.c_str());
        fixed_origin_ptr = std::make_shared<Eigen::Vector3d>(orig_x, orig_y, orig_z);
        std::cout<<"Selected origin coord: \n" << fixed_origin_ptr->transpose() << std::endl;
        source_filename = config["source_file"].as<std::string>();
        target_filename = config["target_file"].as<std::string>();
        std::cout<< "source path: " << source_filename << std::endl;
        std::cout<< "target path: " << target_filename << std::endl;
    }
    catch(std::exception& e){
        std::cout<< e.what() << std::endl;
        printf("Could not load config file, total failure");
        return 1;
    }

    unordered_map<int, shared_ptr<Eigen::Vector3d> > xyz_l, xyz_w;
    if(readPointsJSON(source_filename.c_str(), xyz_l))
    {
        printf("Read Source failure!");
        return 1;
    }
    if(readPointsJSON(target_filename.c_str(), xyz_w))
    {
        printf("Read Target failure!");
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

    Eigen::Vector3d p_guess(guess_translation.data());
    Eigen::Quaterniond q_guess =  
        Eigen::AngleAxisd(guess_eular_angle[0], Vector3d::UnitX())* 
        Eigen::AngleAxisd(guess_eular_angle[1], Vector3d::UnitY())* 
        Eigen::AngleAxisd(guess_eular_angle[2], Vector3d::UnitZ());
    g2o::SE3Quat T_wl_guess(q_guess, p_guess);
    // Eigen::Quaterniond q_temp(Eigen::AngleAxisd(-0.6, Vector3d::UnitZ()).toRotationMatrix()); /* operator= not supported */

    std::cout << "Nominal source_frame -> target_frame SE3: \n" << T_wl_guess <<'\n';
    
    g2o::VertexSE3Expmap * v_se3_wl = new g2o::VertexSE3Expmap();
    if(fixOrigin) 
    {
        v_se3_wl-> setFixPositionMode(true);
        v_se3_wl-> setFixPosition(fixed_origin_ptr);
    }
    v_se3_wl->setId(0);
    v_se3_wl->setEstimate(T_wl_guess.inverse());
    optimizer.addVertex(v_se3_wl);

    int vertex_id = 0; /* 0 is reserved for v_se3_il */
    for (auto& it : xyz_l)
    {

        if(!xyz_w.count(it.first))
        {
            fprintf(stderr,"world collection does not include %d\n",it.first);
            continue;
        }
        g2o::VertexPointXYZ * v_xyz_w = new g2o::VertexPointXYZ();
        std::cout << (*xyz_w[it.first]).transpose() << std::endl;
        
        v_xyz_w->setId(++vertex_id);
        v_xyz_w->setFixed(true); /* fix the first camera pose */
        v_xyz_w->setEstimate(*xyz_w[it.first]); 
        optimizer.addVertex(v_xyz_w); 


        g2o::Edge_SE3_XYZ * e = new g2o::Edge_SE3_XYZ();
        e->setVertex(0, v_se3_wl);
        e->setVertex(1, v_xyz_w);
        std::cout << (*it.second).transpose() << std::endl;
        e->setMeasurement(*it.second);
        e->information() = Matrix3d::Identity()/( 0.02 * 0.02 );
        optimizer.addEdge(e);
    }


    optimizer.initializeOptimization();
    optimizer.setVerbose(verbose);
    optimizer.optimize(maxIterations);

    auto T_final = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertices().find(0)->second)->estimate();
    std::cout << " Initial Guess : \n" << T_wl_guess << '\n';
    std::cout << " Final Result  : \n" << T_final  << '\n';
    for(auto& it : xyz_l){
        if(!xyz_w.count(it.first))
        {
            continue;
        }
        std::cout << "Target: " << (*xyz_w[it.first]).transpose() << std::endl;
        std::cout << "Converted Source: " << T_final.map(*it.second).transpose() << std::endl;
        printf("id: %d, error: %.5lf\n", it.first, (*xyz_w[it.first] - T_final.map(*it.second)).norm());
        if(fixOrigin){
            double d1 = (*xyz_w[it.first] - *fixed_origin_ptr).norm();
            double d2 = it.second->norm();
            printf("distance compare: %d %.3lf %.3lf\n",it.first,d1, d2 );
        }
    }

    std::ofstream fmat;
    std::string mat_filename = source_filename;
    mat_filename.replace(mat_filename.find_last_of('.'),5,"_mat.txt");
    fmat.open (mat_filename.c_str(), ios::trunc | ios::out);
    printf("Writing matrix to %s\n",mat_filename.c_str());
    fmat <<  T_final << '\n';
    fmat.close();

    std::cout << "Final Transformation:\n" << T_final << std::endl;
    std::cout << "Meausred lidar origin in lidar_frame: \n" << T_final.inverse().map(*fixed_origin_ptr).transpose() << std::endl;
    std::cout << "Meausred lidar origin in lidar_frame: " << T_final.inverse().map(*fixed_origin_ptr).norm() << std::endl;
    
    std::unordered_map<int, std::shared_ptr<Eigen::Vector3d> > xyz_final;
    Eigen::Affine3d A_final;
    A_final.matrix() = T_final.to_homogeneous_matrix();
    convertFromLidarToWorld(xyz_l, xyz_final, A_final);
    std::string outputfile = source_filename.replace(source_filename.find_last_of('.'),5,"_converted.json");
    writePointsJSON(outputfile.c_str(), xyz_final);

    // for(auto it = xyz_l.begin() ; it != xyz_l.end(); it++){
    //     for(auto it2 = it ; it2 != xyz_l.end(); it2++){
    //         if(it == it2) continue;
    //     }
    // }
}
