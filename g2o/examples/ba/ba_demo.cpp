// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <stdint.h>

#include <unordered_set>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h" /* VertexSE3Expmap EdgeProjectXYZ2UV defined here; VertexSBAPointXYZ defined in types_sba.h*/
//#include "g2o/math_groups/se3quat.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"

using namespace Eigen;
using namespace std;

class Sample {
 public:
  static int uniform(int from, int to) { return static_cast<int>(g2o::Sampler::uniformRand(from, to)); }
};
/* the only edge used in this example is EdgeProjectXYZ2UV, its linearizeOplus() defined in types_six_dof_expmap.cpp */
int main(int argc, const char* argv[]){
  if (argc<2)
  {
    cout << endl;
    cout << "Please type: " << endl;
    cout << "ba_demo [PIXEL_NOISE] [OUTLIER RATIO] [ROBUST_KERNEL] [STRUCTURE_ONLY] [DENSE]" << endl;
    cout << endl;
    cout << "PIXEL_NOISE: noise in image space (E.g.: 1)" << endl;
    cout << "OUTLIER_RATIO: probability of spuroius observation  (default: 0.0)" << endl;
    cout << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)" << endl;
    cout << "STRUCTURE_ONLY: performe structure-only BA to get better point initializations (0 or 1; default: 0==false)" << endl;
    cout << "DENSE: Use dense solver (0 or 1; default: 0==false)" << endl;
    cout << endl;
    cout << "Note, if OUTLIER_RATIO is above 0, ROBUST_KERNEL should be set to 1==true." << endl;
    cout << endl;
    exit(0);
  }

  double PIXEL_NOISE = atof(argv[1]);
  double OUTLIER_RATIO = 0.0;

  if (argc>2)  {
    OUTLIER_RATIO = atof(argv[2]);
  }

  bool ROBUST_KERNEL = false;
  if (argc>3){
    ROBUST_KERNEL = atoi(argv[3]) != 0;
  }
  bool STRUCTURE_ONLY = false;
  if (argc>4){
    STRUCTURE_ONLY = atoi(argv[4]) != 0;
  }

  bool DENSE = false;
  if (argc>5){
    DENSE = atoi(argv[5]) != 0;
  }

  cout << "PIXEL_NOISE: " <<  PIXEL_NOISE << endl;
  cout << "OUTLIER_RATIO: " << OUTLIER_RATIO<<  endl;
  cout << "ROBUST_KERNEL: " << ROBUST_KERNEL << endl;
  cout << "STRUCTURE_ONLY: " << STRUCTURE_ONLY<< endl;
  cout << "DENSE: "<<  DENSE << endl;



  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver; /* 6-dof state with 3-dof measurement */
  if (DENSE) {
    linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
  } else {
    linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
  }

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( /* use Levenberg */
    g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
  );
  optimizer.setAlgorithm(solver);


  vector<Vector3d> true_points; /* 500 sample points, -1.5 <= x <= 1.5, -0.5 <= y <= 0.5, 3 <= z <= 4  */
  for (size_t i=0;i<500; ++i)
  {
    true_points.push_back(Vector3d((g2o::Sampler::uniformRand(0., 1.)-0.5)*3,
                                   g2o::Sampler::uniformRand(0., 1.)-0.5,
                                   g2o::Sampler::uniformRand(0., 1.)+3));
  }

  double focal_length= 1000.;
  Vector2d principal_point(320., 240.); /* center of the camera axis pixel */

  vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat> > true_poses; /* camera poses, not sampled points */
  g2o::CameraParameters * cam_params
      = new g2o::CameraParameters (focal_length, principal_point, 0.); /* camera params, for intrinsic matrix ? */
  cam_params->setId(0);

  if (!optimizer.addParameter(cam_params)) {
    assert(false); /* essentially exit(1) */
  }

  cout<< "-----------------------\n";
  Vector3d trans(2,0,0), temp(1,0,0);
  Eigen:: Quaterniond q;
  q.setIdentity(); /* identity matrix, no rotation */
  g2o::SE3Quat pose(q, trans); /* simple x-axis translation matrix */
  g2o::VertexSE3Expmap * v_se3 = new g2o::VertexSE3Expmap();
  v_se3->setEstimate(pose); /* estimate of camera poses */
  cout << "Estimate: \n" << v_se3->estimate() << '\n';
  cout << "Converted: \n" << v_se3->estimate().map(temp) << '\n';
  cout<< "-----------------------\n";


  /** 15 camera anti-poses, first one is fixed, trans is x-axis only, 0 <= dx <= 0.6 
   * According to the definition of computeError in types_xis_dof_expmap.h, the value/estimate of 
   * this SE3 vertex is actually a transformation from world_frame to camera_local_frame, which 
   * means the "trans" vector3d is NOT the camera poses in world_frame, instead it is the opposite:
   * world origin's pose in camera_frame, which I term as "Camera anti-pose"
   * 
   * Please note, should this "trans" vector3d the position of camera in world_frame, then it actually 
   * converts points from camera_frame to world_frame, see the small example above, say "trans" is [2,0,0]
   * then a point (1,0,0) in the camera_frame is mapped to (3,0,0) in world_frame, no the other way around
  */
  
  int vertex_id = 0;
  for (size_t i=0; i<15; ++i) {
    Vector3d trans(i*0.04-1.,0,0);

    Eigen:: Quaterniond q;
    q.setIdentity(); /* identity matrix, no rotation */
    g2o::SE3Quat pose(q,trans); /* simple x-axis translation matrix */
    g2o::VertexSE3Expmap * v_se3 = new g2o::VertexSE3Expmap();
    v_se3->setId(vertex_id);
    if (i<2){
      v_se3->setFixed(true); /* fix the first camera pose */
    }
    v_se3->setEstimate(pose); /* estimate of camera poses */
    optimizer.addVertex(v_se3); /* add new camera pose as vertex */
    true_poses.push_back(pose); /* store true poses for statistical analysis */
    vertex_id++;
  }
  int point_id=vertex_id; /* points will be added after camera poses, both types are VERTEX in the graph*/
  int point_num = 0;
  double sum_diff2 = 0;

  cout << endl;
  unordered_map<int,int> pointid_2_trueid;
  unordered_set<int> inliers;

  /* iterate over all sampled true points */
  for (size_t i=0; i<true_points.size(); ++i){
    g2o::VertexSBAPointXYZ * v_p
        = new g2o::VertexSBAPointXYZ();
    v_p->setId(point_id);
    v_p->setMarginalized(true); /* marginalized */
    v_p->setEstimate(true_points.at(i) /* huge random noise */
                     + Vector3d(g2o::Sampler::gaussRand(0., 1),
                                g2o::Sampler::gaussRand(0., 1),
                                g2o::Sampler::gaussRand(0., 1)));
    /* num_obs is each point's occurrence in 15 poses, 0 <= num_obs <= 15 */
    int num_obs = 0;
    for (size_t j=0; j<true_poses.size(); ++j){
      Vector2d z = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));
      if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480){
        ++num_obs;
      }
    }
    /* if the point appears in multiple camera poses */
    if (num_obs>=2){
      optimizer.addVertex(v_p); /* then add this point as a vertex, need to add edges connecting thie point-vertex with pose-vertices */
      bool inlier = true;
      /* since graph construction is just one-time operation, high complexity is ok */
      for (size_t j=0; j<true_poses.size(); ++j){
        Vector2d z
            = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));

        if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480){
          double sam = g2o::Sampler::uniformRand(0., 1.);
          if (sam<OUTLIER_RATIO){ /* recap: OUTLIER_RATIO is the ratio of outlier, default is 0.0 */
            z = Vector2d(Sample::uniform(0,640), /* uniform distribution, just garbage */
                         Sample::uniform(0,480));
            inlier= false; /* if outlier, then this point is essentially garbage, huge huge noise */
          }
          z += Vector2d(g2o::Sampler::gaussRand(0., PIXEL_NOISE),
                        g2o::Sampler::gaussRand(0., PIXEL_NOISE)); /* add some pixel noise */
          g2o::EdgeProjectXYZ2UV * e
              = new g2o::EdgeProjectXYZ2UV(); /* projection-edge that connects an point-vertex with a pose-vertex */
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                       (optimizer.vertices().find(j)->second)); /* ->first is "id", and ->second is "value" ? */
          e->setMeasurement(z);
          e->information() = Matrix2d::Identity();
          if (ROBUST_KERNEL) {
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
          }
          e->setParameterId(0, 0);
          optimizer.addEdge(e);
        }
      }

      if (inlier){
        inliers.insert(point_id);
        Vector3d diff = v_p->estimate() - true_points[i];

        sum_diff2 += diff.dot(diff);
      }
      pointid_2_trueid.insert(make_pair(point_id,i));
      ++point_id;
      ++point_num;
    }
  }
  cout << endl;
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  if (STRUCTURE_ONLY){ /*  */
    g2o::StructureOnlySolver<3> structure_only_ba; /* performs optimization on the landmarks while the poses are kept fixed */
    cout << "Performing structure-only BA:"   << endl;
    g2o::OptimizableGraph::VertexContainer points;
    for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
      g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
      if (v->dimension() == 3)
        points.push_back(v);
    }
    structure_only_ba.calc(points, 24);
  }
  //optimizer.save("test.g2o");
  cout << endl;
  cout << "Performing full BA:" << endl;
  optimizer.optimize(24);
  cout << endl;
  cout << "Point error before optimisation (inliers only): " << sqrt(sum_diff2/inliers.size()) << endl;
  point_num = 0;
  sum_diff2 = 0;
  for (unordered_map<int,int>::iterator it=pointid_2_trueid.begin();
       it!=pointid_2_trueid.end(); ++it){
    g2o::HyperGraph::VertexIDMap::iterator v_it
        = optimizer.vertices().find(it->first);
    if (v_it==optimizer.vertices().end()){
      cerr << "Vertex " << it->first << " not in graph!" << endl;
      exit(-1);
    }
    g2o::VertexSBAPointXYZ * v_p
        = dynamic_cast< g2o::VertexSBAPointXYZ * > (v_it->second);
    if (v_p==0){
      cerr << "Vertex " << it->first << "is not a PointXYZ!" << endl;
      exit(-1);
    }
    Vector3d diff = v_p->estimate()-true_points[it->second];
    if (inliers.find(it->first)==inliers.end())
      continue;
    sum_diff2 += diff.dot(diff);
    ++point_num;
  }
  cout << "Point error after optimisation (inliers only): " << sqrt(sum_diff2/inliers.size()) << endl;
  cout << endl;
}
