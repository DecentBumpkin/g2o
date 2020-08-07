#ifndef JSON_IO_HPP
#define JSON_IO_HPP

#include <iostream>
#include <unordered_map>
#include <memory>

#include "rapidjson/reader.h"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"


#include <Eigen/Dense>
#include <Eigen/Geometry>

int readPointsJSON(const char* name, std::unordered_map<int, std::shared_ptr<Eigen::Vector3d> >& map);
int writePointsJSON(const char* name, const std::unordered_map<int, std::shared_ptr<Eigen::Vector3d> >& map);
void convertFromLidarToWorld(const std::unordered_map<int, std::shared_ptr<Eigen::Vector3d> >& map_src,  
                            std::unordered_map<int, std::shared_ptr<Eigen::Vector3d> >& map_final, 
                            const Eigen::Affine3d& A);

#endif