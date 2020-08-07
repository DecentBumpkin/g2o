#include "types_static.hpp"

namespace g2o {

VertexSE3Expmap::VertexSE3Expmap() : BaseVertex<6, SE3Quat>() 
{
    IsPositionFixed = false;
}

bool VertexSE3Expmap::read(std::istream& ) 
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

bool VertexSE3Expmap::write(std::ostream& ) const 
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

VertexPointXYZ::VertexPointXYZ() : BaseVertex<3, Vector3>()
{
}

bool VertexPointXYZ::read(std::istream& )
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

bool VertexPointXYZ::write(std::ostream& ) const
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

Edge_SE3_XYZ::Edge_SE3_XYZ() : BaseBinaryEdge<3, Vector3,VertexSE3Expmap, VertexPointXYZ>()
{
    Edge_SE3_XYZ::dRidx << 
        0.0,0.0,0.0,
        0.0,0.0,1.0,
        0.0,-1.0,0.0;
    Edge_SE3_XYZ::dRidy << 
        0.0,0.0,-1.0,
        0.0,0.0,0.0,
        1.0,0.0,0.0;
    Edge_SE3_XYZ::dRidz << 
        0.0,1.0,0.0,
        -1.0,0.0,0.0,
        0.0,0.0,0.0;
}

bool Edge_SE3_XYZ::read(std::istream& )
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

bool Edge_SE3_XYZ::write(std::ostream& ) const
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

void Edge_SE3_XYZ::linearizeOplus(){
    const VertexSE3Expmap* v_se3_wl = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    // const VertexPointXYZ* v_xyz_w = static_cast<const VertexPointXYZ*>(_vertices[1]);

    SE3Quat T_se3_wl = v_se3_wl->estimate();
    Vector3 xyz_l = measurement();
    Vector3 xyz_w_meas = T_se3_wl.map(xyz_l);
    // Matrix3 R_wl = T_se3_wl.rotation().toRotationMatrix();
    
    _jacobianOplusXj = -Matrix3::Identity();
    _jacobianOplusXi.block<3,3>(0,3) = Matrix3::Identity();
    // _jacobianOplusXi.block<3,3>(0,3) = Matrix3::Zero();
    _jacobianOplusXi.block<3,1>(0,0) = - dRidx * xyz_w_meas;
    _jacobianOplusXi.block<3,1>(0,1) = - dRidy * xyz_w_meas;
    _jacobianOplusXi.block<3,1>(0,2) = - dRidz * xyz_w_meas;
    

}

/* define static members */
Matrix3 Edge_SE3_XYZ::dRidx; /* dRidx * p is the first column of skew-symmetrix(p) */
Matrix3 Edge_SE3_XYZ::dRidy;
Matrix3 Edge_SE3_XYZ::dRidz; 


} // end namespace

