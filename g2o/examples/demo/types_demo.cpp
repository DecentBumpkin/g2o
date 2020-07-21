#include "types_demo.hpp"

namespace g2o {

VertexSE3Expmap::VertexSE3Expmap() : BaseVertex<6, SE3Quat>() 
{
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

Edge_SE3_SE3_XYZ::Edge_SE3_SE3_XYZ() : BaseMultiEdge<3, Vector3>()
{
    resize(3); /* void BaseMultiEdge::resize(size_t size) */
    Edge_SE3_SE3_XYZ::dRidx << 
        0.0,0.0,0.0,
        0.0,0.0,1.0,
        0.0,-1.0,0.0;
    Edge_SE3_SE3_XYZ::dRidy << 
        0.0,0.0,-1.0,
        0.0,0.0,0.0,
        1.0,0.0,0.0;
    Edge_SE3_SE3_XYZ::dRidz << 
        0.0,1.0,0.0,
        -1.0,0.0,0.0,
        0.0,0.0,0.0;
}

bool Edge_SE3_SE3_XYZ::read(std::istream& )
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

bool Edge_SE3_SE3_XYZ::write(std::ostream& ) const
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

void Edge_SE3_SE3_XYZ::linearizeOplus(){
    const VertexSE3Expmap* v_se3_il = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    const VertexSE3Expmap* v_se3_wi = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    // const VertexPointXYZ*  v2 = static_cast<const VertexPointXYZ*>(_vertices[2]);

    SE3Quat T_il = v_se3_il->estimate();
    SE3Quat T_wi = v_se3_wi->estimate();
    Vector3 xyz_l = measurement();
    Vector3 xyz_i = T_il.map(xyz_l);
    Vector3 xyz_w = T_wi.map(xyz_i);
    Matrix3 R_wi = T_wi.rotation().toRotationMatrix();
    
    _jacobianOplus[1].block<3,3>(0,3) = -Matrix3::Identity();
    _jacobianOplus[1].block<3,1>(0,0) = dRidx * xyz_w;
    _jacobianOplus[1].block<3,1>(0,1) = dRidy * xyz_w;
    _jacobianOplus[1].block<3,1>(0,2) = dRidz * xyz_w;
    _jacobianOplus[0].block<3,3>(0,3) = -R_wi;
    _jacobianOplus[0].block<3,1>(0,0) = R_wi * dRidx * xyz_i;
    _jacobianOplus[0].block<3,1>(0,1) = R_wi * dRidy * xyz_i;
    _jacobianOplus[0].block<3,1>(0,2) = R_wi * dRidz * xyz_i;
    _jacobianOplus[2] = Matrix3::Identity();

    std::cout <<"Jacobian: \n" << _jacobianOplus[0] << '\n';

}

/* define static members */
Matrix3 Edge_SE3_SE3_XYZ::dRidx; /* dRidx * p is the first column of skew-symmetrix(p) */
Matrix3 Edge_SE3_SE3_XYZ::dRidy;
Matrix3 Edge_SE3_SE3_XYZ::dRidz; 

} // end namespace

