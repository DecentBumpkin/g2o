#ifndef TYPES_DEMO_HPP
#define TYPES_DEMO_HPP
#include "g2o/core/base_multi_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/types/slam3d/se3quat.h"
#include <Eigen/Geometry>
#include <iostream>
#define G2O_TYPES_DEMO_API
namespace g2o {

class G2O_TYPES_DEMO_API VertexSE3Expmap : public BaseVertex<6, SE3Quat>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexSE3Expmap();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
        _estimate = SE3Quat();
    }

    virtual void oplusImpl(const number_t* update_)  {
        Eigen::Map<const Vector6> update(update_);
        setEstimate(SE3Quat::exp(update)*estimate());
        std::cout << "new translation: " << estimate().translation().transpose() << '\n';
    }
};

/**
 * \brief Point vertex, XYZ
 */
 class G2O_TYPES_DEMO_API VertexPointXYZ : public BaseVertex<3, Vector3>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPointXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
      _estimate.fill(0);
    }

    virtual void oplusImpl(const number_t* update)
    {
      Eigen::Map<const Vector3> v(update);
      _estimate += v;
    }
};


  /**
   * \brief Imu2Lidar(SE3), ImuPoseInWorld(SE3), FeaturePositionInWorld(XYZ)
   */
  class G2O_TYPES_DEMO_API Edge_SE3_SE3_XYZ : public BaseMultiEdge<3, Vector3>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      Edge_SE3_SE3_XYZ();

      void computeError()
      {
        const VertexSE3Expmap* v_se3_il = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        const VertexSE3Expmap* v_se3_wi = static_cast<const VertexSE3Expmap*>(_vertices[1]);
        const VertexPointXYZ*  v_xyz_feat = static_cast<const VertexPointXYZ*>(_vertices[2]);
        _error =  v_xyz_feat->estimate() - (v_se3_wi->estimate() * v_se3_il->estimate()).map(measurement());
        std::cout << "Error: \n" << _error << '\n';
      }
      void linearizeOplus() override;
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      static Matrix3 dRidx;
      static Matrix3 dRidy;
      static Matrix3 dRidz;
  };


  class G2O_TYPES_DEMO_API PointXYZMeasurementEdge:  public g2o::BaseUnaryEdge<3, Vector3, VertexPointXYZ>
  {
    PointXYZMeasurementEdge() {}
    
    void computeError()
    {
      // const VertexPosition3D* v = static_cast<const VertexPosition3D*>(_vertices[0]);
      // _error = v->estimate() - _measurement;
    }
    
    void linearizeOplus();

    virtual bool read(std::istream& /*is*/)
    {
      return false;
    }
    
    virtual bool write(std::ostream& /*os*/) const
    {
      return false;
    }
  };
  /**
   * rewrite EdgeSE3Expmap(SE3->SE3->SE3) to a unary edge(SE3->SE3->SE3) 
   * set information matrix to identity is FINE 
   * https://github.com/RainerKuemmerle/g2o/issues/325
  */
  class G2O_TYPES_DEMO_API ImuSE3MeasurementEdge : public g2o::BaseUnaryEdge<3, SE3Quat, VertexSE3Expmap>
  {
  public:
    ImuSE3MeasurementEdge(){}
    
    void computeError()
    {
      // const VertexPosition3D* v = static_cast<const VertexPosition3D*>(_vertices[0]);
      // _error = v->estimate() - _measurement;
    }
    
    void linearizeOplus();

    virtual bool read(std::istream& /*is*/)
    {
      return false;
    }
    
    virtual bool write(std::ostream& /*os*/) const
    {
      return false;
    }
  };

} // namespace

#endif
