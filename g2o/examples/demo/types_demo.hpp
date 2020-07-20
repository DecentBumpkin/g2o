#ifndef TYPES_DEMO_HPP
#define TYPES_DEMO_HPP
#include "g2o/core/base_multi_edge.h"
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
        const VertexSE3Expmap* v0 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
        const VertexPointXYZ*  v2 = static_cast<const VertexPointXYZ*>(_vertices[2]);
        _error = v1->estimate().map(v0->estimate().map(measurement())) - v2->estimate();
      }
      void linearizeOplus();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      static Matrix3 dRidx;
      static Matrix3 dRidy;
      static Matrix3 dRidz;
  };
}

#endif
