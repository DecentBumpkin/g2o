#ifndef TYPES_STATIC_HPP
#define TYPES_STATIC_HPP
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/types/slam3d/se3quat.h"
#include <Eigen/Geometry>
#include <iostream>
#define G2O_TYPES_DEMO_API
namespace g2o {

class G2O_TYPES_DEMO_API VertexSE3Expmap : public BaseVertex<6, SE3Quat>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexSE3Expmap(bool _fixOrigin = false); // by default in declaration, do NOT fixed origin

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
        _estimate = SE3Quat();
    }

    virtual void oplusImpl(const number_t* update_)  {
        Eigen::Map<const Vector6> update(update_);
        if(!fixOrigin)
        {
          setEstimate(SE3Quat::exp(update)*estimate());
        }
        else{
          SE3Quat temp = SE3Quat::exp(update) * estimate();
          temp.setTranslation(Eigen::Vector3d(-2.016, 3.546, 0.829));
          std::cout << temp << std::endl;
          setEstimate(temp);
        }
    }

    bool fixOrigin;
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


class G2O_TYPES_DEMO_API Edge_SE3_XYZ : public BaseBinaryEdge<3, Vector3, VertexSE3Expmap, VertexPointXYZ>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Edge_SE3_XYZ();

    void computeError()
    {
      const VertexSE3Expmap* v_se3_wl = static_cast<const VertexSE3Expmap*>(_vertices[0]);
      const VertexPointXYZ*  v_xyz_w = static_cast<const VertexPointXYZ*>(_vertices[1]);

      _error =  v_se3_wl->estimate().map(measurement()) - v_xyz_w->estimate();
      // std::cout << "Error: \n" << _error << '\n';
    }
    void linearizeOplus() override;
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    static Matrix3 dRidx;
    static Matrix3 dRidy;
    static Matrix3 dRidz;
};

} // namespace
#endif