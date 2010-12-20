#ifndef UTILS_H
#define UTILS_H

#include "algorithm"

#include "vector"

#include "math.h"

#include <btBulletDynamicsCommon.h>

namespace HeloUtils {

  static float PI = M_PI;
  static float PI_2 = M_PI_2;
  static float PI_4 = M_PI_4;

  template < typename T >
  inline T clamp(const T &max, const T &min, const T &val)
  {
    return std::min(std::max(val, min), max);
  }

  template < typename T >
  inline T unit_clamp(const T &val)
  {
    return clamp(static_cast<T>(1.0), static_cast<T>(-1.0), val);
  }

  template < typename T >
  inline T punit_clamp(const T &val)
  {
    return clamp(static_cast<T>(1.0), static_cast<T>(0.0), val);
  }

  template < typename T >
  inline T nunit_clamp(const T &val)
  {
    return clamp(static_cast<T>(0.0), static_cast<T>(-1.0), val);
  }

  template < typename T >
  inline T POW2(const T &x)
  {
    return x * x;
  }

  inline void LocalApplyForce(btRigidBody *body, const btVector3 &force, const btVector3 &point)
  {
    const btMatrix3x3 &rot = body->getCenterOfMassTransform().getBasis();
    btVector3 appliedForce = rot * force;
    btVector3 appliedPoint = rot * point;
    body->applyForce(appliedForce, appliedPoint);
  }

  inline void LocalApplyTorque(btRigidBody *body, const btVector3 &torque)
  {
    const btMatrix3x3 &rot = body->getCenterOfMassTransform().getBasis();
    btVector3 appliedTorque = rot * torque;
    body->applyTorque(appliedTorque);
  }

  class PieceWiseLinearFunction {
    std::vector< std::pair<float, float> > fdata;

  public:
    float operator[](float x)
    {
      /*Find the appropriate curve section*/
      unsigned int upper_index = 1;
      while(upper_index < fdata.size() && x > fdata[upper_index].first)
	++upper_index;

      std::pair<float, float> u = fdata[upper_index];
      std::pair<float, float> l = fdata[upper_index - 1];
      x -= l.first;

      /* Linearly interpolate the found curve section */
      return l.second + (u.second - l.second) * (x /(u.first - l.first));
    }

    void addDataPoint(float x, float y)
    {
      fdata.push_back(std::pair<float, float>(x, y));
    }

  };



} // namespace HeloUtils

#endif // UTILS_H
