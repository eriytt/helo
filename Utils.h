#ifndef UTILS_H
#define UTILS_H

#include <algorithm>

#include <vector>

#include <cmath>

#include <btBulletDynamicsCommon.h>

#include <Ogre.h>

namespace HeloUtils {

  static const float PI = M_PI;
  static const float PI_2 = M_PI_2;
  static const float PI_4 = M_PI_4;

  inline float GetAirDensity(float altitude)
  {
    return 1.2041;
  }

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

  template < typename T >
  inline T Deg2Rad(const T &x)
  {
    return x / 360.0 * 2.0 * PI;
  }

  template < typename T >
  inline T Rad2Deg(const T &x)
  {
    return x * 360.0 / (2.0 * PI);
  }


  inline void LocalApplyForce(btRigidBody *body, const btVector3 &force, const btVector3 &point)
  {
    const btMatrix3x3 &rot = body->getCenterOfMassTransform().getBasis();
    btVector3 appliedForce = rot * force;
    btVector3 appliedPoint = rot * point;
    body->applyForce(appliedForce, appliedPoint);
  }

  inline void LocalApplyImpulse(btRigidBody *body, const btVector3 &impulse, const btVector3 &point)
  {
    const btMatrix3x3 &rot = body->getCenterOfMassTransform().getBasis();
    btVector3 appliedImpulse = rot * impulse;
    btVector3 appliedPoint = rot * point;
    body->applyImpulse(appliedImpulse, appliedPoint);
  }


  inline void LocalApplyTorque(btRigidBody *body, const btVector3 &torque)
  {
    const btMatrix3x3 &rot = body->getCenterOfMassTransform().getBasis();
    btVector3 appliedTorque = rot * torque;
    body->applyTorque(appliedTorque);
  }

  inline void LocalApplyTorqueImpulse(btRigidBody *body, const btVector3 &torqueImpulse)
  {
    const btMatrix3x3 &rot = body->getCenterOfMassTransform().getBasis();
    btVector3 appliedTorqueImpulse = rot * torqueImpulse;
    body->applyTorqueImpulse(appliedTorqueImpulse);
  }

  inline void Bullet2OgreVector(const btVector3 &btVec, Ogre::Vector3 &oVec)
  {
    oVec.x = btVec.getX();
    oVec.y = btVec.getY();
    oVec.z = btVec.getZ();
  }

  inline Ogre::Vector3 Bullet2OgreVector(const btVector3 &btVec)
  {
    return Ogre::Vector3(btVec.m_floats);
  }

  inline btVector3 Ogre2BulletVector(const Ogre::Vector3 &oVec)
  {
    return btVector3(oVec.x, oVec.y, oVec.z);
  }


  inline void Ogre2BulletVector(const Ogre::Vector3 &oVec, btVector3 &btVec)
  {
    btVec.setX(oVec.x);
    btVec.setY(oVec.y);
    btVec.setZ(oVec.z);
    
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

      const std::pair<float, float> &u = fdata[upper_index];
      const std::pair<float, float> &l = fdata[upper_index - 1];
      x -= l.first;

      /* Linearly interpolate the found curve section */
      return l.second + (u.second - l.second) * (x /(u.first - l.first));
    }

    void addDataPoint(float x, float y)
    {
      fdata.push_back(std::pair<float, float>(x, y));
    }

    void mirrorData(float at, float multiplier = 1.0)
    {
      std::vector< std::pair<float, float> > mirror_data;
      size_t num_values = fdata.size();
      for (int i = num_values - 1; i >= 0; --i)
        {
          std::pair<float, float> m = fdata[i];
          m.first = at - m.first;
          m.second *= multiplier;
          mirror_data.push_back(m);
        }
      for (size_t i = 0; i < num_values; ++i)
          fdata.push_back(mirror_data[i]);
    }

  };

  class Trackable
  {
  protected:
    Ogre::Vector3 cameraPosition;

  public:
    typedef struct {
      float p;
      float minGroundOffset;
    } CameraParams;

  public:
    Trackable(const Ogre::Vector3 &campos) : cameraPosition(campos) {}
    Trackable() : cameraPosition((Ogre::Vector3::NEGATIVE_UNIT_Z * 20) + (Ogre::Vector3::UNIT_Y * 3)) {}
    virtual Ogre::SceneNode *getSceneNode() = 0;
    virtual const Ogre::Vector3 &getTrackOffset() {return Ogre::Vector3::ZERO;}
    virtual const Ogre::Vector3 &getCameraUp() {return Ogre::Vector3::UNIT_Y;}
    virtual bool cameraFollow() {return true;}
    virtual const Ogre::Vector3 &getCameraPosition() {return cameraPosition;}
    virtual CameraParams getCameraParameters() 
    {
      CameraParams params;
      params.p = 1.0;
      params.minGroundOffset = 5.0;
      return params;
    }
  };

  template <typename T1, typename T2>
  Ogre::Real Fraction(T1 nominator, T2 denominator) {return static_cast<Ogre::Real>(nominator) / static_cast<Ogre::Real>(denominator);}

} // namespace HeloUtils

#endif // UTILS_H
