#ifndef AGSPECTATOR
#define AGSPECTATOR

#pragma once

#include <UnigineComponentSystem.h>
#include <UnigineControls.h>
#include <UnigineGame.h>

class AGSpectator : public Unigine::ComponentBase {
 public:
  // methods
  AGSpectator(const Unigine::NodePtr &node, int num);
  virtual ~AGSpectator() {}
  using __this_class = AGSpectator;
  const char *getClassName() const override { return "AGSpectator"; }
  COMPONENT_UPDATE(update);

  // property name
  // PROP_NAME("ag_spectator");

  // parameters
  PROP_PARAM(Mask, param_collision_mask, 1);
  PROP_PARAM(Float, param_collision_radius, 0.5f);
  PROP_PARAM(Float, param_min_velocity, 2.0f);
  PROP_PARAM(Float, param_max_velocity, 4.0f);
  PROP_PARAM(Float, param_acceleration, 4.0f);
  PROP_PARAM(Float, param_damping, 8.0f);
  PROP_PARAM(Float, param_turning, 90.0f);
  PROP_PARAM(Float, param_min_theta_angle, -90.0f);
  PROP_PARAM(Float, param_max_theta_angle, 90.0f);

 public:
  // transform
  // (note: use this methods if you want to change position of the player!
  // node->setWorldTransform() doesn't work)
  void setTransform(const Unigine::Math::Mat4 &transform);
  void setWorldTransform(const Unigine::Math::Mat4 &transform);

  // control
  void setControlled(int controlled);
  int isControlled() const;

  // collision
  void setCollision(int collision);
  int getCollision() const;

  // collision mask
  void setCollisionMask(int mask);
  int getCollisionMask() const;

  // collision radius
  void setCollisionRadius(float radius);
  float getCollisionRadius() const;

  // minimum velocity
  void setMinVelocity(float velocity);
  float getMinVelocity() const;

  // maximum velocity
  void setMaxVelocity(float velocity);
  float getMaxVelocity() const;

  // minimum theta angle
  void setMinThetaAngle(float angle);
  float getMinThetaAngle() const;

  // maximum theta angle
  void setMaxThetaAngle(float angle);
  float getMaxThetaAngle() const;

  // acceleration
  void setAcceleration(float acceleration);
  float getAcceleration() const;

  // damping
  void setDamping(float damping);
  float getDamping() const;

  // turning
  void setTurning(float turning);
  float getTurning() const;

  // phi angle
  void setPhiAngle(float angle);
  float getPhiAngle() const;

  // theta angle
  void setThetaAngle(float angle);
  float getThetaAngle() const;

  // view direction
  void setViewDirection(const Unigine::Math::vec3 &view);
  const Unigine::Math::vec3 &getViewDirection() const;

  // contacts
  int getNumContacts() const;
  const Unigine::ShapeContactPtr &getContact(int num) const;
  float getContactDepth(int num) const;
  Unigine::Math::vec3 getContactNormal(int num) const;
  Unigine::ObjectPtr getContactObject(int num);
  Unigine::Math::Vec3 getContactPoint(int num) const;
  Unigine::ShapePtr getContactShape(int num) const;
  int getContactSurface(int num) const;

 protected:
  // world main loop
  void update();

 private:
  void read_property_parameters();

  // update transformation
  void update_transform();

  // flush
  void flush_transform();

  Unigine::Math::vec3 velocity;
  Unigine::CameraPtr camera;

  int controlled;
  Unigine::ControlsPtr controls;

  int collision;
  Unigine::ShapeSpherePtr shape;

  float min_velocity;
  float max_velocity;
  float min_theta_angle;
  float max_theta_angle;
  float acceleration;
  float damping;
  float turning;

  Unigine::Math::Vec3 position;
  Unigine::Math::vec3 direction;
  float phi_angle;
  float theta_angle;

  Unigine::Vector<Unigine::ShapeContactPtr> contacts;
};

#endif /* AGSPECTATOR */
