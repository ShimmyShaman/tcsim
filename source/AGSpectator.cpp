#include "AGSpectator.h"

#include <UnigineGeometry.h>

REGISTER_COMPONENT(AGSpectator);

#define PLAYER_SPECTATOR_IFPS (1.0f / 60.0f)
#define PLAYER_SPECTATOR_CLAMP 89.9f
#define PLAYER_SPECTATOR_COLLISIONS 4

using namespace Unigine;
using namespace Math;

AGSpectator::AGSpectator(const NodePtr &node, int num) : ComponentBase(node, num)
{
  if (!node)  // registerComponent
    return;

  if (node->getType() != Node::PLAYER_DUMMY) {
    Log::error("AGSpectator::init(): add this component to a PlayerDummy!\n");
    return;
  }

  PlayerPtr player = checked_ptr_cast<Player>(node);
  camera = player->getCamera();
  controls = player->getControls();
  controlled = 1;

  shape = ShapeSphere::create();
  shape->setContinuous(0);

  position = Vec3(-3.f, -3.f, 0.25f);
  direction = normalize(vec3(1.f, 1.f, 0.f));
  // phi_angle = 45.0f;
  // theta_angle = 0.0f;

  read_property_parameters();

  flush_transform();
  update_transform();
}

void AGSpectator::update()
{
  float ifps = Game::getIFps();
  vec3 up = camera->getUp();

  // impulse
  vec3 impulse = vec3_zero;

  // ortho basis
  vec3 tangent, binormal;
  Geometry::orthoBasis(camera->getUp(), tangent, binormal);

  // controls
  if (controlled && controls) {
    // Log::message("phi:%.2f  theta:%.2f\n", phi_angle, theta_angle);
    // direction
    phi_angle += controls->getMouseDX();
    theta_angle += controls->getMouseDY();
    if (controls->getState(Controls::STATE_TURN_UP))
      theta_angle -= turning * ifps;
    if (controls->getState(Controls::STATE_TURN_DOWN))
      theta_angle += turning * ifps;
    if (controls->getState(Controls::STATE_TURN_LEFT))
      phi_angle -= turning * ifps;
    if (controls->getState(Controls::STATE_TURN_RIGHT))
      phi_angle += turning * ifps;
    theta_angle = clamp(theta_angle, min_theta_angle, max_theta_angle);

    // new basis
    vec3 x = (quat(up, -phi_angle) * quat(tangent, -theta_angle)) * binormal;
    vec3 y = normalize(cross(up, x));
    vec3 z = normalize(cross(x, y));

    // direction
    direction = x;

    // movement
    if (controls->getState(Controls::STATE_FORWARD))
      impulse += x;
    if (controls->getState(Controls::STATE_BACKWARD))
      impulse -= x;
    if (controls->getState(Controls::STATE_MOVE_LEFT))
      impulse += y;
    if (controls->getState(Controls::STATE_MOVE_RIGHT))
      impulse -= y;
    if (controls->getState(Controls::STATE_CROUCH))
      impulse -= z;
    if (controls->getState(Controls::STATE_JUMP))
      impulse += z;
    impulse.normalize();    

    // velocity
    if (controls->getState(Controls::STATE_RUN))
      impulse *= max_velocity;
    else
      impulse *= min_velocity;
  }

  // time
  float time = ifps;

  // target velocity
  float target_velocity = length(impulse);

  // movement
  do {
    // adaptive time step
    ifps = min(time, PLAYER_SPECTATOR_IFPS);
    time -= ifps;

    // save old velocity
    float old_velocity = length(velocity);

    // integrate velocity
    velocity += impulse * (acceleration * ifps);

    // damping
    float current_velocity = length(velocity);
    if (target_velocity < UNIGINE_EPSILON || current_velocity > target_velocity)
      velocity *= Math::exp(-damping * ifps);

    // clamp maximum velocity
    current_velocity = length(velocity);
    if (current_velocity > old_velocity) {
      if (current_velocity > target_velocity)
        velocity *= target_velocity / current_velocity;
    }

    // frozen velocity
    if (current_velocity < UNIGINE_EPSILON)
      velocity = vec3_zero;

    // integrate position
    position += Vec3(velocity * ifps);

    // world collision
    contacts.clear();
    if (getCollision()) {
      for (int i = 0; i < PLAYER_SPECTATOR_COLLISIONS; i++) {
        shape->setCenter(position);
        shape->getCollision(contacts, 0.0f);
        if (contacts.size() == 0)
          break;
        float inum_contacts = 1.0f / Math::itof(contacts.size());
        for (int j = 0; j < contacts.size(); j++) {
          const ShapeContactPtr &c = contacts[j];
          position += Vec3(c->getNormal() * (c->getDepth() * inum_contacts));
          velocity -= c->getNormal() * dot(velocity, c->getNormal());
        }
      }
    }

    // shape position
    shape->setCenter(position);
  } while (time > UNIGINE_EPSILON);

  flush_transform();
}

//////////////////////////////////////////////////////////////////////////
// Parameters
//////////////////////////////////////////////////////////////////////////

void AGSpectator::setControlled(int c) { controlled = c; }

int AGSpectator::isControlled() const { return controlled; }

void AGSpectator::setCollision(int c) { collision = c; }

int AGSpectator::getCollision() const { return collision; }

void AGSpectator::setCollisionMask(int mask) { shape->setCollisionMask(mask); }

int AGSpectator::getCollisionMask() const { return shape->getCollisionMask(); }

void AGSpectator::setCollisionRadius(float radius) { shape->setRadius(radius); }

float AGSpectator::getCollisionRadius() const { return shape->getRadius(); }

void AGSpectator::setMinVelocity(float in_velocity) { min_velocity = max(in_velocity, 0.0f); }

float AGSpectator::getMinVelocity() const { return min_velocity; }

void AGSpectator::setMaxVelocity(float in_velocity) { max_velocity = max(in_velocity, 0.0f); }

float AGSpectator::getMaxVelocity() const { return max_velocity; }

void AGSpectator::setMinThetaAngle(float angle)
{
  min_theta_angle = clamp(angle, -PLAYER_SPECTATOR_CLAMP, PLAYER_SPECTATOR_CLAMP);
}

float AGSpectator::getMinThetaAngle() const { return min_theta_angle; }

void AGSpectator::setMaxThetaAngle(float angle)
{
  max_theta_angle = clamp(angle, -PLAYER_SPECTATOR_CLAMP, PLAYER_SPECTATOR_CLAMP);
}

float AGSpectator::getMaxThetaAngle() const { return max_theta_angle; }

void AGSpectator::setAcceleration(float accel) { acceleration = max(accel, 0.0f); }

float AGSpectator::getAcceleration() const { return acceleration; }

void AGSpectator::setDamping(float d) { damping = max(d, 0.0f); }

float AGSpectator::getDamping() const { return damping; }

void AGSpectator::setTurning(float t) { turning = max(t, 0.0f); }

float AGSpectator::getTurning() const { return turning; }

void AGSpectator::read_property_parameters()
{
  setCollision(1);
  setCollisionMask(param_collision_mask);
  setCollisionRadius(param_collision_radius);
  setMinVelocity(param_min_velocity);
  setMaxVelocity(param_max_velocity);
  setAcceleration(param_acceleration);
  setDamping(param_damping);
  setTurning(param_turning);
  setMinThetaAngle(param_min_theta_angle);
  setMaxThetaAngle(param_max_theta_angle);
}

//////////////////////////////////////////////////////////////////////////
// Dynamic
//////////////////////////////////////////////////////////////////////////

void AGSpectator::setPhiAngle(float angle)
{
  angle = angle - phi_angle;
  direction = quat(camera->getUp(), angle) * direction;
  phi_angle += angle;

  flush_transform();
}

float AGSpectator::getPhiAngle() const { return phi_angle; }

void AGSpectator::setThetaAngle(float angle)
{
  angle = clamp(angle, min_theta_angle, max_theta_angle) - theta_angle;
  direction = quat(cross(camera->getUp(), direction), angle) * direction;
  theta_angle += angle;

  flush_transform();
}

float AGSpectator::getThetaAngle() const { return theta_angle; }

void AGSpectator::setViewDirection(const vec3 &view)
{
  direction = normalize(view);

  // ortho basis
  vec3 tangent, binormal;
  Geometry::orthoBasis(camera->getUp(), tangent, binormal);

  // decompose view direction
  Log::message("AGSpectator::setViewDirection()\n");
  phi_angle = Math::atan2(dot(direction, tangent), dot(direction, binormal)) * UNIGINE_RAD2DEG;
  theta_angle = Math::acos(clamp(dot(direction, camera->getUp()), -1.0f, 1.0f)) * UNIGINE_RAD2DEG - 90.0f;
  theta_angle = clamp(theta_angle, min_theta_angle, max_theta_angle);

  flush_transform();
}

const vec3 &AGSpectator::getViewDirection() const { return direction; }

//////////////////////////////////////////////////////////////////////////
// Contacts
//////////////////////////////////////////////////////////////////////////

int AGSpectator::getNumContacts() const { return contacts.size(); }

const ShapeContactPtr &AGSpectator::getContact(int num) const { return contacts[num]; }

float AGSpectator::getContactDepth(int num) const { return contacts[num]->getDepth(); }

vec3 AGSpectator::getContactNormal(int num) const { return contacts[num]->getNormal(); }

ObjectPtr AGSpectator::getContactObject(int num) { return contacts[num]->getObject(); }

Vec3 AGSpectator::getContactPoint(int num) const { return contacts[num]->getPoint(); }

ShapePtr AGSpectator::getContactShape(int num) const { return contacts[num]->getShape1(); }

int AGSpectator::getContactSurface(int num) const { return contacts[num]->getSurface(); }

//////////////////////////////////////////////////////////////////////////
// Flush
//////////////////////////////////////////////////////////////////////////

void AGSpectator::flush_transform()
{
  node->setWorldTransform(setTo(position, position + Vec3(direction), camera->getUp()));
}

//////////////////////////////////////////////////////////////////////////
// Transformation
//////////////////////////////////////////////////////////////////////////

void AGSpectator::setTransform(const Mat4 &transform)
{
  node->setTransform(transform);
  update_transform();
}

void AGSpectator::setWorldTransform(const Mat4 &transform)
{
  node->setWorldTransform(transform);
  update_transform();
}

void AGSpectator::update_transform()
{
  // update transformation
  vec3 up = camera->getUp();

  // ortho basis
  vec3 tangent, binormal;
  Geometry::orthoBasis(up, tangent, binormal);

  // decompose transformation
  position = node->getWorldTransform().getColumn3(3);
  direction = normalize(vec3(-node->getWorldTransform().getColumn3(2)));

  // decompose direction
  phi_angle = Math::atan2(dot(direction, tangent), dot(direction, binormal)) * UNIGINE_RAD2DEG;
  theta_angle = Math::acos(clamp(dot(direction, up), -1.0f, 1.0f)) * UNIGINE_RAD2DEG - 90.0f;
  theta_angle = clamp(theta_angle, min_theta_angle, max_theta_angle);
}
