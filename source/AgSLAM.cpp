#include "AgSLAM.h"

using namespace Unigine::Math;

AgSLAMFramePtr AgSLAMFrame::createOrigin(Vec3 originPosition, float originRotation)
{
  // Create
  AgSLAMFramePtr origin = std::make_shared<AgSLAMFrame>();
  origin->type = AgSLAMFrame::FrameType::Origin;

  origin->position = originPosition;
  origin->rotation = originRotation;
}

Vec3 AgSLAMFrame::getPosition() { return position; }

float AgSLAMFrame::getRotation() { return rotation; }

void AgSLAM::initialize(Vec3 originPosition, float originRotation)
{
  current = AgSLAMFrame::createOrigin(originPosition, originRotation);
}

AgSLAMFramePtr AgSLAM::getCurrent() { return current; }

AgSLAMFramePtr AgSLAM::appendFrame(Vec3 translation, float rotation) {}

// void AgSLAM::getCurrent