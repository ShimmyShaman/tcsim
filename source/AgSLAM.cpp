#include "AgSLAM.h"

using namespace Unigine::Math;

Vec3 AgSLAMFrame::getPosition() { return position; }

float AgSLAMFrame::getRotation() { return rotation; }

void AgSLAM::initialize(Vec3 originPosition, float originRotation)
{
  current = AgSLAMFrame::createOrigin(originPosition, originRotation);
}

// void AgSLAM::getCurrent