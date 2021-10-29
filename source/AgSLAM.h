#ifndef AGSLAM
#define AGSLAM

#include <UnigineMathLib.h>

class AgSLAM {
 public:
  void initialize();

  float getEstimateRotation();
  Unigine::Math::Vec3 getEstimatePosition();

 private:
};

#endif /* AGSLAM */
