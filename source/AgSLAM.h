#ifndef AGSLAM
#define AGSLAM

#include <UnigineMathLib.h>

#include <memory>

#include "g2o/core/sparse_optimizer.h"
// #include "g2o/types/SE3
#include "g2o/types/sba/vertex_se3_expmap.h"

class AgSLAMFrame;
using AgSLAMFramePtr = std::shared_ptr<AgSLAMFrame>;

class AgSLAMFrame {
 public:
  enum class FrameType {
    Origin,
    Keyframe,
    Frame,
  };

  static AgSLAMFramePtr createOrigin();
  static AgSLAMFramePtr createFrame();
  // static AgSLAMFramePtr createKeyframe(Vec3 translation = Unigine::Math::Vec3(0,0,0), Vec3 rotation = 0);

  AgSLAMFrame() {}

  // FrameType getFrameType();
  AgSLAMFramePtr getPrevious();
  AgSLAMFramePtr getNext();

  float getRotation();
  Unigine::Math::Vec3 getPosition();

  g2o::VertexSE3Expmap *vertex;

 private:
  FrameType type;
  AgSLAMFramePtr prev, next;

};

class AgSLAM {
 public:
  void initialize(Unigine::Math::Vec3 originPosition = Unigine::Math::Vec3(0, 0, 0), float originRotation = 0);

  AgSLAMFramePtr getCurrent();

  /* Appends another frame of activity to the SLAM system
     @translation the estimated translation from the previous frames update to use as a constraint
     @rotation the estimated rotation from the previous frames update to utilize as a constraint
  */
  AgSLAMFramePtr appendFrame(Unigine::Math::Vec3 translation, float rotation);

 private:
  g2o::SparseOptimizer optimizer;

  AgSLAMFramePtr current;
};

#endif /* AGSLAM */