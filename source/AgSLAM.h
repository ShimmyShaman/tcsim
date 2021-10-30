#ifndef AGSLAM
#define AGSLAM

#include <UnigineMathLib.h>

#include <memory>

class AgSLAMFrame;
using AgSLAMFramePtr = std::shared_ptr<AgSLAMFrame>;

class AgSLAMFrame {
 public:
  enum class FrameType {
    Origin,
    Keyframe,
    Frame,
  };

  static AgSLAMFramePtr createOrigin(Vec3 position, float rotation);
  // static AgSLAMFramePtr createFrame(Vec3 translation = Unigine::Math::Vec3(0,0,0), Vec3 rotation = 0);
  // static AgSLAMFramePtr createKeyframe(Vec3 translation = Unigine::Math::Vec3(0,0,0), Vec3 rotation = 0);

  // FrameType getFrameType();
  AgSLAMFramePtr getPrevious();
  AgSLAMFramePtr getNext();

  float getRotation();
  Unigine::Math::Vec3 getPosition();

 private:
  FrameType type;
  Unigine::Math::Vec3 position;
  float rotation;
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
  AgSLAMFrame appendFrame(Unigine::Math::Vec3 translation, float rotation);

 private:
  AgSLAMFramePtr current;
};

#endif /* AGSLAM */
