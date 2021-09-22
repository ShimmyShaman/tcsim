#ifndef AGEVALTHREAD
#define AGEVALTHREAD

#include <UnigineGame.h>
#include <UnigineLogic.h>
#include <UnigineStreams.h>
#include <UnigineTextures.h>
#include <UnigineThread.h>
#include <UnigineViewport.h>
#include <UnigineWidgets.h>

#include <vector>

#include <torch/script.h>  // One-stop header.

#include <opencv4/opencv2/opencv.hpp>

void saveTextureToFile(Unigine::TexturePtr &texture, const char *image_path);

typedef struct _detectedTennisBall {
  double prob;
  int left, top, right, bottom;
} DetectedTennisBall;

class AgEvalThread : public Unigine::Thread {
 public:
  const int MAX_PREDICTIONS_PER_FRAME = 20;

  AgEvalThread();

  bool queueEvaluation(Unigine::TexturePtr screenshot, void (*callback)(std::vector<DetectedTennisBall> &));

 protected:
  void process() override;

  void predict();

 private:
  mutable Unigine::Mutex lock;

  bool eval_queued;
  void (*eval_callback)(std::vector<DetectedTennisBall> &);

  // torch::NoGradGuard no_grad;  // TODO check if removing this helps memory
  // torch::jit::script::Module mb1ssd;
  cv::Mat img;
  at::Tensor img_blob;
  std::vector<torch::jit::IValue> inputs;
  struct {
    int width, height;
  } input;
};

#endif /* AGEVALTHREAD */
