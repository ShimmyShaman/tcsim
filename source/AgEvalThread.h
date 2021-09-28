#ifndef AGEVALTHREAD
#define AGEVALTHREAD

#include <UnigineGame.h>
#include <UnigineLogic.h>
#include <UnigineStreams.h>
#include <UnigineTextures.h>
#include <UnigineThread.h>
#include <UnigineViewport.h>
#include <UnigineWidgets.h>
#include <torch/script.h>  // One-stop header.

#include <opencv4/opencv2/opencv.hpp>
#include <vector>

void saveTextureToFile(Unigine::TexturePtr &texture, const char *image_path);

typedef struct _detectedTennisBall {
  double prob;
  int left, top, right, bottom;
} DetectedTennisBall;

class AgEvalThread : public Unigine::Thread {
 public:
  const int MAX_PREDICTIONS_PER_FRAME = 20;

  AgEvalThread();

  bool queueEvaluation(Unigine::TexturePtr screenshot, void *state_arg,
                       void (*callback)(void *, std::vector<DetectedTennisBall> &));

 protected:
  void process() override;

  void detect(std::vector<DetectedTennisBall> &detected);

 private:
  mutable Unigine::Mutex lock;

  bool eval_queued;
  void *eval_state_arg;
  void (*eval_callback)(void *, std::vector<DetectedTennisBall> &);

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
