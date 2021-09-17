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

#include <vector>

void saveTextureToFile(Unigine::TexturePtr &texture, const char *image_path);

typedef struct _detectedTennisBall {
  double prob;
  int left, top, right, bottom;
} DetectedTennisBall;

class AgEvalThread : public Unigine::Thread {
 public:
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
  torch::jit::script::Module mb1ssd;
  std::vector<torch::jit::IValue> inputs;
};

#endif /* AGEVALTHREAD */
