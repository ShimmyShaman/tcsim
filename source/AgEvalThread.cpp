
#include "AgEvalThread.h"

#include <UnigineImage.h>
#include <UnigineStreams.h>

using namespace Unigine;

const char *const SCREENSHOT_PATH = "/home/simpson/proj/tennis_court/screenshot.jpg";
const char *const INFERENCE_RESULT_PATH = "/home/simpson/proj/tennis_court/inference_result.txt";

void saveTextureToFile(TexturePtr &texture, const char *image_path)
{
  // texture->copy2D();
  ImagePtr image = Image::create();
  texture->getImage(image);
  if (!Render::isFlipped())
    image->flipY();
  // image->getFormatName
  image->convertToFormat(Image::FORMAT_RGB8);
  // image->getPixels();

  // Save to file
  image->save(image_path);
}

AgEvalThread::AgEvalThread() { eval_queued = false; }

ImagePtr image = Image::create();
bool AgEvalThread::queueEvaluation(TexturePtr screenshot, void (*callback)(std::vector<DetectedTennisBall> &))
{
  ScopedLock atomic(lock);

  if (eval_queued) {
    return false;
  }
  // puts("screenshot_saved");
  // saveTextureToFile(screenshot, SCREENSHOT_PATH);
  screenshot->getImage(image);
  // image->load(SCREENSHOT_PATH);
  image->convertToFormat(Unigine::Image::FORMAT_RGB32F);
  // image->flipY();

  input.width = image->getWidth();
  input.height = image->getHeight();
  // printf("image format name:'%s'\n", image->getFormatName());
  // printf("numpixels:%zu\n", image->getNumPixels());
  // printf("%i %i\n", image->getWidth(), image->getHeight());
  // printf("rgba 0,0 : {%f, %f, %f}\n", image->get2D(0, 0).f.r, image->get2D(0, 0).f.g, image->get2D(0, 0).f.b);

  // cv::Mat img = cv::imread(SCREENSHOT_PATH);
  img = cv::Mat(cv::Size(image->getWidth(), image->getHeight()), CV_32FC3, image->getPixels());
  // img.convertTo(img, CV_32FC3);
  cv::resize(img, img, cv::Size(300, 300));

  // cv::imwrite("/home/simpson/proj/tennis_court/sss.tiff", img);

  // cv::divide(img, 255.f)
  cv::subtract(img, 0.5f, img);
  cv::multiply(img, 2.f, img);

  // return true;

  // unsigned char *pixels = image->getPixels();
  // uint8_t *pixu = (uint8_t *)img.data;
  // printf("pixels 0,0 : {%u, %u, %u}\n", pixu[0], pixu[1], pixu[2]);
  // printf("pixels 0,299 : {%u, %u, %u}\n", pixu[299 * 300 * 3], pixu[299 * 300 * 3 + 1], pixu[299 * 300 * 3 + 2]);
  // printf("pixels 299,0 : {%u, %u, %u}\n", pixu[299 * 3], pixu[299 * 3 + 1], pixu[299 * 3 + 2]);
  // float *pixf = (float *)img.data;
  // printf("pixels 0,0 : {%f, %f, %f}\n", pixf[0], pixf[1], pixf[2]);
  // printf("pixels 0,299 : {%f, %f, %f}\n", pixf[299 * 300 * 3], pixf[299 * 300 * 3 + 1], pixf[299 * 300 * 3 + 2]);
  // printf("pixels 299,0 : {%f, %f, %f}\n", pixf[299 * 3], pixf[299 * 3 + 1], pixf[299 * 3 + 2]);
  // //     .requires_grad(true);

  // img_blob = torch::ones({1, 3, image->getWidth(), image->getHeight()}, options);
  // img_blob = torch::from_blob(img.data, {1, 3, 300, 300}, options);

  auto options = torch::TensorOptions().dtype(torch::kFloat32).layout(torch::kStrided).device(torch::kCPU);
  auto tensor = torch::from_blob(img.data, {1, 300, 300, 3}, options).permute({0, 3, 1, 2});
  // std::cout << "tensorsizenow:" << tensor.sizes() << std::endl;
  // uint8_t means_d[3] = {127, 127, 127};
  // tensor = torch::subtract(tensor, torch::from_blob(means_d, {1, 3, 1, 1}));
  // torch::convert
  // return true;

  inputs.clear();
  inputs.push_back(tensor);

  // // puts("eval_queued");

  eval_callback = callback;
  eval_queued = true;

  return true;
}

void AgEvalThread::detect(std::vector<DetectedTennisBall> &detected)
{
  // Load the model each and every time so as to avoid the bug that prevents multiple usage of the same model
  torch::jit::script::Module mb1ssd;
  try {
    // torch::Device device = torch::kCPU;
    // if (torch::cuda::is_available()) {
    //   std::cout << "CUDA is available! Training on GPU." << std::endl;
    // torch::Device device = torch::kCUDA;
    // // }
    mb1ssd = torch::jit::load("/home/simpson/proj/tennis_court/py/ssd_voc.pt");
    mb1ssd.eval();
    // mb1ssd.to(device);
    // printf("mb1ssd train = %s\n", mb1ssd.is_training() ? "train-mode" : "eval-mode");
    // torch::Tensor tensor = torch::rand({2, 3});
    // std::cout << tensor << std::endl;

    // mb1ssd.is_training
  }
  catch (const c10::Error &e) {
    std::cerr << "Error loading the mb1-ssd model\n" << e.what() << std::endl;
    return;
  }

  // Execute the model and turn its output into a tensor
  // inputs.clear();
  // inputs.push_back(torch::ones({1, 3, 300, 300}));

  // at::Tensor sib = inputs[0].toTensor();
  // float *sibf = (float *)sib.data_ptr();
  // printf("agInputs:");
  // for (int i = 0; i < 6; ++i) printf("%f::", sibf[i]);
  // puts("");
  // at::Tensor tensor = inputs[0].toTensor();
  // printf("pixels 0,0 : {%f, %f, %f}\n", tensor[0][0][0][0].item<float>(), tensor[0][1][0][0].item<float>(),
  //        tensor[0][2][0][0].item<float>());
  // printf("pixels 299,0 : {%f, %f, %f}\n", tensor[0][0][299][0].item<float>(), tensor[0][1][299][0].item<float>(),
  //        tensor[0][2][299][0].item<float>());
  // printf("pixels 0,299 : {%f, %f, %f}\n", tensor[0][0][0][299].item<float>(), tensor[0][1][0][299].item<float>(),
  //        tensor[0][2][0][299].item<float>());
  // printf("pixels 60,230 : {%f, %f, %f}\n", tensor[0][0][60][230].item<float>(), tensor[0][1][60][230].item<float>(),
  //        tensor[0][2][60][230].item<float>());

  c10::IValue result = mb1ssd.forward(inputs);
  auto tuple = result.toTuple();
  auto scores = tuple->elements()[0].toTensor();
  auto boxes = tuple->elements()[1].toTensor();
  // std::cout << "scores:" << scores.sizes() << std::endl;
  // std::cout << "boxes:" << boxes.sizes() << std::endl;
  // for (int i = 0; i < 7; ++i) {
  //   std::cout << scores[0][i][0].item<float>() << "<>" << scores[0][i][1] << std::endl;
  // }
  // float *v = (float *)scores.data_ptr();
  // std::cout << "Results above 0.4:" << std::endl;

  // auto res = torch::greater(scores, 0.4f);
  // std::cout << res[0][0][1] << "::" << res.size(1) << std::endl;
  // auto resum = torch::sum(res, {1}, true);

  struct _pred {
    float prob;
    int idx;
  };
  std::vector<_pred> ordered_probs;

  int scores_len = scores.size(1);
  for (int i = 0; i < scores_len; ++i) {
    float f = scores[0][i][1].item<float>();
    if (f > 0.4f) {
      _pred p;
      p.prob = f;
      p.idx = i;
      ordered_probs.push_back(p);
    }
  }
  std::sort(ordered_probs.begin(), ordered_probs.end(), [](_pred a, _pred b) { return a.prob >= b.prob; });

  detected.clear();
  for (int i = 0; i < MAX_PREDICTIONS_PER_FRAME && i < ordered_probs.size(); ++i) {
    _pred p = ordered_probs[i];

    DetectedTennisBall dtb;
    dtb.prob = p.prob;
    dtb.left = (int)(boxes[0][p.idx][0].item<float>() * input.width);
    dtb.top = (int)(boxes[0][p.idx][1].item<float>() * input.height);
    dtb.right = (int)(boxes[0][p.idx][2].item<float>() * input.width);
    dtb.bottom = (int)(boxes[0][p.idx][3].item<float>() * input.height);

    detected.push_back(dtb);
  }

  // std::this_thread::sleep_for(std::chrono::milliseconds(200));
}
// {
//   char cmd[512];
//   sprintf(cmd, "python3 ~/proj/pytorch-ssd/ssd_inference.py %s %s", SCREENSHOT_PATH, INFERENCE_RESULT_PATH);
//   system(cmd);
//   puts("ah");
// }

void AgEvalThread::process()
{
  while (isRunning()) {
    lock.lock();
    if (eval_queued) {
      lock.unlock();

      // Create a vector of inputs.
      // puts("ab");

      float begin = Game::getTime();
      std::vector<DetectedTennisBall> detected;
      detect(detected);
      printf("predict() took %.4f seconds\n", Game::getTime() - begin);

      // ip0.cpu();
      // puts("ad");
      // std::cout << output->elements().size() << '\n';
      // output.

      // FilePtr inf = File::create(INFERENCE_RESULT_PATH, "r");
      // // inf->open(INFERENCE_RESULT_PATH, "r");
      // if (!inf->isOpened()) {
      //   puts("Error Opening Inference Result File!");
      // }
      // else {
      //   std::vector<DetectedTennisBall> detected;

      //   char c;
      //   while (1) {
      //     DetectedTennisBall dt;

      //     String line = inf->readLine();
      //     if (line.size() < 1)
      //       break;
      //     StringArray<256> sa = String::split(line.get(), ":,");

      //     dt.prob = String::atod(sa[0]);
      //     dt.left = String::atoi(sa[1]);
      //     dt.top = String::atoi(sa[2]);
      //     dt.right = String::atoi(sa[3]);
      //     dt.bottom = String::atoi(sa[4]);
      //     detected.push_back(dt);
      //   }
      //   inf->close();

      //   eval_callback(detected);
      // }

      lock.lock();
      eval_queued = false;
      lock.unlock();

      eval_callback(detected);

      continue;
    }

    lock.unlock();
    sleep(1);
  }
}