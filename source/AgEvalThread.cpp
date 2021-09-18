
#include "AgEvalThread.h"

#include <UnigineImage.h>
#include <UnigineStreams.h>

#include <opencv4/opencv2/opencv.hpp>

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

AgEvalThread::AgEvalThread()
{
  try {
    // torch::Device device = torch::kCPU;
    // if (torch::cuda::is_available()) {
    //   std::cout << "CUDA is available! Training on GPU." << std::endl;
    // torch::Device device = torch::kCUDA;
    // // }
    mb1ssd = torch::jit::load("/home/simpson/proj/tennis_court/py/ssd_voc.pt");
    // mb1ssd.to(device);
    printf("mb1ssd train = %s\n", mb1ssd.is_training() ? "train-mode" : "eval-mode");
    // torch::Tensor tensor = torch::rand({2, 3});
    // std::cout << tensor << std::endl;

    // mb1ssd.is_training
  }
  catch (const c10::Error &e) {
    std::cerr << "Error loading the mb1-ssd model\n" << e.what() << std::endl;
  }

  eval_queued = false;
}

ImagePtr image = Image::create();
bool AgEvalThread::queueEvaluation(TexturePtr screenshot, void (*callback)(std::vector<DetectedTennisBall> &))
{
  ScopedLock atomic(lock);

  if (eval_queued) {
    return false;
  }
  // puts("screenshot_saved");
  // saveTextureToFile(screenshot, SCREENSHOT_PATH);
  // screenshot->getImage(image);
  image->load(SCREENSHOT_PATH);
  // image->convertToFormat(Unigine::Image::FORMAT_RGB32F);
  image->flipY();

  printf("image format name:'%s'\n", image->getFormatName());
  printf("numpixels:%zu\n", image->getNumPixels());
  printf("%i %i\n", image->getWidth(), image->getHeight());
  // printf("rgba 0,0 : {%f, %f, %f}\n", image->get2D(0, 0).f.r, image->get2D(0, 0).f.g, image->get2D(0, 0).f.b);

  cv::Mat img = cv::Mat(cv::Size(image->getWidth(), image->getHeight()), CV_8UC3, image->getPixels());
  cv::resize(img, img, TODO)
  return true;

  unsigned char *pixels = image->getPixels();
  float *pixf = (float *)pixels;
  // printf("pixels 0,0 : {%f, %f, %f}\n", pixf[0], pixf[1], pixf[2]);
  // printf("pixels 0,508 : {%f, %f, %f}\n", pixf[508 * image->getWidth() * 3], pixf[508 * image->getWidth() * 3 + 1],
  //        pixf[508 * image->getWidth() * 3 + 2]);
  // printf("pixels 958,0 : {%f, %f, %f}\n", pixf[958 * 3], pixf[958 * 3 + 1], pixf[958 * 3 + 2]);

  auto options = torch::TensorOptions().dtype(torch::kFloat32).layout(torch::kStrided).device(torch::kCPU);
  // //     .requires_grad(true);

  // img_blob = torch::ones({1, 3, image->getWidth(), image->getHeight()}, options);
  img_blob = torch::from_blob(pixels, {1, 3, image->getWidth(), image->getHeight()}, options);
  // torch::sub
  inputs.clear();
  inputs.push_back(img_blob.as_strided({1, 3, 300, 300}, {0, 1, 3, 3 * image->getWidth()}, 0));

  puts("eval_queued");

  eval_callback = callback;
  eval_queued = true;

  return true;
}

void AgEvalThread::predict()
{
  // Execute the model and turn its output into a tensor.
  puts("av");
  {
    // inputs.clear();
    // inputs.push_back(torch::ones({1, 3, 300, 300}));

    at::Tensor sib = inputs[0].toTensor();
    float *sibf = (float *)sib.data_ptr();
    printf("agInputs:");
    for (int i = 0; i < 6; ++i) printf("%f::", sibf[i]);
    puts("");

    c10::IValue result = mb1ssd.forward(inputs);
    auto tuple = result.toTuple();
    std::cout << "tuple:" << tuple << std::endl;
    auto scores = tuple->elements()[0];
    float *v = (float *)scores.toTensor().data_ptr();
    std::cout << "First 15 of result1:";
    for (int i = 0; i < 15; ++i) {
      std::cout << v[i] << "<>";
    }
    std::cout << std::endl;
    // std::cout << std::endl << scores << std::endl;

    // v.cpu().detach().numpy_T
    puts("ac");
  }
  {
    char cmd[512];
    sprintf(cmd, "python3 ~/proj/pytorch-ssd/ssd_inference.py %s %s", SCREENSHOT_PATH, INFERENCE_RESULT_PATH);
    system(cmd);
    puts("ah");
  }
  // {
  //   auto options = torch::TensorOptions()
  //                      // .dtype(torch::kInt8)
  //                      .dtype(torch::kFloat32)
  //                      .layout(torch::kStrided)
  //                      .device(torch::kCPU);

  //   float *f = (float *)malloc(sizeof(float) * 3 * 300 * 300);
  //   for (int a = 0; a < 3 * 300 * 300; ++a) f[a] = 1.f;
  //   auto t = torch::from_blob(f, {1, 3, 300, 300}, options);

  //   inputs.clear();
  //   inputs.push_back(t);

  //   c10::IValue result = mb1ssd.forward(inputs);
  //   auto tuple = result.toTuple();
  //   auto scores = tuple->elements()[0];
  //   float *v = (float *)scores.toTensor().data_ptr();
  //   std::cout << "First 15 of result2:";
  //   for (int i = 0; i < 15; ++i) {
  //     std::cout << v[i] << "<>";
  //   }
  //   std::cout << std::endl;
  //   // std::cout << std::endl << scores << std::endl;

  //   // v.cpu().detach().numpy_T
  //   free(f);
  //   puts("am");
  // }

  // def predict(self, image, top_k=-1, prob_threshold=None):
  //     cpu_device = torch.device("cpu")
  //     height, width, _ = image.shape
  //     print(f"predict():\n--height, width, _ = {height}, {width}, {_}")
  //     print("image=" + str(image.shape))
  //     image = self.transform(image)
  //     print(f"self.transform(image)=" + str(image.shape))
  //     images = image.unsqueeze(0)
  //     print("image.unsqueeze(0) = " + str(images.shape))
  //     images = images.to(self.device)
  //     print(f"images.to(self.device) = " + str(images.shape))
  //     with torch.no_grad():
  //         self.timer.start()
  //         scores, boxes = self.net.forward(images)
  //         print("scores, boxes = self.net.forward(images) = " + str(scores.shape) + ',' + str(boxes.shape))
  //         print("Inference time: ", self.timer.end())
  //     boxes = boxes[0]
  //     print("boxes = boxes[0] = " + str(boxes.shape))
  //     scores = scores[0]
  //     print("scores = scores[0] = " + str(scores.shape))
  //     if not prob_threshold:
  //         prob_threshold = self.filter_threshold
  //     print("prob_threshold = " + str(prob_threshold))
  //     # this version of nms is slower on GPU, so we move data to CPU.
  //     boxes = boxes.to(cpu_device)
  //     scores = scores.to(cpu_device)
  //     picked_box_probs = []
  //     picked_labels = []
  //     print("scores.size(1) = " + str(scores.size(1)))
  //     for class_index in range(1, scores.size(1)):
  //         probs = scores[:, class_index]
  //         mask = probs > prob_threshold
  //         probs = probs[mask]
  //         if probs.size(0) == 0:
  //             continue
  //         subset_boxes = boxes[mask, :]
  //         box_probs = torch.cat([subset_boxes, probs.reshape(-1, 1)], dim=1)
  //         box_probs = box_utils.nms(box_probs, self.nms_method,
  //                                   score_threshold=prob_threshold,
  //                                   iou_threshold=self.iou_threshold,
  //                                   sigma=self.sigma,
  //                                   top_k=top_k,
  //                                   candidate_size=self.candidate_size)
  //         picked_box_probs.append(box_probs)
  //         picked_labels.extend([class_index] * box_probs.size(0))
  //     if not picked_box_probs:
  //         return torch.tensor([]), torch.tensor([]), torch.tensor([])
  //     picked_box_probs = torch.cat(picked_box_probs)
  //     picked_box_probs[:, 0] *= width
  //     picked_box_probs[:, 1] *= height
  //     picked_box_probs[:, 2] *= width
  //     picked_box_probs[:, 3] *= height
  //     return picked_box_probs[:, :4], torch.tensor(picked_labels), picked_box_probs[:, 4]
}

void AgEvalThread::process()
{
  while (isRunning()) {
    lock.lock();
    if (eval_queued) {
      lock.unlock();

      // Create a vector of inputs.
      puts("ab");

      predict();

      // ip0.cpu();
      puts("ad");
      // std::cout << output->elements().size() << '\n';
      // output.

      FilePtr inf = File::create(INFERENCE_RESULT_PATH, "r");
      // inf->open(INFERENCE_RESULT_PATH, "r");
      if (!inf->isOpened()) {
        puts("Error Opening Inference Result File!");
      }
      else {
        std::vector<DetectedTennisBall> detected;

        char c;
        while (1) {
          DetectedTennisBall dt;

          String line = inf->readLine();
          if (line.size() < 1)
            break;
          StringArray<256> sa = String::split(line.get(), ":,");

          dt.prob = String::atod(sa[0]);
          dt.left = String::atoi(sa[1]);
          dt.top = String::atoi(sa[2]);
          dt.right = String::atoi(sa[3]);
          dt.bottom = String::atoi(sa[4]);
          detected.push_back(dt);
        }
        inf->close();

        eval_callback(detected);
      }

      lock.lock();
      eval_queued = false;
      lock.unlock();
      continue;
    }

    lock.unlock();
    sleep(1);
  }
}