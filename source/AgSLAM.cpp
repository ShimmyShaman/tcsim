#include "AgSLAM.h"

#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

using namespace Unigine::Math;

AgSLAMFramePtr AgSLAMFrame::createOrigin()
{
  // Create
  AgSLAMFramePtr fptr = std::make_shared<AgSLAMFrame>();
  fptr->type = AgSLAMFrame::FrameType::Origin;
}

AgSLAMFramePtr AgSLAMFrame::createFrame()
{
  // Create
  AgSLAMFramePtr fptr = std::make_shared<AgSLAMFrame>();
  fptr->type = AgSLAMFrame::FrameType::Frame;
}

Vec3 AgSLAMFrame::getPosition()
{
  g2o::SE3Quat estimate;
  vertex->getEstimateData((number_t*)&estimate);
  return Vec3((float)estimate.translation().x(), (float)estimate.translation().y(), (float)estimate.translation().z());
}

float AgSLAMFrame::getRotation()
{
  g2o::SE3Quat estimate;
  vertex->getEstimateData((number_t*)&estimate);
  Eigen::AngleAxisd aa(estimate.rotation());

  May just have to operate in quaternions
  return 0.f;
}

void AgSLAM::initialize(Vec3 originPosition, float originRotation)
{
  // Set Optimizer
  optimizer.setVerbose(true);

  std::string solverName = "lm_fix6_3";
  // solverName = "lm_dense6_3";
  // solverName = "lm_fix6_3_cholmod";
  g2o::OptimizationAlgorithmProperty solverProperty;
  optimizer.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct(solverName, solverProperty));

  // g2o::BlockSolver_6_3::LinearSolverType* linearSolver;
  // linearSolver = new g2o::LinearSolverDense<g2o ::BlockSolver_6_3::PoseMatrixType>();
  // // linearSolver = new g2o::LinearSolverCholmod<g2o ::BlockSolver_6_3::PoseMatrixType>();

  // g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
  // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  // optimizer.setAlgorithm(solver);

  // Set Camera Params
  double focal_length = 1000.;
  Eigen::Vector2d principal_point(150., 150.);
  g2o::CameraParameters* cam_params = new g2o::CameraParameters(focal_length, principal_point, 0.);
  cam_params->setId(0);
  assert(optimizer.addParameter(cam_params));

  // Create the origin frame
  current = AgSLAMFrame::createOrigin();

  current->vertex = new g2o::VertexSE3Expmap();
  current->vertex->setId(0);
  current->vertex->setFixed(true);
  g2o::SE3Quat se3quat;
  se3quat.setRotation(g2o::Quaternion(Eigen::AngleAxisd((double)originRotation, Eigen::Vector3d::UnitZ())));
  se3quat.setTranslation(Eigen::Vector3d((double)originPosition.x, (double)originPosition.y, (double)originPosition.z));
  current->vertex->setEstimate(se3quat);

  optimizer.addVertex(current->vertex);
}

AgSLAMFramePtr AgSLAM::getCurrent() { return current; }

AgSLAMFramePtr AgSLAM::appendFrame(Vec3 translation_delta, float rotation_delta)
{
  // Create the frame
  AgSLAMFramePtr next = AgSLAMFrame::createFrame();

  // Obtain transformation data structs
  g2o::SE3Quat delta, estimate;
  g2o::Quaternion quat_delta =
      g2o::Quaternion(Eigen::AngleAxisd((double)rotation_delta, Eigen::Vector3d::UnitZ())).normalized();
  delta.setRotation(quat_delta);
  delta.setTranslation(
      Eigen::Vector3d((double)translation_delta.x, (double)translation_delta.y, (double)translation_delta.z));
  current->vertex->getEstimateData((number_t*)&estimate);

  estimate.setRotation(estimate.rotation() * quat_delta);
  estimate.setTranslation(estimate.translation() + delta.translation());

  // Add the new vertex
  next->vertex = new g2o::VertexSE3Expmap();
  next->vertex->setId(0);
  next->vertex->setFixed(true);
  next->vertex->setEstimate(estimate);

  optimizer.addVertex(next->vertex);

  // // Add the odometry edge from the previous vertex
  // g2o::EdgeSE3Expmap* edge = new g2o::EdgeSE3Expmap();
  // edge->setVertex(0, current->vertex);
  // edge->setVertex(1, next->vertex);
  // edge->setMeasurement();
  // edge->information() = Eigen::Matrix2d::Identity();
  // edge->setParameterId(0, 0);

  // optimizer.addEdge

  // Increment Frames
  current->next = next;
  next->prev = current;
  current = next;
}

void AgSLAM::submitFrameImage(AgSLAMFramePtr frame, cv::Mat& img)
{
  // Initialize ORB detector
  static cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  static cv::InputArray mask = cv::noArray();
  // static cv::Mat prev_img = cv::Mat::zeros(cv::Size(300, 300), CV_8UC3);

  // find the keypoints and descriptors with ORB
  detector->detectAndCompute(img, mask, frame->orb_keypoints, frame->orb_descriptors);
  if (!frame->prev)
    return;

  // Search for matches amongst previous frames
  static cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create();
  static std::vector<cv::DMatch> matches;
  matches.clear();

  matcher->match(frame->prev->orb_descriptors, frame->orb_descriptors, matches);
  printf("%zu matches\n", matches.size());

  // // TODO Temp
  // static Vec3 prev_actual_agt;
  // static float prev_actual_agq;
  // {
  //   prev_actual_agt = agt;
  //   prev_actual_agq = agq;
  // }

  // if (matches.size()) {
  //   std::sort(matches.begin(), matches.end(), [](auto a, auto b) -> bool { return a.distance <= b.distance; });

  //   static int aaa = 0;
  //   ++aaa;
  //   if (aaa == 114) {
  //     cv::Mat final_img;
  //     cv::drawMatches(prev_img, prev_keypoints, img, keypoints, matches, final_img);
  //     // // cv::resize(final_img, final_img, cv::Size(600, 300));

  //     printf("matches=%zu\n", matches.size());
  //     for (size_t mi = 0; mi < matches.size(); ++mi) {
  //       printf("--dist=%.3f [%.2f %.2f]\n", matches[mi].distance,
  //              keypoints[matches[mi].trainIdx].pt.x - prev_keypoints[matches[mi].queryIdx].pt.x,
  //              keypoints[matches[mi].trainIdx].pt.y - prev_keypoints[matches[mi].queryIdx].pt.y);
  //     }

  //     cv::imwrite("/home/simpson/proj/tennis_court/ss/orb_111.jpg", final_img);
  //     puts("wrote orb_111.jpg");

  //     // Do Bundle Adjustment
  //   }
  // }

  // // Move to previous image
  // prev_keypoints.swap(keypoints);
  // descriptors.copyTo(prev_descriptors);
  // std::swap(img, prev_img);
}