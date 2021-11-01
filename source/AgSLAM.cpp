#include "AgSLAM.h"

#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

using namespace Unigine::Math;

AgSLAMFramePtr AgSLAMFrame::createOrigin()
{
  // Create
  AgSLAMFramePtr origin = std::make_shared<AgSLAMFrame>();
  origin->type = AgSLAMFrame::FrameType::Origin;
}

Vec3 AgSLAMFrame::getPosition()
{
  return Vec3_zero;
  // Vec3(vertex->getEstimateData() _estimate.translation().x, vertex->_estimate.translation().y,
  // vertex->_estimate.translation().z); TODO
}

float AgSLAMFrame::getRotation()
{
  return 0;  // TODO
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
  current = AgSLAMFrame::createOrigin(originPosition, originRotation);

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

  current->vertex = new g2o::VertexSE3Expmap();
  current->vertex->setId(0);
  current->vertex->setFixed(true);
  g2o::SE3Quat se3quat;
  se3quat.setRotation(g2o::Quaternion(Eigen::AngleAxisd((double)originRotation, Eigen::Vector3d::UnitZ())));
  se3quat.setTranslation(Eigen::Vector3d((double)originPosition.x, (double)originPosition.y, (double)originPosition.z));
  current->vertex->setEstimate(se3quat);

  optimizer.addVertex(current->vertex);
}

// void AgSLAM::getCurrent