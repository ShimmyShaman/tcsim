#ifndef ALLIGATOR
#define ALLIGATOR

#include <UnigineComponentSystem.h>
#include <UnigineControls.h>
#include <UnigineGame.h>
#include <UnigineGeometry.h>
#include <UnigineLogic.h>
#include <UniginePlayers.h>
#include <UnigineStreams.h>
#include <UnigineViewport.h>
#include <UnigineWidgets.h>

#include <memory>
#include <mutex>
#include <vector>

#include "AgEvalThread.h"

const char *const IMAGE_PATH_FORMAT = "/media/simpson/Backup/data/tennis_court/JPEGImages/ss_%i.jpg";
const char *const ANNOTATION_PATH_FORMAT = "/media/simpson/Backup/data/tennis_court/Annotations/ss_%i.xml";

#define OCCG_SIZE 60
#define OCCG_STRIDE 0.3f
#define OCCG_OFF (OCCG_STRIDE * OCCG_SIZE / 2)

class Alligator {
 public:
  struct BallDetection {
    float x, y, dist2;
    float prob;
    struct {
      int l, r, t, b;
    } scr;
  };
  class _TrackedDetection {
   public:
    enum TargetStatus {
      None,
      Targetted,
      MarkedForRemoval,
    };

    float x, y;
    float score;
    int occ;

    // For Evaluation Purposes
    BallDetection eval_alloc;
    float eval_score;

    TargetStatus primary_target;
  };

  struct EvalState {
    Unigine::Math::Vec3 agc_t, agc_dir;
    Unigine::Math::Mat4 agc_proj;

    Unigine::TexturePtr eval_screengrab;

    std::atomic_bool eval_in_progress;
    float eval_time;
    std::vector<BallDetection> eval_detections;

    std::vector<std::shared_ptr<Alligator::_TrackedDetection>> tracked_detections;
    std::mutex td_mutex;
    struct {
      float p_obs;
    } occ_grid[OCCG_SIZE][OCCG_SIZE];
  };

  void init();
  void shutdown();

  void update();
  void captureAlligatorPOV();

 private:
  enum AutonomyMode {
    AM_Manual,
    AM_Autonomous,
    AM_AnnImg_Gen,
  };

  void handleUserInput();

  /* Annotates the currently displayed screen then saves it to file. Returns the number of objects found in the current
   * screen. */
  int annotateScreen(int capture_index, const char *const path_format = ANNOTATION_PATH_FORMAT);

  void getTennisBallScreenLocations(std::vector<cv::Rect> &output);
  void createAnnotatedSample();
  void evaluateScreenImage();

  void randomize_tennis_ball_placements(bool restrict_corner_court = false, int ball_count = -1);
  void setAutonomyMode(AutonomyMode mode);
  void processAndReissueEvaluation();
  void updateAutonomy(const float ifps, float &agql, float &agqr);
  void updateAutoAnnotation(const float ifps, float &agql, float &agqr);
  void moveToTarget(const float ifps, const float agq, Unigine::Math::Vec3 &delta, float &agql, float &agqr);

  struct EvalState eval_state;

  Unigine::PlayerPtr main_player, ag_player;
  Unigine::PlayerSpectatorPtr spectator;

  std::vector<Unigine::ObjectMeshStaticPtr> tennis_balls;
  std::vector<Unigine::ObjectMeshStaticPtr> tb_markers;
  std::vector<Unigine::ObjectMeshStaticPtr> percent_poles;

  int prev_AUX0_state = 0, prev_AUX5_state = 0, prev_AUX7_state = 0, prev_AUX8_state = 0, prev_AUX9_state = 0;

  float last_screenshot;
  Unigine::TexturePtr ag_pov_screen;  // alligator PoV
  Unigine::ViewportPtr ag_viewport;   // alligator Viewport
  Unigine::WidgetSpritePtr sprite;

  AgEvalThread *eval_thread;

  Unigine::NodePtr alligator;
  float agq;
  Unigine::Math::Vec3 agt;
  bool alligator_mode;

  Unigine::NodePtr est_alligator;
  float est_agq;
  Unigine::Math::Vec3 est_agt;
  float prev_agwp_l, prev_agwp_r;

  AutonomyMode ag_control_mode;
  enum NavState {
    NSUnassigned,
    NSTarget,
    NSPass,
    NSScanning,
    NSAutoRoute,
  };
  NavState wps;
};

#endif /* ALLIGATOR */
