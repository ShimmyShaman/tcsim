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

#include <mutex>
#include <vector>

#include "AgEvalThread.h"

#define OCCG_SIZE 60
#define OCCG_STRIDE 0.3f
#define OCCG_OFF (OCCG_STRIDE * OCCG_SIZE / 2)

class Alligator {
 public:
  struct BallDetection {
    float x, y;
    float prob;
  };
  struct TrackedDetection {
    float x, y;
    float score;
    int occ;

    // For Evaluation Purposes
    BallDetection eval_alloc;
    float eval_score;
  };
  struct EvalState {
    Unigine::Math::Vec3 agc_t;
    Unigine::Math::Mat4 agc_proj, agc_view;
    Unigine::Math::Vec2 img_size;
    std::atomic_bool eval_in_progress;
    float eval_time;
    std::vector<BallDetection> eval_detections;

    std::vector<TrackedDetection> tracked_detections;
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

  /* Annotates the currently displayed screen then saves it to file. Returns the number of objects found in the current
   * screen. */
  int annotateScreen(int capture_index);

  void createAnnotatedSample();
  void evaluateScreenImage();

  void randomize_tennis_ball_placements(int ball_count = -1);
  void setAutonomyMode(AutonomyMode mode);
  void updateAutonomy(float ifps, float &agql, float &agqr);
  void updateAutoAnnotation(float ifps, float &agql, float &agqr);
  void moveToTarget(float ifps, Unigine::Math::Vec3 &delta, float &agql, float &agqr);

  struct EvalState eval_state;

  Unigine::PlayerPtr main_player, ag_player;
  Unigine::PlayerSpectatorPtr spectator;

  std::vector<Unigine::ObjectMeshStaticPtr> tennis_balls;
  std::vector<Unigine::ObjectMeshStaticPtr> tb_markers;

  int prev_AUX0_state = 0, prev_AUX5_state = 0, prev_AUX7_state = 0, prev_AUX8_state = 0;

  float last_screenshot;
  Unigine::TexturePtr screenshot;    // alligator PoV
  Unigine::ViewportPtr ag_viewport;  // alligator Viewport
  Unigine::WidgetSpritePtr sprite;

  AgEvalThread *eval_thread;

  Unigine::NodePtr alligator;
  float agq;
  Unigine::Math::Vec3 agt;
  bool alligator_mode;

  AutonomyMode ag_control_mode;
  enum NavState {
    NSUnassigned,
    NSTarget,
    NSPass,
    NSScanning,
    NSAutoRoute,
  };
  float prev_agql, prev_agqr;
  NavState wps;
};

#endif /* ALLIGATOR */
