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

#include <vector>

#include "AgEvalThread.h"

#define OCCG_SIZE 60
#define OCCG_STRIDE 0.3f
#define OCCG_OFF (OCCG_STRIDE * OCCG_SIZE / 2)

class Alligator {
 public:
  struct EvalState {
    Unigine::Math::Vec3 agc_t;
    Unigine::Math::Mat4 agc_proj, agc_view;
    Unigine::Math::Vec2 img_size;
    std::atomic_bool eval_in_progress;
    float occ_grid[OCCG_SIZE][OCCG_SIZE];
  };

  void init();
  void shutdown();

  void update();
  void captureAlligatorPOV();

 private:
  struct EvalState eval_state;

  /* Annotates the currently displayed screen then saves it to file. Returns the number of objects found in the current
   * screen. */
  int annotateScreen(int capture_index);

  void createAnnotatedSample();
  void evaluateScreenImage();

  void randomize_tennis_ball_placements(int ball_count = -1);
  void setAutonomyMode(bool autonomy);
  void updateAutonomy(float ifps, float &agql, float &agqr);

  Unigine::PlayerPtr main_player, ag_player;
  Unigine::PlayerSpectatorPtr spectator;

  std::vector<Unigine::ObjectMeshStaticPtr> tennis_balls;

  int prev_AUX0_state = 0, prev_AUX5_state = 0;
  bool auton_control = false;

  float last_screenshot;
  Unigine::TexturePtr screenshot;    // alligator PoV
  Unigine::ViewportPtr ag_viewport;  // alligator Viewport
  Unigine::WidgetSpritePtr sprite;

  AgEvalThread *eval_thread;

  Unigine::NodePtr alligator;
  float agq;
  Unigine::Math::Vec3 agt;
  bool alligator_mode;
};

#endif /* ALLIGATOR */
