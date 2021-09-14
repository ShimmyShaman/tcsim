/* Copyright (C) 2005-2021, UNIGINE. All rights reserved.
 *
 * This file is a part of the UNIGINE 2 SDK.
 *
 * Your use and / or redistribution of this software in source and / or
 * binary form, with or without modification, is subject to: (i) your
 * ongoing acceptance of and compliance with the terms and conditions of
 * the UNIGINE License Agreement; and (ii) your inclusion of this notice
 * in any version of this software that you use or redistribute.
 * A copy of the UNIGINE License Agreement is available by contacting
 * UNIGINE. at http://unigine.com/
 */

#ifndef APPWORLDLOGIC
#define APPWORLDLOGIC

#include <UnigineGame.h>
#include <UnigineLogic.h>
#include <UnigineStreams.h>
#include <UnigineThread.h>
#include <UnigineViewport.h>
#include <UnigineWidgets.h>

#include <vector>

using namespace Unigine;

typedef struct _detectedTennisBall {
} DetectedTennisBall;

class AgEvalThread : public Unigine::Thread {
 public:
  AgEvalThread() : eval_queued(false) {}

  bool queueEvaluation(TexturePtr screenshot, void (*callback)(std::vector<DetectedTennisBall> &));

 protected:
  void process() override;

 private:
  mutable Mutex lock;

  bool eval_queued;
  void (*eval_callback)(std::vector<DetectedTennisBall> &);
};

class AppWorldLogic : public Unigine::WorldLogic {
 public:
  AppWorldLogic();
  virtual ~AppWorldLogic();

  int init() override;

  int update() override;
  int postUpdate() override;
  int updatePhysics() override;

  int shutdown() override;

  int save(const Unigine::StreamPtr &stream) override;
  int restore(const Unigine::StreamPtr &stream) override;

  void createAnnotatedSample();
  void evaluateScreenImage();

 private:
  /* Annotates the currently displayed screen then saves it to file. Returns the number of objects found in the current
   * screen. */
  int annotateScreen(int capture_index);
  void randomize_tennis_ball_placements();
  void captureAlligatorPOV();
  void updateAutonomy(float ifps, float &agql, float &agqr);

  int initCamera();

  PlayerPtr main_player, ag_player;
  PlayerSpectatorPtr spectator;

  std::vector<ObjectMeshStaticPtr> tennis_balls;

  int prev_AUX0_state = 0, prev_AUX5_state = 0;
  bool auton_control = false;

  float last_screenshot;
  Unigine::TexturePtr screenshot;    // alligator PoV
  Unigine::ViewportPtr ag_viewport;  // alligator Viewport
  Unigine::WidgetSpritePtr sprite;

  AgEvalThread *eval_thread;
};

#endif /* APPWORLDLOGIC */
