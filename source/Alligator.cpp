#include "Alligator.h"

#include <UnigineApp.h>
#include <UniginePlayers.h>

#include <atomic>
#include <fstream>
#include <iostream>
#include <optional>

using namespace Unigine;
using namespace Unigine::Math;

NodePtr markerPole, mark;
static int auto_wp_idx = 0;
static Vec3 auto_wps[] = {
    Vec3(-8.9f, -6.8f, 0.f),  Vec3(-7.6f, -5.1f, 0.f),   Vec3(-12.8f, -5.0f, 0.f), Vec3(-10.4f, 3.1f, 0.f),
    Vec3(-6.2f, 1.4f, 0.f),   Vec3(-1.8f, 0.2f, 0.f),    Vec3(-1.9f, -7.2f, 0.f),  Vec3(13.5f, -9.8f, 0.f),
    Vec3(6.2f, -17.1f, 0.f),  Vec3(0.4f, 1.9f, 0.f),     Vec3(3.2f, -2.1f, 0.f),   Vec3(1.3f, 8.6f, 0.f),
    Vec3(-8.6, 5.1f, 0.f),    Vec3(-9.1f, 8.2f, 0.f),    Vec3(-15.9f, 9.1f, 0.f),  Vec3(-16.0f, -3.4f, 0.f),
    Vec3(-7.2f, -14.0f, 0.f), Vec3(-11.2f, -16.2f, 0.f),
};
static int auto_wps_count = sizeof(auto_wps) / sizeof(Vec3);

void printMatrix(const char *name, mat4 mat)
{
  int len = strlen(name);

  printf("%s: %.2f %.2f %.2f %.2f\n", name, mat[0], mat[1], mat[2], mat[3]);
  int i = len;
  for (i = 0; i < len + 2; ++i) printf(" ");
  printf("%.2f %.2f %.2f %.2f\n", mat[4], mat[5], mat[6], mat[7]);
  for (i = 0; i < len + 2; ++i) printf(" ");
  printf("%.2f %.2f %.2f %.2f\n", mat[8], mat[9], mat[10], mat[11]);
  for (i = 0; i < len + 2; ++i) printf(" ");
  printf("%.2f %.2f %.2f %.2f\n", mat[12], mat[13], mat[14], mat[15]);
}

inline void wrapAngle(float &a)
{
  while (a >= 180.f) a -= 360.f;
  while (a < -180.f) a += 360.f;
}

void Alligator::init()
{
  last_screenshot = -4.75f;

  ControlsApp::setStateKey(Controls::STATE_AUX_0, (int)'t');  // Camera Toggle
  ControlsApp::setStateKey(Controls::STATE_AUX_1, (int)'u');
  ControlsApp::setStateKey(Controls::STATE_AUX_2, (int)'o');
  ControlsApp::setStateKey(Controls::STATE_AUX_3, (int)'j');
  ControlsApp::setStateKey(Controls::STATE_AUX_4, (int)'l');
  ControlsApp::setStateKey(Controls::STATE_AUX_5, (int)'y');      // Autonomous Toggle
  ControlsApp::setStateKey(Controls::STATE_AUX_6, App::KEY_F4);   // Elegant Exit
  ControlsApp::setStateKey(Controls::STATE_AUX_7, App::KEY_F10);  // Screenshot
  ControlsApp::setStateKey(Controls::STATE_AUX_8, App::KEY_F7);   // Annotated Image Collection Mode

  // main_player = ag_player = Game::getPlayer();
  // printf("ag_player=%p\n", World::getNodeByName("ag_camera"));
  ag_player = PlayerPtr(static_cast<Player *>(World::getNodeByName("ag_camera").get()));
  // printf("ag_player=%p\n", World::getNodeByName("ag_camera"));
  main_player = spectator =
      PlayerSpectatorPtr(static_cast<PlayerSpectator *>(World::getNodeByName("player_spectator").get()));
  alligator_mode = false;

  alligator = World::getNodeByName("alligator");
  // ags = alligator->getScale();
  agq = alligator->getRotation().getAngle(vec3_up);
  agt = alligator->getPosition();

  // Alligator PoV
  ag_viewport = Viewport::create();
  screenshot = Texture::create();
  GuiPtr gui = Gui::get();
  sprite = WidgetSprite::create(gui);
  gui->addChild(sprite, Gui::ALIGN_OVERLAP | Gui::ALIGN_BACKGROUND);
  sprite->setPosition(0, 0);
  sprite->setRender(screenshot, !Render::isFlipped());
  screenshot->create2D(300, 300, Texture::FORMAT_RGB8, Texture::FILTER_POINT | Texture::USAGE_RENDER);

  // printf("dir: %.1f, %.1f, %.1f\n", alligator->getDirection().x, alligator->getDirection().y,
  //        alligator->getDirection().z);
  // alligator->setDirection(Vec3_right, Vec3_up);

  // printf("dir: %.1f, %.1f, %.1f\n", alligator->getDirection().x, alligator->getDirection().y,
  //        alligator->getDirection().z);

  NodePtr tb_root = World::getNodeByName("TennisBalls");
  for (int a = 0; a < tb_root->getNumChildren(); ++a) {
    ObjectMeshStaticPtr tb_ptr = ObjectMeshStaticPtr(static_cast<ObjectMeshStatic *>(tb_root->getChild(a).get()));

    tennis_balls.push_back(tb_ptr);
  }
  randomize_tennis_ball_placements(16);

  // Obtain Tennis Ball Marker Meshes
  NodePtr tbm_root = World::getNodeByName("Markers")->findNode("TBMarkers", 0);
  if (!tbm_root)
    puts("ERROR - Can't Find Markers/TBMarkers");
  for (int a = 0; a < tbm_root->getNumChildren(); ++a) {
    ObjectMeshStaticPtr tbm_ptr = ObjectMeshStaticPtr(static_cast<ObjectMeshStatic *>(tbm_root->getChild(a).get()));

    tb_markers.push_back(tbm_ptr);
  }
  printf("tb_markers.size():%zu\n", tb_markers.size());

  // -- Store Data
  // eval_state.agc_t = ag_player->getCamera()->getPosition();
  // eval_state.agc_ivp, ag_player->getCamera()->getProjection(),
  //     lookAt(eval_state.agc_t, ag_player->getViewDirection(), Vec3_up));
  // inverse4(eval_state.agc_ivp, eval_state.agc_ivp);

  eval_state.eval_in_progress = false;
  eval_thread = new AgEvalThread();
  eval_thread->run();

  memset(eval_state.occ_grid, 0.f, sizeof(eval_state.occ_grid));

  markerPole = World::getNodeByName("MarkerPole");
  mark = World::getNodeByName("Mark");
  markerPole->setPosition(vec3(-10.9f, -4.5f, 0.f));
  wps = NSUnassigned;
  ag_control_mode = AM_Manual;
}

void Alligator::shutdown()
{
  // Write here code to be called on world shutdown: delete resources that were created during world script execution
  // to avoid memory leaks.

  eval_thread->stop();
  delete eval_thread;
}

void Alligator::update()
{
  float ifps = Game::getIFps();

  if (main_player->getControls()->getState(Controls::STATE_AUX_6)) {
    App::exit();
  }
  if (main_player->getControls()->getState(Controls::STATE_AUX_7) != prev_AUX7_state) {
    if (prev_AUX7_state) {
      // Released
      prev_AUX7_state = 0;

      // Change Cameras
      if (screenshot) {
        saveTextureToFile(screenshot, "/home/simpson/proj/tennis_court/screenshot.jpg");
        puts("Screenshot saved to 'screenshot.jpg'");
      }
    }
    else {
      // Pressed
      prev_AUX7_state = 1;
    }
  }
  if (main_player->getControls()->getState(Controls::STATE_AUX_8) != prev_AUX8_state) {
    if (prev_AUX8_state) {
      // Released
      prev_AUX8_state = 0;

      // Set auto-navigation mode
      setAutonomyMode(AM_AnnImg_Gen);
    }
    else {
      // Pressed
      prev_AUX8_state = 1;
    }
  }
  if (main_player->getControls()->getState(Controls::STATE_AUX_0) != prev_AUX0_state) {
    if (prev_AUX0_state) {
      // Released
      prev_AUX0_state = 0;

      // Change Cameras
      if (main_player == ag_player) {
        main_player = spectator;

        // spectator->flushTransform TODO
        ag_player->setEnabled(false);
        spectator->setEnabled(true);

        alligator_mode = false;
      }
      else {
        main_player = ag_player;

        spectator->setEnabled(false);
        ag_player->setEnabled(true);

        alligator_mode = true;
      }
      Game::setPlayer(main_player);
    }
    else {
      // Pressed
      prev_AUX0_state = 1;
    }
  }
  if (main_player->getControls()->getState(Controls::STATE_AUX_5) != prev_AUX5_state) {
    if (prev_AUX5_state) {
      // Released
      prev_AUX5_state = 0;

      // Change Autonomous Mode
      setAutonomyMode(ag_control_mode == AM_Autonomous ? AM_Manual : AM_Autonomous);
    }
    else {
      // Pressed
      prev_AUX5_state = 1;
    }
  }

  // In meters wheel movement
  float agql = 0, agqr = 0;
  const float UserSpeed = 0.538f;
  if (main_player->getControls()->getState(Controls::STATE_AUX_1)) {
    agql = ifps * UserSpeed;
    // while (agql < 0) agql += 360.f;
    // while (agql > 360.f) agql -= 360.f;
  }
  if (main_player->getControls()->getState(Controls::STATE_AUX_2)) {
    agqr = ifps * UserSpeed;
    // while (agqr < 0) agqr += 360.f;
    // while (agqr >= 360.f) agqr -= 360.f;
  }
  if (main_player->getControls()->getState(Controls::STATE_AUX_3)) {
    agql = -ifps * UserSpeed;
    // while (agql < 0) agql += 360.f;
    // while (agql > 360.f) agql -= 360.f;
  }
  if (main_player->getControls()->getState(Controls::STATE_AUX_4)) {
    agqr = -ifps * UserSpeed;
    // while (agqr < 0) agqr += 360.f;
    // while (agqr >= 360.f) agqr -= 360.f;
  }

  if (ag_control_mode != AM_Manual) {
    if (agql || agqr)
      setAutonomyMode(AM_Manual);
    else {
      switch (ag_control_mode) {
        case AM_Autonomous:
          updateAutonomy(ifps, agql, agqr);
          break;
        case AM_AnnImg_Gen:
          updateAutoAnnotation(ifps, agql, agqr);
          break;
      }
    }
  }

  if (agql || agqr) {
    // 46 cm seperate the back wheels from each other.
    const float wheel_seperation = 0.4685f;
    // When they both rotate (dual rotation) in opposite directions the rotation circle is half
    // that of when one wheel rotates around the other stationary wheel
    const float dual_rotation_circumference = 0.5f * 2.f * Math::Consts::PI * wheel_seperation;
    const float single_rotation_circumference = 2.f * Math::Consts::PI * wheel_seperation;

    Mat4 tsfm;
    float amt;

    // Dual Wheel Motion
    if (agql > 0) {
      if (agqr > 0) {
        // Move Forward
        if (agql > agqr) {
          amt = agqr;
          agql -= amt;
          agqr = 0.f;
        }
        else {
          amt = agql;
          agqr -= amt;
          agql = 0.f;
        }

        // Move the vehicle forward by amt metres
        Vec3 t(0.f, amt, 0.f);
        mul(t, rotate(Vec3_up, agq), t);
        agt += t;
      }
      else if (agqr < 0) {
        // Rotate clockwise around the centre the complement amount
        if (agql > -agqr) {
          amt = -agqr;
          agql -= amt;
          agqr = 0.f;
        }
        else {
          amt = agql;
          agqr += amt;
          agql = 0.f;
        }

        // Obtain the rotation in degrees
        amt = 360.f * amt / dual_rotation_circumference;
        agq -= amt;
      }
    }
    else if (agql < 0) {
      if (agqr < 0) {
        // Move in Reverse
        if (agql < agqr) {
          amt = -agqr;
          agql += amt;
          agqr = 0.f;
        }
        else {
          amt = -agql;
          agqr += amt;
          agql = 0.f;
        }

        // Move the vehicle backwards by amt metres
        Vec3 t(0.f, -amt, 0.f);
        mul(t, rotate(Vec3_up, agq), t);
        agt += t;
      }
      else if (agqr > 0) {
        // Rotate counter-clockwise around the centre the complement amount
        if (-agql > agqr) {
          amt = agqr;
          agql += amt;
          agqr = 0.f;
        }
        else {
          amt = -agql;
          agqr -= amt;
          agql = 0.f;
        }

        // Obtain the rotation in degrees
        amt = 360.f * amt / dual_rotation_circumference;
        agq += amt;
      }
    }

    // Single Wheel Motion
    if (agql) {
      // -- Rotate around the right wheel
      // Find the transformation matrix to set the right wheel to the origin
      Vec3 off(-0.5f * wheel_seperation, 0.f, 0.f);
      mul(off, rotate(Vec3_up, agq), off);

      // Rotate
      amt = -360.f * agql / single_rotation_circumference;
      mul(tsfm, rotate(Vec3_up, amt), translate(off));

      // Translate back
      mul(tsfm, translate(-off), tsfm);

      mul(off, tsfm, Vec3_zero);
      add(agt, off, agt);
      agq += amt;
    }
    else if (agqr) {
      // -- Rotate around the left wheel
      // Find the transformation matrix to set the left wheel to the origin
      Vec3 off(0.5f * wheel_seperation, 0.f, 0.f);
      mul(off, rotate(Vec3_up, agq), off);

      // Rotate
      amt = 360.f * agqr / single_rotation_circumference;
      mul(tsfm, rotate(Vec3_up, amt), translate(off));

      // Translate back
      mul(tsfm, translate(-off), tsfm);

      mul(off, tsfm, Vec3_zero);
      add(agt, off, agt);
      agq += amt;
    }

    wrapAngle(agq);
    mul(tsfm, translate(agt), rotate(Vec3_up, agq));
    // printf("agt=[%.2f %.2f 0.f] agq=%.2f\n", agt.x, agt.y, agq);
    alligator->setTransform(tsfm);
  }
}

const char *const IMAGE_PATH_FORMAT = "/home/simpson/data/tennis_court/JPEGImages/ss_%i.jpg";
const char *const ANNOTATION_PATH_FORMAT = "/home/simpson/data/tennis_court/Annotations/ss_%i.xml";

int Alligator::annotateScreen(int capture_index)
{
  // quat cameraAngle = ag_player->getWorldRotation();
  // printf("Camera: %.2f\n", ag_player->getWorldRotation().getAngle(Vec3_up));
  vec3 tangent = ag_player->getWorldRotation().getTangent();
  mul(tangent, tangent, 0.034f);
  // float tangentAngle = getAngle(ag_player)
  // printf("Camera Tangent: %.2f %.2f %.2f\n", tangent.x, tangent.y, tangent.z);

  struct TBBB {
    int x0, y0, x1, y1;
  };
  std::vector<struct TBBB> found;

  vec3 pos, tp;
  for (auto tb : tennis_balls) {
    struct TBBB bb;

    pos = tb->getPosition();
    sub(tp, pos, tangent);
    tp.z += 0.035f;
    if (!ag_player->getScreenPosition(bb.x0, bb.y0, tp))
      continue;
    add(tp, pos, tangent);
    tp.z -= 0.035f;
    if (!ag_player->getScreenPosition(bb.x1, bb.y1, tp))
      continue;

    if (bb.x1 <= 4 || bb.x0 >= App::getWidth() - 4 || bb.y1 <= 4 || bb.y0 >= App::getHeight() - 4)
      continue;

    found.push_back(bb);
    // printf("tennisball-: [%i, %i, %i, %i]\n", x0, y0, x1 - x0, y1 - y0);
  }

  if (!found.size())
    return 0;

  // Write the annotation file
  char fp[256];
  sprintf(fp, ANNOTATION_PATH_FORMAT, capture_index);

  std::ofstream f(fp);
  if (!f.is_open()) {
    puts("Error Opening Annotation File for Writing!!");
    return 0;
  }

  f << "<annotation>" << std::endl;
  f << "  <folder>captures</folder>" << std::endl;
  f << "  <filename>ss_" << capture_index << ".jpg</filename>" << std::endl;
  char buf[256];
  sprintf(buf, IMAGE_PATH_FORMAT, capture_index);
  f << "  <path>" << buf << "</path>" << std::endl;
  f << "  <source>" << std::endl;
  f << "    <database>SimTennisCourtBalls</database>" << std::endl;
  f << "  </source>" << std::endl;
  f << "  <size>" << std::endl;
  f << "    <width>" << App::getWidth() << "</width>" << std::endl;
  f << "    <height>" << App::getHeight() << "</height>" << std::endl;
  f << "    <depth>3</depth>" << std::endl;
  f << "  </size>" << std::endl;
  f << "  <segmented>0</segmented>" << std::endl;
  for (auto tb : found) {
    f << "  <object>" << std::endl;
    f << "    <name>TennisBall</name>" << std::endl;
    f << "    <pose>Unspecified</pose>" << std::endl;
    f << "    <truncated>0</truncated>" << std::endl;
    f << "    <difficult>0</difficult>" << std::endl;
    f << "    <bndbox>" << std::endl;
    f << "      <xmin>" << tb.x0 << "</xmin>" << std::endl;
    f << "      <ymin>" << tb.y0 << "</ymin>" << std::endl;
    f << "      <xmax>" << tb.x1 << "</xmax>" << std::endl;
    f << "      <ymax>" << tb.y1 << "</ymax>" << std::endl;
    f << "    </bndbox>" << std::endl;
    f << "  </object>" << std::endl;
  }
  f << "</annotation>" << std::endl;
  f.close();
}

void Alligator::createAnnotatedSample()
{
  static Vec3 last_cam_position;
  static quat last_cam_rotation;
  if (ag_player->getWorldPosition() == last_cam_position && ag_player->getWorldRotation() == last_cam_rotation)
    return;
  last_cam_position = ag_player->getWorldPosition();
  last_cam_rotation = ag_player->getWorldRotation();

  static int capture_index = 0;
  if (capture_index % 50 == 0) {
    randomize_tennis_ball_placements();
  }

  // Annotate the screen (if any annotations exist, return if not)
  if (annotateScreen(capture_index) == 0)
    return;

  // Save to file
  char image_path[256];
  sprintf(image_path, IMAGE_PATH_FORMAT, capture_index);
  saveTextureToFile(screenshot, image_path);
  Log::message("annotated image sample(%i) created\n", capture_index);

  ++capture_index;
}

void Alligator::randomize_tennis_ball_placements(int ball_count)
{
  static Random rng;
  rng.setSeed(1000);

  if (ball_count < 0) {
    ball_count = rng.getInt(2, 28);
    if (ball_count > 14)
      ball_count -= rng.getInt(0, 7);
  }
  printf("Repositioned %i balls (randomly)\n", ball_count);

  for (auto tb : tennis_balls) {
    --ball_count;
    if (ball_count < 0) {
      tb->setPosition(Vec3(0.f, 0.f, 10000.f));
      continue;
    }

    tb->setPosition(Vec3(rng.getFloat(-0.5f, -16.f), rng.getFloat(-7.f, 7.f), 0.034f));
    tb->setRotation(quat(rng.getDir(), rng.getFloat(0.f, 360.f)));
  }
  if (ball_count > 0) {
    puts("--Too many ball placements requested for algorithm -- need more tennis ball assets or something");
  }
}

void Alligator::captureAlligatorPOV()
{
  // static Vec3 last_cam_position;
  // static quat last_cam_rotation;
  // if (main_player != ag_player ||
  //     ag_player->getWorldPosition() == last_cam_position && ag_player->getWorldRotation() == last_cam_rotation)
  //   return;
  // last_cam_position = ag_player->getWorldPosition();
  // last_cam_rotation = ag_player->getWorldRotation();

  // Render
  // ag_viewport->appendSkipFlags(Viewport::SKIP_POSTEFFECTS);

  // saving current render state and clearing it
  RenderState::saveState();
  RenderState::clearStates();

  // enabling polygon front mode to correct camera flipping
  // RenderState::setPolygonFront(1);

  // Check for resize
  static int appWidth = 0, appHeight = 0;
  if (App::getWidth() != appWidth || screenshot->getHeight() != appHeight) {
    // screenshot->create2D(300, 300, Texture::FORMAT_RGB8, Texture::FILTER_POINT | Texture::USAGE_RENDER);
    appWidth = App::getWidth();
    appHeight = App::getHeight();

    float scale = MIN(0.42f * appWidth, 0.42f * appHeight);

    // adjust sprite size
    sprite->setWidth((int)scale);
    sprite->setHeight((int)scale);
    sprite->arrange();
  }

  // rendering and image from the camera of the current mirror to the texture
  ag_viewport->renderTexture2D(ag_player->getCamera(), screenshot);

  // restoring back render stateRS
  RenderState::setPolygonFront(0);
  RenderState::restoreState();
}

void Alligator::setAutonomyMode(Alligator::AutonomyMode mode)
{
  ag_control_mode = mode;

  switch (ag_control_mode) {
    case AM_Manual:
      break;
    case AM_Autonomous:
      wps = NSUnassigned;
      break;
    case AM_AnnImg_Gen: {
      // Find the nearest waypoint and set to wps_index
      float ldist = 130000.f;
      auto_wp_idx = 0;
      for (int i = 0; i < auto_wps_count; ++i) {
        vec3 delta = auto_wps[i] - agt;
        float dist2 = length2(delta);

        if (dist2 < ldist) {
          ldist = dist2;
          auto_wp_idx = i;
        }
      }

      // Increment wp-index to move to the next target
      auto_wp_idx = (auto_wp_idx + 1) % auto_wps_count;
      wps = NSAutoRoute;
    } break;
  }
}

void evaluationCallback(void *state, std::vector<DetectedTennisBall> &result)
{
  Alligator::EvalState &es = *(Alligator::EvalState *)state;
  // printf("evaluationCallback() detected_count=%lu\n", result.size());

  // -- Invert the proj/view matrix
  mat4 ivp;
  mul(ivp, es.agc_proj, es.agc_view);
  ivp = inverse(ivp);
  std::queue<Alligator::BallDetection> detections;
  es.eval_detections.clear();

  for (auto b : result) {
    // printf("Ball:(%i%%) [%i %i %i %i]\n", (int)(b.prob * 100), b.left, b.top, b.right, b.bottom);
    // Unproject the ball ground contact point

    // Obtain the world position
    float x = 0.5f * (b.left + b.right), y = b.top;
    vec4 near((float)(x / es.img_size.x - 0.5f) * 2.f, (float)(y / es.img_size.y - 0.5f) * 2.f, -1.f, 1.f);
    vec4 far((float)(x / es.img_size.x - 0.5f) * 2.f, (float)(y / es.img_size.y - 0.5f) * 2.f, 1.f, 1.f);

    mul(near, ivp, near);
    mul(far, ivp, far);

    near.x /= near.w;
    near.y /= near.w;
    near.z /= near.w;
    far.x /= far.w;
    far.y /= far.w;
    far.z /= far.w;
    vec3 dir = normalize(far.xyz - near.xyz);

    vec3 pred(es.agc_t.x + dir.x * es.agc_t.z / -dir.z, es.agc_t.y + dir.y * es.agc_t.z / -dir.z, 0.f);

    Alligator::BallDetection dt;
    dt.x = pred.x;
    dt.y = pred.y;
    dt.prob = b.prob;
    detections.push(dt);

    es.eval_detections.push_back(dt);

    // eval_pred = pred;
    // printf("dir: %.2f %.2f %.2f\n", dir.x, dir.y, dir.z);
    // printf("prd: %.2f %.2f 0\n", pred.x, pred.y);
    // printf("mrk: %.2f %.2f 0\n", mark->getPosition().x, mark->getPosition().y);
    // markerPole->setPosition(pred);
  }

  std::lock_guard<std::mutex> lg(es.td_mutex);

  // Find the best existing tdi to attach the reading to
  // --or create another one
  while (detections.size()) {
    auto b = detections.front();
    detections.pop();

    // printf("detection: %.2f %.2f %.2f\n", b.x, b.y, b.prob);

    auto bat = es.tracked_detections.end();
    float bs = 0.f;
    for (auto tdi = es.tracked_detections.begin(); tdi != es.tracked_detections.end(); ++tdi) {
      float score = ((b.x - tdi->x) * (b.x - tdi->x) + (b.y - tdi->y) * (b.y - tdi->y));

      if (score > 0.8f || (bs != 0.f && score > bs))
        continue;

      if (tdi->eval_score != 0.f) {
        if (score < tdi->eval_score) {
          // Better match than previous tie
          bat = tdi;
          bs = score;
        }
        continue;
      }

      if (bat != es.tracked_detections.end()) {
        bat->eval_score = 0.f;
      }

      // Better match than previous tie
      bat = tdi;
      bs = score;
    }

    // Set
    if (bs != 0.f) {
      if (bat->eval_score != 0.f) {
        detections.push(bat->eval_alloc);
      }
      bat->eval_alloc = b;
      bat->eval_score = bs;
    }
    else {
      // Construct another node
      Alligator::TrackedDetection ap;
      ap.x = 0;
      ap.y = 0;
      ap.occ = 0;
      ap.score = 0.5f;  // Initial boost
      ap.eval_score = 0.00001f;

      ap.eval_alloc = b;

      es.tracked_detections.push_back(ap);
    }
  }

  // Decay all occupancies in the view frustrum
  WorldBoundFrustum bf(es.agc_proj, es.agc_view);

  float decay_rate = 0.003f * Math::pow((float)es.tracked_detections.size() / 15, 3.5f);

  // Integrate any set eval scores
  for (auto tdi = es.tracked_detections.begin(); tdi != es.tracked_detections.end();) {
    // Decay score
    float decay = 0.00001f + tdi->score * decay_rate * MAX(1, 7 - tdi->occ);
    if (tdi->score - decay > 10000.f) {
      printf("MASSIVE: decay:%.2f prev_score:%.2f\n", decay, tdi->score);
      App::exit();
    }
    tdi->score -= decay;

    if (tdi->eval_score == 0.f) {
      if (bf.insideFast(Vec3(tdi->x, tdi->y, 0.f))) {
        // Inside view frustrum and no attached detection
        // Reduce score
        // printf("reducing [%.2f %.2f] to %.2f\n", tdi->x, tdi->y, tdi->score);
        tdi->score -= 5 * decay;
      }

      if (tdi->score <= 0) {
        printf("removing [%.2f %.2f]\n", tdi->x, tdi->y);
        tdi = es.tracked_detections.erase(tdi);
        continue;
      }

      tdi++;
      continue;
    }

    // printf("eval_alloc: %.2f %.2f %.2f\n", tdi->eval_alloc.x, tdi->eval_alloc.y, tdi->eval_alloc.prob);

    float mod = 1.f;
    if (tdi->occ)
      mod = 1.f / tdi->occ;
    ++tdi->occ;
    tdi->x = tdi->x * (1.f - mod) + tdi->eval_alloc.x * mod;
    tdi->y = tdi->y * (1.f - mod) + tdi->eval_alloc.y * mod;
    tdi->score = tdi->score * (1.f - mod) + tdi->eval_alloc.prob * mod + MAX(0.4f, tdi->occ * 0.05f);

    // printf("modified [%.2f %.2f] to %.2f\n", tdi->x, tdi->y, tdi->score);

    // Reset
    tdi->eval_score = 0.f;
    tdi++;
  }

  // Toggle
  es.eval_in_progress = false;
}

void Alligator::moveToTarget(float ifps, Vec3 &delta, float &agql, float &agqr)
{
  // Continue along prescribed path
  float theta = getAngle(Vec3_forward, delta, Vec3_up);

  // -- Move to the target
  // Reduce the angle
  float angularDiff = theta - agq;
  wrapAngle(angularDiff);
  // float turnForce = Unigine::Math::abs(angularDiff);
  // float qlm, qrm;

  const float MaxSpeed = 0.700f;
  agql = ifps * MaxSpeed;
  agqr = ifps * MaxSpeed;
  if (angularDiff > 1.0f) {
    if (angularDiff > 90.f) {
      agql *= -(angularDiff - 90.f) / 90.f;
    }
    else {
      agql *= 1.f - angularDiff / 90.f;
    }
  }
  else if (angularDiff < 1.0f) {
    if (angularDiff < -90.f) {
      agqr *= (angularDiff + 90.f) / 90.f;
    }
    else {
      agqr *= 1.f + angularDiff / 90.f;
    }
  }

  prev_agql = agql / ifps;
  prev_agqr = agqr / ifps;
  // printf("agql=%.4f agqr=%.4f\n", agql, agqr);

  // vec2 d = waypoint - agt.xy;
  // float th = 180.f + acosf32((-1.f * d.y) / d.length()) * 180.f / M_PI;

  // printf("t=%.2f a=%.2f diff=%.2f\n", theta, agq, angularDiff);
  // float prev_l = 0.f, prev_r = 0.f;
}

void Alligator::updateAutoAnnotation(float ifps, float &agql, float &agqr)
{
  last_screenshot += ifps;
  if (last_screenshot >= 0.45f) {
    last_screenshot = 0.f;

    createAnnotatedSample();
  }

  static vec3 wp = auto_wps[0];
  vec3 delta = wp - agt;
  float dist2 = length2(delta);
  if (dist2 < 0.3f) {
    auto_wp_idx = (auto_wp_idx + 1) % auto_wps_count;
    wp = auto_wps[auto_wp_idx];

    // Update delta
    delta = wp - agt;
  }
  moveToTarget(ifps, delta, agql, agqr);
}

void Alligator::updateAutonomy(float ifps, float &agql, float &agqr)
{
  if (!eval_state.eval_in_progress) {
    // saveTextureToFile(screenshot, "/home/simpson/proj/tennis_court/screenshot.jpg");
    // Integrate the results of the previous evaluation

    // Update the render positions of detected ball markers
    int a = 0;
    for (; a < eval_state.eval_detections.size() && a < tb_markers.size(); ++a) {
      ObjectMeshStaticPtr tbm = tb_markers[a];
      Alligator::BallDetection bd = eval_state.eval_detections[a];
      tbm->setEnabled(true);
      tbm->setPosition(Vec3(bd.x, bd.y, 0.4f));
    }
    printf("showing %i ball detections\n", a);
    for (; a < tb_markers.size(); ++a) {
      tb_markers[a]->setEnabled(false);
    }

    // -- Queue another evaluation
    if (eval_thread->queueEvaluation(screenshot, this, evaluationCallback)) {
      eval_state.eval_in_progress = true;

      // -- Store Data
      eval_state.agc_t = ag_player->getCamera()->getPosition();
      eval_state.img_size = vec2(screenshot->getWidth(), screenshot->getHeight());

      // printf("eval_agc_t: %.2f %.2f %.2f\n", eval_agc_t.x, eval_agc_t.y, eval_agc_t.z);
      // printf("view-dir: %.2f %.2f %.2f\n", ag_player->getViewDirection().x, ag_player->getViewDirection().y,
      //        ag_player->getViewDirection().z);
      eval_state.agc_proj = ag_player->getAspectCorrectedProjection();
      eval_state.agc_view = lookAt(eval_state.agc_t, eval_state.agc_t + ag_player->getViewDirection(), Vec3_up);
      eval_state.eval_time = Game::getTime();
    }

    // Formulate a new planned route
    // float hp = 0.f;
    // int hx = 0, hy = 0, hx2 = 0, hy2 = 0;
    // for (int ix = 0; ix < OCCG_SIZE; ++ix)
    //   for (int iy = 0; iy < OCCG_SIZE; ++iy) {
    //     if (eval_state.occ_grid[ix][iy].p_ball > hp) {
    //       hx = ix;
    //       hy = iy;
    //       hp = eval_state.occ_grid[ix][iy].p_ball;
    //     }
    //   }
    // markerPole->setPosition(vec3(0.3f * (hx - 30), 0.3f * (hy - 30), 0.f));
    // route_len = 1;
    // route[0] = 0.3f * (hx - 30);
    // route[1] = 0.3f * (hy - 30);
  }

  // puts("here")

  static vec2 wp;
  static float wps_time;
  if (wps == NSUnassigned) {
    printf("Before [%zu tracked detections]\n", eval_state.tracked_detections.size());
    std::lock_guard<std::mutex> lg(eval_state.td_mutex);
    if (eval_state.tracked_detections.size()) {
      std::sort(eval_state.tracked_detections.begin(), eval_state.tracked_detections.end(),
                [](const Alligator::TrackedDetection &a, const Alligator::TrackedDetection &b) -> bool {
                  return a.score > b.score;
                });

      printf("[%zu tracked detections]\n", eval_state.tracked_detections.size());
      for (int i = 0; i < 5 && i < eval_state.tracked_detections.size(); ++i) {
        // for (auto td : eval_state.tracked_detections) {
        auto td = eval_state.tracked_detections[i];
        printf("--%.2f [%.2f %.2f] (%i)\n", td.score, td.x, td.y, td.occ);
      }

      for (std::vector<Alligator::TrackedDetection>::iterator target_i = eval_state.tracked_detections.begin();
           target_i != eval_state.tracked_detections.end();) {
        wp.x = target_i->x;
        wp.y = target_i->y;

        vec3 delta = vec3(wp, 0) - agt;
        if (delta.length() < 1.f) {
          ++target_i;
          continue;
        }

        wps = NSTarget;
        printf("moving to [%.2f %.2f] score=%.2f(%i)\n", wp.x, wp.y, target_i->score, target_i->occ);
        markerPole->setPosition(Vec3(wp.x, wp.y, 0.f));

        target_i = eval_state.tracked_detections.erase(target_i);
        break;
      }
    }

    if (!wps) {
      puts("scanning...");
      wps = NSScanning;
      wps_time = Game::getTime();
    }
  }
  // if (!wp.length2()) {
  //   wp = wps.front();
  //   wps.erase(wps.begin());
  // }

  vec3 delta = vec3(wp, 0) - agt;
  float dist2 = length2(delta);
  switch (wps) {
    case NSTarget:
      if (dist2 < 0.3f) {
        // if (wps == 2) {
        //   // Finish
        //   wps = 0;
        //   return;
        // }

        wps = NSPass;
        wps_time = Game::getTime();
      }
      else
        moveToTarget(ifps, delta, agql, agqr);
      break;
    case NSPass: {
      if (Game::getTime() > wps_time + 3.5f) {
        wps = NSUnassigned;
      }
      else {
        // Just continue on
        agql = ifps * prev_agql;
        agqr = ifps * prev_agqr;
      }
    }
    case NSScanning: {
      agql = ifps * 0.25f;
      agqr = ifps * -0.25f;
      if (Game::getTime() > wps_time + 6.5f) {
        wps = NSUnassigned;
      }
    } break;
  }
}