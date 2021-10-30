#include "Alligator.h"

#include <UnigineApp.h>
#include <UniginePlayers.h>
#include <UnigineVisualizer.h>

#include <atomic>
#include <fstream>
#include <iostream>
#include <optional>

#include "AgUtilities.h"

#define USE_ESTIMATE_MODEL
// Default speed of the vehicle (700 mm/sec)
#define DEFAULT_SPEED 0.700f

#define SCREENSHOT_PATH "/home/simpson/proj/tennis_court/screenshot.jpg"

using namespace Unigine;
using namespace Unigine::Math;

#define uabs Unigine::Math::abs
using TrackedDetection = std::shared_ptr<Alligator::_TrackedDetection>;
using TargetStatus = Alligator::_TrackedDetection::TargetStatus;

NodePtr markerPole, mark;
static int auto_wp_idx = 0;
static Vec3 auto_wps[] = {
    Vec3(-8.9f, -6.8f, 0.f),  Vec3(-7.6f, -5.1f, 0.f),   Vec3(-12.8f, -5.0f, 0.f),  Vec3(-10.4f, 3.1f, 0.f),
    Vec3(-6.2f, 1.4f, 0.f),   Vec3(-1.8f, 0.2f, 0.f),    Vec3(-1.9f, -7.2f, 0.f),   Vec3(13.5f, -9.8f, 0.f),
    Vec3(6.2f, -17.1f, 0.f),  Vec3(0.4f, 1.9f, 0.f),     Vec3(3.2f, -2.1f, 0.f),    Vec3(1.3f, 8.6f, 0.f),
    Vec3(-8.6, 5.1f, 0.f),    Vec3(-9.1f, 8.2f, 0.f),    Vec3(-15.9f, 9.1f, 0.f),   Vec3(-16.0f, -3.4f, 0.f),
    Vec3(-7.2f, -14.0f, 0.f), Vec3(-11.2f, -16.2f, 0.f), Vec3(-15.0f, -15.0f, 0.f), Vec3(-1.8f, -9.8f, 0.f),
    Vec3(-9.3f, -19.1f, 0.f), Vec3(-1.3f, -23.0f, 0.f),  Vec3(-14.0f, -16.0f, 0.f), Vec3(1.7f, -16.7f, 0.f),
    Vec3(10.3f, -11.5f, 0.f), Vec3(-5.9f, -5.2f, 0.f),   Vec3(-12.8f, -8.8f, 0.f),  Vec3(-11.2f, 7.8f, 0.f),
    Vec3(-2.59f, 2.19f, 0.f), Vec3(-7.2f, 7.8f, 0.f),    Vec3(10.2f, 7.8f, 0.f),    Vec3(14.5f, -5.8f, 0.f),
    Vec3(14.4f, 1.4f, 0.f),   Vec3(4.2f, -9.2f, 0.f),    Vec3(-15.f, 0.f, 0.f),     Vec3(-12.8f, -7.f, 0.f),
    Vec3(-13.f, 5.f, 0.f),    Vec3(-1.9f, 4.7f, 0.f),    Vec3(-11.f, -1.8f, 0.f),   Vec3(-15.3f, -2.3f, 0.f),
};
static int auto_wps_count = sizeof(auto_wps) / sizeof(Vec3);
static Random rng;

const int SampleWidth = 300, SampleHeight = 300;

void Alligator::init()
{
  last_screenshot = -4.75f;
  rng.setSeed(1002);

  ControlsApp::setStateKey(Controls::STATE_AUX_0, (int)'t');  // Camera Toggle
  ControlsApp::setStateKey(Controls::STATE_AUX_1, (int)'u');
  ControlsApp::setStateKey(Controls::STATE_AUX_2, (int)'o');
  ControlsApp::setStateKey(Controls::STATE_AUX_3, (int)'j');
  ControlsApp::setStateKey(Controls::STATE_AUX_4, (int)'l');
  ControlsApp::setStateKey(Controls::STATE_AUX_5, (int)'y');      // Autonomous Toggle
  ControlsApp::setStateKey(Controls::STATE_AUX_6, App::KEY_F4);   // Elegant Exit
  ControlsApp::setStateKey(Controls::STATE_AUX_7, App::KEY_F10);  // Screenshot
  ControlsApp::setStateKey(Controls::STATE_AUX_8, App::KEY_F7);   // Annotated Image Collection Mode
  ControlsApp::setStateKey(Controls::STATE_AUX_9, App::KEY_F6);   // Annotated Current Perspective

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

  // Camera Estimate
  est_alligator = World::getNodeByName("est_model");
#ifdef USE_ESTIMATE_MODEL
  est_alligator->setRotation(alligator->getRotation());
  est_alligator->setPosition(agt);

  est_offset_agq = agq;
  est_offset_agt = agt;
  ag_cam_offset = ag_player->getPosition();
  ag_slam.initialize(Vec3(0, 0, 0.2f), 0.f);

  est_trav_l = est_trav_r = 0.f;
  prev_est_trav_l = prev_est_trav_r = 0.f;
#else
  est_alligator->setEnabled(false);
#endif

  // Alligator PoV
  ag_viewport = Viewport::create();
  ag_pov_screen = Texture::create();
  GuiPtr gui = Gui::get();
  sprite = WidgetSprite::create(gui);
  gui->addChild(sprite, Gui::ALIGN_OVERLAP | Gui::ALIGN_BACKGROUND);
  sprite->setPosition(0, 0);
  sprite->setRender(ag_pov_screen, !Render::isFlipped());
  ag_pov_screen->create2D(SampleWidth, SampleHeight, Texture::FORMAT_RGB8,
                          Texture::FILTER_POINT | Texture::USAGE_RENDER);

  fpsLabel = WidgetLabel::create(gui, "??");
  fpsLabel->setFontSize(14);
  fpsLabel->setPosition(8, 8);
  fpsLabel->setFontColor(vec4_black);
  gui->addChild(fpsLabel, Gui::ALIGN_RIGHT | Gui::ALIGN_TOP);
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
  placeRandomTennisBalls(true, true, 16);

  // Obtain Tennis Ball Marker Meshes
  NodePtr tbm_root = World::getNodeByName("Markers")->findNode("TBMarkers", 0);
  if (!tbm_root)
    puts("ERROR - Can't Find Markers/TBMarkers");
  for (int a = 0; a < tbm_root->getNumChildren(); ++a) {
    ObjectMeshStaticPtr tbm_ptr = ObjectMeshStaticPtr(static_cast<ObjectMeshStatic *>(tbm_root->getChild(a).get()));

    tb_markers.push_back(tbm_ptr);
  }
  printf("tb_markers.size():%zu\n", tb_markers.size());

  // Obtain Tennis Ball Marker Meshes
  NodePtr pp_root = World::getNodeByName("Markers")->findNode("PercentPoles", 0);
  if (!pp_root)
    puts("ERROR - Can't Find Markers/PercentPoles");
  for (int a = 0; a < pp_root->getNumChildren(); ++a) {
    ObjectMeshStaticPtr pp_ptr = ObjectMeshStaticPtr(static_cast<ObjectMeshStatic *>(pp_root->getChild(a).get()));

    percent_poles.push_back(pp_ptr);
  }
  printf("percent_poles.size():%zu\n", percent_poles.size());

  // -- Store Data
  // eval_state.agc_t = ag_player->getCamera()->getPosition();
  // eval_state.agc_ivp, ag_player->getCamera()->getProjection(),
  //     lookAt(eval_state.agc_t, ag_player->getViewDirection(), Vec3_up));
  // inverse4(eval_state.agc_ivp, eval_state.agc_ivp);

  eval_state.eval_in_progress = false;
  eval_state.eval_screengrab = Texture::create();
  eval_state.eval_screengrab->create2D(SampleWidth, SampleHeight, Texture::FORMAT_RGB8, Texture::FILTER_POINT);
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

void transformAlligator(float agql, float agqr, Vec3 &agt, float &agq, Mat4 &tsfm)
{
  // 46 cm seperate the back wheels from each other.
  const float wheel_seperation = 0.4685f;
  // When they both rotate (dual rotation) in opposite directions the rotation circle is half
  // that of when one wheel rotates around the other stationary wheel
  const float dual_rotation_circumference = 0.5f * 2.f * Math::Consts::PI * wheel_seperation;
  const float single_rotation_circumference = 2.f * Math::Consts::PI * wheel_seperation;

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
}

void Alligator::handleUserInput()
{
  if (main_player->getControls()->getState(Controls::STATE_AUX_6)) {
    App::exit();
  }
  if (main_player->getControls()->getState(Controls::STATE_AUX_7) != prev_AUX7_state) {
    if (prev_AUX7_state) {
      // Released
      prev_AUX7_state = 0;

      // Take Screenshot
      if (ag_pov_screen) {
        saveTextureToFile(ag_pov_screen, SCREENSHOT_PATH);
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
      setAutonomyMode(ag_control_mode == AM_AnnImg_Gen ? AM_Manual : AM_AnnImg_Gen);
      placeRandomTennisBalls(true, false, 0);
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
      setAutonomyMode(ag_control_mode == AM_Manual ? AM_Autonomous : AM_Manual);
    }
    else {
      // Pressed
      prev_AUX5_state = 1;
    }
  }
  if (main_player->getControls()->getState(Controls::STATE_AUX_9) != prev_AUX9_state) {
    if (prev_AUX9_state) {
      // Released
      prev_AUX9_state = 0;

      static int annotation_index = 0;

      static ImagePtr uimg = Image::create();
      ag_pov_screen->getImage(uimg);

      cv::Mat cvimg = cv::Mat(cv::Size(uimg->getWidth(), uimg->getHeight()), CV_8UC3, uimg->getPixels());
      cv::cvtColor(cvimg, cvimg, cv::COLOR_RGB2BGR);
      cv::flip(cvimg, cvimg, 0);

      // Annotate current perspective
      std::vector<cv::Rect> sighted_balls;
      getTennisBallScreenLocations(sighted_balls);
      for (auto dt : sighted_balls) {
        cv::rectangle(cvimg, dt, cv::Scalar(255, 255, 255, 255), 1);
        // printf("drew-rect %i %i %i %i\n", dt.x, dt.y, dt.width, dt.height);
      }

      std::string fn = std::string("/home/simpson/proj/tennis_court/ss/ann_ss_")
                           .append(std::to_string(annotation_index))
                           .append(".jpg");
      cv::imwrite(fn.c_str(), cvimg);
      Log::message("image sample '%s' written to file\n", fn.c_str());

      ++annotation_index;
    }
    else {
      // Pressed
      prev_AUX9_state = 1;
    }
  }
}

void Alligator::update()
{
  float ifps = Game::getIFps();

  // Update fps label
  char buf[10];
  sprintf(buf, "%i", (int)App::getMeanFps());
  fpsLabel->setText(buf);

  // User Input
  handleUserInput();

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
        case AM_Autonomous: {
          updateAutonomy(ifps, agql, agqr);
        } break;
        case AM_AnnImg_Gen:
          updateAutoAnnotation(ifps, agql, agqr);
          break;
      }
    }
  }

  if (agql || agqr) {
    Mat4 tsfm;
    transformAlligator(agql, agqr, agt, agq, tsfm);
    // printf("agt=[%.2f %.2f 0.f] agq=%.2f\n", agt.x, agt.y, agq);
    alligator->setTransform(tsfm);

    // Ball Pickup & Respawn
    {
      static int ball_balance = 0;
      Vec3 t = alligator->getPosition() + ag_player->getCamera()->getPosition();
      t.x *= 0.5f;
      t.y *= 0.5f;
      t.z = 0.f;
      WorldBoundSphere ag_bs(t, 0.28f);
      for (auto tb : tennis_balls) {
        if (!tb->isEnabled() || !ag_bs.inside(tb->getPosition()))
          continue;
        tb->setEnabled(false);

        ++ball_balance;
      }

      if (ball_balance) {
        static float last_add = -15.f;
        last_add = (last_add + ifps) * 0.98f;
        if (last_add > 0) {
          placeRandomTennisBalls(false, true, 1);
          last_add -= 15.f - ball_balance;
          --ball_balance;
        }
      }
    }
  }
}

void Alligator::getTennisBallScreenLocations(std::vector<cv::Rect> &output)
{
  // quat cameraAngle = ag_player->getWorldRotation();
  // printf("Camera: %.2f\n", ag_player->getWorldRotation().getAngle(Vec3_up));
  vec3 tangent = ag_player->getWorldRotation().getTangent();
  mul(tangent, tangent, 0.034f);
  // float tangentAngle = getAngle(ag_player)
  // printf("Camera Tangent: %.2f %.2f %.2f\n", tangent.x, tangent.y, tangent.z);

  vec3 pos, tp;
  // int significant = 0;
  for (auto tb : tennis_balls) {
    cv::Rect bb;

    if (!tb->isEnabled())
      continue;

    pos = tb->getPosition();
    // if ((pos - ag_player->getWorldPosition()).length2() > 111 /* Under 11m^2 */) {
    //   printf("--too-far: [%.1f, %.1f] <> [%.1f, %.1f]\n", pos.x, pos.y, ag_player->getWorldPosition().x,
    //          ag_player->getWorldPosition().y);
    //   continue;
    // }

    sub(tp, pos, tangent);
    tp.z += 0.035f;
    if (!ag_player->getScreenPosition(bb.x, bb.y, tp, SampleWidth, SampleHeight)) {
      // printf("--rejectedA: [%i, %i]\n", bb.x, bb.y);
      continue;
    }

    add(tp, pos, tangent);
    tp.z -= 0.035f;
    int r, b;
    if (!ag_player->getScreenPosition(r, b, tp, SampleWidth, SampleHeight)) {
      bb.width = r - bb.x;
      bb.height = b - bb.y;
      // printf("--rejectedB: [%i, %i, w:%i, h:%i]\n", bb.x, bb.y, bb.width, bb.height);
      continue;
    }
    bb.width = r - bb.x;
    bb.height = b - bb.y;

    // printf("option-: [%i, %i, w:%i, h:%i]\n", bb.x, bb.y, bb.width, bb.height);
    if (bb.x + bb.width <= 0 || bb.x >= SampleWidth - 1 || bb.y + bb.height <= 0 || bb.y >= SampleHeight - 1)
      continue;

    // No 0 pixel bounding boxes
    if (bb.width * bb.height < 1)
      continue;

    output.push_back(bb);
    // printf("tennisball-: [%i, %i, w:%i, h:%i]\n", bb.x, bb.y, bb.width, bb.height);
  }
}

int Alligator::annotateScreen(int capture_index, const char *const path_format)
{
  std::vector<cv::Rect> found;
  getTennisBallScreenLocations(found);
  if (!found.size())
    return 0;

  // Write the annotation file
  char fp[256];
  sprintf(fp, path_format, capture_index);

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
  f << "    <width>" << SampleWidth << "</width>" << std::endl;
  f << "    <height>" << SampleHeight << "</height>" << std::endl;
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
    f << "      <xmin>" << tb.x << "</xmin>" << std::endl;
    f << "      <ymin>" << tb.y << "</ymin>" << std::endl;
    f << "      <xmax>" << tb.x + tb.width << "</xmax>" << std::endl;
    f << "      <ymax>" << tb.y + tb.height << "</ymax>" << std::endl;
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
    placeRandomTennisBalls(true, false, 0);
  }
  else if (capture_index > 10800) {
    Log::message("7200 samples generated. Ending.");
    setAutonomyMode(AM_Manual);
  }

  // Annotate the screen (if any annotations exist, return if not)
  if (annotateScreen(capture_index) == 0)
    return;

  // Save to file
  char image_path[256];
  sprintf(image_path, IMAGE_PATH_FORMAT, capture_index);
  saveTextureToFile(ag_pov_screen, image_path);
  Log::message("annotated image sample(%i) created\n", capture_index);

  ++capture_index;
}

void Alligator::placeRandomTennisBalls(bool reset_all, bool restrict_corner_court, int ball_amount)
{
  if (reset_all) {
    for (auto tb : tennis_balls) tb->setEnabled(false);

    if (ball_amount <= 0) {
      ball_amount = rng.getInt(16, 132);
      if (ball_amount > 92) {
        ball_amount -= rng.getInt(0, 15);
      }
      if (restrict_corner_court)
        ball_amount = ball_amount / 3;
    }
  }

  int bc = ball_amount;
  for (auto tb : tennis_balls) {
    if (tb->isEnabled())
      continue;

    // Enable & Place
    tb->setEnabled(true);
    if (restrict_corner_court)
      tb->setPosition(Vec3(rng.getFloat(-16.4f, -0.5f), rng.getFloat(-8.5f, 10.f), 0.034f));
    else
      tb->setPosition(Vec3(rng.getFloat(-16.4f, 16.5f), rng.getFloat(-16.5f, 10.f), 0.034f));
    tb->setRotation(quat(rng.getDir(), rng.getFloat(0.f, 360.f)));

    --bc;
    if (bc <= 0) {
      break;
    }
  }

  if (reset_all)
    printf("%s %i balls to the court\n", reset_all ? "Repositioned" : "Added", ball_amount);
}

void Alligator::captureAlligatorPOV()
{
  // Render
  // ag_viewport->appendSkipFlags(Viewport::SKIP_POSTEFFECTS);

  // saving current render state and clearing it
  RenderState::saveState();
  RenderState::clearStates();

  // Check for resize
  static int appWidth = 0, appHeight = 0;
  if (App::getWidth() != appWidth || ag_pov_screen->getHeight() != appHeight) {
    // ag_pov_screen->create2D(300, 300, Texture::FORMAT_RGB8, Texture::FILTER_POINT | Texture::USAGE_RENDER);
    appWidth = App::getWidth();
    appHeight = App::getHeight();

    float scale = MIN(0.42f * appWidth, 0.42f * appHeight);

    // adjust sprite size
    sprite->setWidth((int)scale);
    sprite->setHeight((int)scale);
    sprite->arrange();
  }

  // rendering and image from the camera of the current mirror to the texture
  ag_viewport->renderTexture2D(ag_player->getCamera(), ag_pov_screen);

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
      placeRandomTennisBalls(true, true, 0);
      break;
    case AM_AnnImg_Gen: {
      placeRandomTennisBalls(true, false, 0);

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

void Alligator::moveToTarget(const float ifps, const float agq, Vec3 &delta, float &agql, float &agqr)
{
  // Continue along prescribed path
  float theta = getAngle(Vec3_forward, delta, Vec3_up);

  // -- Move to the target
  // Reduce the angle
  float angularDiff = theta - agq;
  wrapAngle(angularDiff);
  // float turnForce = Unigine::Math::abs(angularDiff);
  // float qlm, qrm;

  const float MaxSpeed = DEFAULT_SPEED;
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
}

void Alligator::updateAutoAnnotation(const float ifps, float &agql, float &agqr)
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
  moveToTarget(ifps, agq, delta, agql, agqr);
}

void evaluationCallback(void *state, std::vector<DetectedTennisBall> &result)
{
  Alligator::EvalState &es = *(Alligator::EvalState *)state;
  // printf("evaluationCallback() detected_count=%lu\n", result.size());

  mat4 agc_view = lookAt(es.agc_t, es.agc_t + es.agc_dir, Vec3_up);

  // -- Invert the proj/view matrix
  mat4 ivp;
  mul(ivp, es.agc_proj, agc_view);
  ivp = inverse(ivp);
  std::queue<Alligator::BallDetection> detections;
  es.eval_detections.clear();

  for (auto b : result) {
    // printf("Ball:(%i%%) [%i %i %i %i]\n", (int)(b.prob * 100), b.left, b.top, b.right, b.bottom);
    // Unproject the ball ground contact point

    // Obtain the world position
    float x = 0.5f * (b.left + b.right), y = es.eval_screengrab->getHeight() - b.bottom;
    vec4 near((float)(x / es.eval_screengrab->getWidth() - 0.5f) * 2.f,
              (float)(y / es.eval_screengrab->getHeight() - 0.5f) * 2.f, -1.f, 1.f);
    vec4 far((float)(x / es.eval_screengrab->getWidth() - 0.5f) * 2.f,
             (float)(y / es.eval_screengrab->getHeight() - 0.5f) * 2.f, 1.f, 1.f);

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
    dt.scr.l = b.left;
    dt.scr.t = b.top;
    dt.scr.r = b.right;
    dt.scr.b = b.bottom;
    dt.x = pred.x;
    dt.y = pred.y;
    dt.dist2 = (es.agc_t - Vec3(pred.x, pred.y, 0.f)).length2();
    dt.prob = b.prob;
    detections.push(dt);

    es.eval_detections.push_back(dt);

    // eval_pred = pred;
    // printf("dir: %.2f %.2f %.2f\n", dir.x, dir.y, dir.z);
    // printf("prd: %.2f %.2f 0\n", pred.x, pred.y);
  }

  // Begin
  std::lock_guard<std::mutex> lg(es.td_mutex);

  // Cleanup Finished Tracked detections
  for (auto tdi = es.tracked_detections.begin(); tdi != es.tracked_detections.end();) {
    auto ptd = *tdi;
    if (ptd->primary_target == TargetStatus::MarkedForRemoval) {
      tdi = es.tracked_detections.erase(tdi);
      printf("Removing Finished Target [%.2f %.2f]\n", ptd->x, ptd->y);
      continue;
    }
    ++tdi;
  }

  // Find the best existing tdi to attach the reading to
  // --or create another one
  while (detections.size()) {
    auto b = detections.front();
    detections.pop();

    // printf("detection: %.2f %.2f %.2f\n", b.x, b.y, b.prob);

    auto bat = es.tracked_detections.end();
    float bs = 0.f;
    for (auto tdi = es.tracked_detections.begin(); tdi != es.tracked_detections.end(); ++tdi) {
      auto ptd = *tdi;
      float score = ((b.x - ptd->x) * (b.x - ptd->x) + (b.y - ptd->y) * (b.y - ptd->y));

      if (score > 0.8f || (bs != 0.f && score > bs))
        continue;

      if (ptd->eval_score != 0.f) {
        if (score < ptd->eval_score) {
          // Better match than previous tie
          bat = tdi;
          bs = score;
        }
        continue;
      }

      if (bat != es.tracked_detections.end()) {
        ptd->eval_score = 0.f;
      }

      // Better match than previous tie
      bat = tdi;
      bs = score;
    }

    // Set
    if (bs != 0.f) {
      auto ptd = *bat;
      if (ptd->eval_score != 0.f) {
        detections.push(ptd->eval_alloc);
      }
      ptd->eval_alloc = b;
      ptd->eval_score = bs;
    }
    else {
      // Construct another node
      TrackedDetection ap = std::make_shared<Alligator::_TrackedDetection>();
      ap->x = 0;
      ap->y = 0;
      ap->occ = 0;
      ap->score = 0.5f;  // Initial boost
      ap->eval_score = 0.00001f;
      ap->primary_target = TargetStatus::None;

      ap->eval_alloc = b;

      es.tracked_detections.push_back(ap);
    }
  }

  // Decay all occupancies in the view frustrum
  WorldBoundFrustum bf(es.agc_proj, agc_view);

  float decay_rate = 0.0003f * Math::pow((float)es.tracked_detections.size() / 60, 3.5f);
  // printf("tracked_detections.size()=%zu  decay_rate=%.4f\n", es.tracked_detections.size(), decay_rate);

  // Integrate any set eval scores
  for (auto tdi = es.tracked_detections.begin(); tdi != es.tracked_detections.end();) {
    auto ptd = *tdi;

    // Decay score
    // float decay = 0.00001f + decay_rate * MAX(1, 7 - ptd->occ);
    ptd->score -= decay_rate * (1.f + 0.15f * MAX(1, 7 - ptd->occ) + (int)((es.eval_time - ptd->prev_occ_time) / 20.f) +
                                (int)((MAX(0.01f, 0.4f - ptd->score)) / 0.1f));

    if (ptd->eval_score == 0.f) {
      if (bf.insideFast(Vec3(ptd->x, ptd->y, 0.f))) {
        // Inside view frustrum and no attached detection
        // Reduce score
        // printf("reducing [%.2f %.2f] to %.2f\n", ptd->x, ptd->y, ptd->score);
        ptd->score -= decay_rate * 3.f;
      }

      if (ptd->score <= 0 && ptd->primary_target != TargetStatus::Targetted) {
        // printf("removing [%.2f %.2f]\n", ptd->x, ptd->y);
        tdi = es.tracked_detections.erase(tdi);
        continue;
      }

      // if (ptd->primary_target == TargetStatus::Targetted) {
      //   printf("Primary Target (no reocc) = %.2f\n", ptd->score);
      // }

      tdi++;
      continue;
    }

    // if (ptd->primary_target == TargetStatus::Targetted) {
    //   printf("Primary Target (reocc) = %.2f\n", ptd->score);
    // }
    // printf("eval_alloc: [%.2f %.2f] dist=%.2f %%:%.2f\n", ptd->eval_alloc.x, ptd->eval_alloc.y,
    // ptd->eval_alloc.dist2,
    //        ptd->eval_alloc.prob);

    float ev_sc_prob = ptd->eval_alloc.prob * ptd->eval_alloc.prob;
    float ev_sc_dist = ptd->eval_alloc.prob * 0.01f * (121.f - MIN(118.f, ptd->eval_alloc.dist2));
    float ev_sc_occ_bonus = MIN(1.f, 0.04f * ptd->occ * ptd->occ);
    float ev_score = 0.1f * ev_sc_prob + 0.7f * ev_sc_dist + 0.2f * ev_sc_occ_bonus;

    ++ptd->occ;
    ptd->prev_occ_time = es.eval_time;
    float mod = 1.f;
    if (ptd->occ > 1) {
      mod = (1.f / ptd->occ) + 0.4f * ev_sc_dist;
    }

    ptd->x = ptd->x * (1.f - mod) + ptd->eval_alloc.x * mod;
    ptd->y = ptd->y * (1.f - mod) + ptd->eval_alloc.y * mod;

    float prev_score = ptd->score;
    ptd->score = ptd->score * (1.f - mod) + mod * ev_score;

    // printf("modified [%.2f %.2f] to %.2f=>%.2f (mod=%.2f prob=%.2f dist=%.2f ocb=%.2f(%i))\n", ptd->x, ptd->y,
    //        prev_score, ptd->score, mod, 0.2f * ev_sc_prob, 0.7f * ev_sc_dist, 0.1f * ev_sc_occ_bonus, ptd->occ);

    // Reset
    ptd->eval_score = 0.f;
    tdi++;
  }

  // Toggle
  es.eval_in_progress = false;
}

void Alligator::processAndReissueEvaluation(const Vec3 &ag_cam_position)
{
  std::lock_guard<std::mutex> lg(eval_state.td_mutex);
  // saveTextureToFile(ag_pov_screen, "/home/simpson/proj/tennis_court/screenshot.jpg");
  // Integrate the results of the previous evaluation

  // cv::Mat img;
  // bool draw_pred = false;
  static int imgnb = 0;
  if (0 && eval_state.eval_detections.size()) {
    ++imgnb;
    if (imgnb < 42) {
      static ImagePtr uimg = Image::create();
      // saveTextureToFile(eval_state.eval_screengrab, SCREENSHOT_PATH);
      // draw_pred = true;
      eval_state.eval_screengrab->getImage(uimg);
      // // uimg->convertToFormat(Unigine::Image::FORMAT_RGB16);
      cv::Mat cvimg = cv::Mat(cv::Size(uimg->getWidth(), uimg->getHeight()), CV_8UC3, uimg->getPixels());
      cv::cvtColor(cvimg, cvimg, cv::COLOR_RGB2BGR);
      cv::flip(cvimg, cvimg, 0);

      // for (auto dt : eval_state.eval_detections) {
      //   cv::rectangle(cvimg, cv::Rect(dt.scr.l, dt.scr.t, dt.scr.r - dt.scr.l, dt.scr.b - dt.scr.t),
      //                 cv::Scalar(255, 255, 255, 255), 2);
      //   printf("drew-rect [%i %i %i %i] --> [%.1f %.1f]\n", dt.scr.l, dt.scr.t, dt.scr.r - dt.scr.l,
      //          dt.scr.b - dt.scr.t, dt.x, dt.y);

      //   //  Visualizer::renderLine3D()

      //   char buf[8];
      //   sprintf(buf, "%i%%", (int)(100 * dt.prob));
      //   cv::putText(cvimg, buf, cv::Point(dt.scr.l + 20, dt.scr.b + 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
      //               cv::Scalar(255, 0, 255, 2));
      // }

      std::string fn =
          std::string("/home/simpson/proj/tennis_court/ss/detect_").append(std::to_string(imgnb)).append(".jpg");
      cv::imwrite(fn.c_str(), cvimg);
      printf("write file '%s'\n", fn.c_str());
    }
  }

  int a = 0;
  // Update Visual Percentage Poles
  std::sort(eval_state.tracked_detections.begin(), eval_state.tracked_detections.end(),
            [](const TrackedDetection &a, const TrackedDetection &b) -> bool { return a->score > b->score; });
  for (a = 0; a < percent_poles.size() && a < eval_state.tracked_detections.size(); ++a) {
    auto td = eval_state.tracked_detections[a].get();
    percent_poles[a]->setEnabled(true);

    if (td->primary_target == TargetStatus::Targetted)
      markerPole->setPosition(Vec3(td->x, td->y, 0.f));
    else
      percent_poles[a]->setPosition(Vec3(td->x, td->y, -0.4f + 0.8f * MIN(MAX(0.f, td->score), 1.f)));

    // DEBUG
    // setAutonomyMode(AM_Manual);
  }
  for (; a < percent_poles.size(); ++a) percent_poles[a]->setEnabled(false);

  // Update the render positions of detected ball markers
  for (a = 0; a < eval_state.eval_detections.size() && a < tb_markers.size(); ++a) {
    ObjectMeshStaticPtr tbm = tb_markers[a];
    Alligator::BallDetection bd = eval_state.eval_detections[a];
    tbm->setEnabled(true);
    tbm->setPosition(Vec3(-.115f + bd.x, bd.y, 0.4f));
  }
  // printf("showing %i ball detections\n", a);
  for (; a < tb_markers.size(); ++a) {
    tb_markers[a]->setEnabled(false);
  }

  // Remove any trackings that have probably been run over or knocked by the alligator
  {
    Vec2 t = (alligator->getPosition().xy + ag_player->getCamera()->getPosition().xy) * 0.5f;

    for (auto tdi = eval_state.tracked_detections.begin(); tdi != eval_state.tracked_detections.end();) {
      auto td = *tdi;
      if (td->primary_target == TargetStatus::Targetted ||
          (t.x - td->x) * (t.x - td->x) + (t.y - td->y) * (t.y - td->y) > 0.45f * 0.45f) {
        // Continue on
        ++tdi;
        continue;
      }

      // Remove it
      tdi = eval_state.tracked_detections.erase(tdi);
    }
  }

  // -- Queue another evaluation
  if (eval_thread->isUnoccupied()) {
    // -- Store Data
    eval_state.agc_t = ag_cam_position;
    eval_state.agc_dir = Vec3(getVector2FromAngle(agq), 0.f);
    mul(eval_state.agc_dir, eval_state.agc_dir, 0.9656086f);
    eval_state.agc_dir.z = -0.26f;
    //   eval_state.agc_dir;

    // puts(">> issuing evaluation...");
    // printf("eval_agc_t: %.2f %.2f %.2f\n", eval_state.agc_t.x, eval_state.agc_t.y, eval_state.agc_t.z);
    // printf("eval_acd: %.2f %.2f %.2f\n", eval_state.agc_dir.x, eval_state.agc_dir.y, eval_state.agc_dir.z);
    // printf("view-dir: %.2f %.2f %.2f agq=%.2f\n", ag_player->getViewDirection().x,
    // ag_player->getViewDirection().y,
    //        ag_player->getViewDirection().z, agq);
    // ag_player->getCamera()->
    // printMatrix("cam-acp:", ag_player->getCamera()->getAspectCorrectedProjection(1.f));
    // printMatrix("agp-acp:", ag_player->getAspectCorrectedProjection(SampleWidth, SampleHeight));
    eval_state.agc_proj = ag_player->getCamera()->getAspectCorrectedProjection(1.f);
    // eval_state.agc_proj = ag_player->getCamera()->getAspectCorrectedProjection
    // eval_state.agc_proj = ag_player->getAspectCorrectedProjection();
    // eval_state.agc_proj = ag_player->getAspectCorrectedProjection(SampleWidth, SampleHeight);
    eval_state.eval_time = Game::getTime();
    eval_state.eval_screengrab->copy(ag_pov_screen);

    eval_state.eval_in_progress = true;
    if (!eval_thread->queueEvaluation(eval_state.eval_screengrab, this, evaluationCallback))
      eval_state.eval_in_progress = false;
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

/*
    Updates the frame in autonomous mode
    @ag_mv_l The actual distance in M to move the left wheel this frame
    @ag_mv_r The actual distance in M to move the right wheel this frame
*/
void Alligator::updateAutonomy(const float ifps, float &ag_mv_l, float &ag_mv_r)
{
  // Handle evaluation processing and update
  if (!eval_state.eval_in_progress)
#ifdef USE_ESTIMATE_MODEL
    processAndReissueEvaluation(est_cam_pos);
#else
    processAndReissueEvaluation(ag_player->getCamera()->getPosition());
#endif

#ifdef USE_ESTIMATE_MODEL
  float v_agq = ag_slam.getEstimateRotation() + est_offset_agq;
  Vec3 v_agt;
  {
    // Convert from SLAM estimated camera position to estimated model root
    Vec2 est_cam_offset = rotateVector2ByAngle(ag_cam_offset.xy, v_agq);
    v_agt = Vec3(ag_slam.getEstimatePosition().xy - est_cam_offset + est_offset_agt.xy, 0.f);
  }
#else
  float v_agq = agq;
  Vec3 v_agt = agt;
#endif

  // Allocate Behaviour
  static float wps_time;
  static TrackedDetection target = nullptr;
  if (wps == NSUnassigned) {
    if (target) {
      target->primary_target = TargetStatus::MarkedForRemoval;
      target = nullptr;
    }
    // printf("Before [%zu tracked detections]\n", eval_state.tracked_detections.size());
    std::lock_guard<std::mutex> lg(eval_state.td_mutex);
    if (eval_state.tracked_detections.size()) {
      // std::sort(eval_state.tracked_detections.begin(), eval_state.tracked_detections.end(),
      //           [](const TrackedDetection &a, const TrackedDetection &b) -> bool { return a->score > b->score;
      //           });

      printf("[%zu tracked detections]\n", eval_state.tracked_detections.size());
      float best_target_score = 0.f;
      int pri = 0;
      for (std::vector<TrackedDetection>::iterator target_i = eval_state.tracked_detections.begin();
           target_i != eval_state.tracked_detections.end(); ++target_i) {
        auto ptd = *target_i;

        vec3 delta = vec3(ptd->x, ptd->y, 0) - v_agt;
        float theta = getAngle(Vec3_forward, delta, Vec3_up);
        float absAngularDiff = MIN(uabs(theta - v_agq), MIN(uabs(theta + 360 - v_agq), uabs(theta - 360 - v_agq)));
        float dist = delta.length();
        if ((absAngularDiff > 20 && dist < 2) ||
            ((absAngularDiff > 30 ? 1.f : 0.f) + dist / absAngularDiff < 3.f / 90)) {
          // Too acute an angle
          continue;
        }

        float tscr = MAX(0.1f, 20.f - (dist + absAngularDiff * 7.2f / 180.f)) * 0.05f * ptd->score;
        if (tscr > best_target_score) {
          best_target_score = tscr;
          target = ptd;

          printf("NextTarget:(%.2f%% ts=%.2f) [%.2f %.2f] (%i) %s {dist=%.1f absAngularDiff=%.1f}\n", ptd->score, tscr,
                 ptd->x, ptd->y, ptd->occ, (ptd->primary_target == TargetStatus::Targetted) ? "TARGET" : "", dist,
                 absAngularDiff);
        }
      }

      if (target != nullptr) {
        wps = NSTarget;
        markerPole->setEnabled(true);
        markerPole->setPosition(Vec3(target->x, target->y, 0.f));

        target->primary_target = TargetStatus::Targetted;
        printf("moving to [%.2f %.2f] score=%.2f(%i)\n", target->x, target->y, target->score, target->occ);
      }
    }

    if (!wps) {
      puts("scanning...");
      markerPole->setEnabled(false);
      wps = NSScanning;
      wps_time = Game::getTime();
    }
  }

  // Execute behavioural step
  switch (wps) {
    case NSTarget: {
      vec3 delta = vec3(target->x, target->y, 0) - v_agt;
      float dist2 = length2(delta);

      if (dist2 < 0.3f) {
        // puts("Reached! >> NSPASS");
        wps = NSPass;
        wps_time = Game::getTime();
        markerPole->setEnabled(false);
        break;
      }

      // Just Move to target smoothly
      moveToTarget(ifps, v_agq, delta, ag_mv_l, ag_mv_r);
    } break;
    case NSPass: {
      if (Game::getTime() > wps_time + 0.5f) {
        target->primary_target = _TrackedDetection::TargetStatus::MarkedForRemoval;
        wps = NSUnassigned;
        puts("NSPASS >> FINISHED");
      }
      else {
        // Just continue on straight
        ag_mv_l = ifps * DEFAULT_SPEED * 0.8f;
        ag_mv_r = ifps * DEFAULT_SPEED * 0.8f;
      }
    } break;
    case NSScanning: {
      ag_mv_l = ifps * 0.15f;
      ag_mv_r = ifps * -0.15f;
      if (Game::getTime() > wps_time + 10.5f) {
        wps = NSUnassigned;
      }
    } break;
  }

#ifdef USE_ESTIMATE_MODEL
  // Interpret actual distance travelled to a 'noisy estimate'
  {
    // Add noise to the actual distance wanted to travel
    static float outstanding_ag_mv_l = 0.f, outstanding_ag_mv_r = 0.f;
    outstanding_ag_mv_l += ag_mv_l;
    outstanding_ag_mv_r += ag_mv_r;

    const float DistancePerSignal = 0.001402f;
    int encoded_signals_l = (int)(outstanding_ag_mv_l / DistancePerSignal);
    int encoded_signals_r = (int)(outstanding_ag_mv_r / DistancePerSignal);
    outstanding_ag_mv_l -= encoded_signals_l * DistancePerSignal;
    outstanding_ag_mv_r -= encoded_signals_r * DistancePerSignal;

    est_trav_l = DistancePerSignal * encoded_signals_l;
    // DistancePerSignal * (int)(encoded_signals_l * rng.getFloat(0.94f, 0.99f))
    est_trav_r = DistancePerSignal * encoded_signals_r;
    // DistancePerSignal * (int)(encoded_signals_l * rng.getFloat(0.98f, 1.01f))
    prev_est_trav_l = est_trav_l;
    prev_est_trav_r = est_trav_r;
  }

  // Calculate the transformation changes to the Alligator Vehicle and send a frame update to the SLAM system
  {
    Mat4 tsfm;
    Vec3 a_agt = v_agt;
    float a_agq = v_agq;
    transformAlligator(est_trav_l, est_trav_r, a_agt, a_agq, tsfm);
    // est_alligator->setTransform(tsfm);

    // Add to the translation delta the difference made to the camera by the rotation (the centre of rotation is
    // at the back between the two back wheels, not at the camera, so a rotation of the vehicle will add a translation
    // of the camera) (as well as the overall vehicle translation (a_agt - v_agt))
    Vec3 cam_translation_delta =
        a_agt - v_agt +
        Vec3(rotateVector2ByAngle(ag_cam_offset.xy, a_agq) - rotateVector2ByAngle(ag_cam_offset.xy, v_agq), 0.f);

    // Add SLAM frame
    float cam_rotate_delta = a_agq - v_agq;
    wrapAngle(cam_rotate_delta);
    ag_slam.appendFrame(cam_translation_delta, cam_rotate_delta);

    // Update the visual position of the estimated camera
    est_cam_pos = est_alligator->getPosition() + Vec3(rotateVector2ByAngle(ag_cam_offset.xy, a_agq), ag_cam_offset.z);
    est_alligator->setPosition(est_cam_pos);
  }

  if (0) {
    ImagePtr image = Image::create();
    ag_pov_screen->getImage(image);
    // image->load(SCREENSHOT_PATH);
    // image->convertToFormat(Unigine::Image::FORMAT_RGB32F);
    image->flipY();

    // input.width = image->getWidth();
    // input.height = image->getHeight();
    // printf("image format name:'%s'\n", image->getFormatName());
    // printf("numpixels:%zu\n", image->getNumPixels());
    // printf("%i %i\n", image->getWidth(), image->getHeight());
    // printf("rgba 0,0 : {%f, %f, %f}\n", image->get2D(0, 0).f.r, image->get2D(0, 0).f.g, image->get2D(0, 0).f.b);

    // cv::Mat img = cv::imread(SCREENSHOT_PATH);
    // cv::Mat img = cv::Mat(cv::Size(image->getWidth(), image->getHeight()), CV_32FC3, image->getPixels());
    cv::Mat img = cv::Mat(cv::Size(image->getWidth(), image->getHeight()), CV_8UC3, image->getPixels());
    // cv::Mat img2;
    // img.
    // if (image->getWidth() != 300 || image->getHeight() != 300)
    //   cv::resize(img, img, cv::Size(300, 300));
    cv::cvtColor(img, img, cv::ColorConversionCodes::COLOR_RGB2GRAY);

    // Initialize ORB detector
    static cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    static std::vector<cv::KeyPoint> prev_keypoints, keypoints;
    static cv::Mat prev_descriptors, descriptors;
    static cv::Mat prev_img = cv::Mat::zeros(cv::Size(300, 300), CV_8UC3);
    static cv::InputArray mask = cv::noArray();

    // find the keypoints and descriptors with ORB
    detector->detectAndCompute(img, mask, keypoints, descriptors);

    static cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create();
    static std::vector<cv::DMatch> matches;

    matches.clear();
    matcher->match(prev_descriptors, descriptors, matches);

    if (matches.size()) {
      std::sort(matches.begin(), matches.end(), [](auto a, auto b) -> bool { return a.distance <= b.distance; });

      cv::Mat final_img;
      cv::drawMatches(prev_img, prev_keypoints, img, keypoints, matches, final_img);
      // // cv::resize(final_img, final_img, cv::Size(600, 300));

      static int aaa = 0;
      ++aaa;
      if (aaa == 114) {
        printf("matches=%zu\n", matches.size());
        for (size_t mi = 0; mi < matches.size(); ++mi) {
          printf("--dist=%.3f [%.2f %.2f]\n", matches[mi].distance,
                 keypoints[matches[mi].trainIdx].pt.x - prev_keypoints[matches[mi].queryIdx].pt.x,
                 keypoints[matches[mi].trainIdx].pt.y - prev_keypoints[matches[mi].queryIdx].pt.y);
        }

        cv::imwrite("/home/simpson/proj/tennis_court/ss/orb_111.jpg", final_img);
        puts("wrote orb_111.jpg");
        // cv::imwrite("/home/simpson/proj/tennis_court/ss/orb_111.jpg", final_img);
        // puts("wrote orb_111.jpg");
      }
    }

    // Move to previous image
    prev_keypoints.swap(keypoints);
    descriptors.copyTo(prev_descriptors);
    std::swap(img, prev_img);
  }
#endif
}