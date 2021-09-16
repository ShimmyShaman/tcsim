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

#include "AppWorldLogic.h"

#include <torch/script.h>  // One-stop header.

#include <atomic>
#include <fstream>
#include <iostream>

#include "AGSpectator.h"
#include "UnigineApp.h"
#include "UniginePlayers.h"
#include "UnigineStreams.h"

using namespace Math;

// World logic, it takes effect only when the world is loaded.
// These methods are called right after corresponding world script's (UnigineScript) methods.

AppWorldLogic::AppWorldLogic() {}

AppWorldLogic::~AppWorldLogic() {}

NodePtr alligator;
float agq;
Vec3 agt;
bool alligator_mode;

mat4 eval_agt, eval_agc_proj;
std::atomic_bool eval_in_progress;

torch::NoGradGuard no_grad;  // TODO check if removing this helps memory
torch::jit::script::Module mb1ssd;

int AppWorldLogic::init()
{
  // Write here code to be called on world initialization: initialize resources for your world scene during the world
  // start.
  // initCamera();

  try {
    mb1ssd = torch::jit::load("/home/simpson/proj/tennis_court/py/ssd_voc.pt");
  }
  catch (const c10::Error &e) {
    std::cerr << "Error loading the mb1-ssd model\n" << e.what() << std::endl;
    return -1;
  }

  Log::message("init()\n");
  last_screenshot = -4.75f;

  ControlsApp::setStateKey(Controls::STATE_AUX_0, (int)'t');
  ControlsApp::setStateKey(Controls::STATE_AUX_1, (int)'u');
  ControlsApp::setStateKey(Controls::STATE_AUX_2, (int)'o');
  ControlsApp::setStateKey(Controls::STATE_AUX_3, (int)'j');
  ControlsApp::setStateKey(Controls::STATE_AUX_4, (int)'l');
  ControlsApp::setStateKey(Controls::STATE_AUX_5, (int)'y');
  ControlsApp::setStateKey(Controls::STATE_AUX_5, (int)'y');
  ControlsApp::setStateKey(Controls::STATE_AUX_6, App::KEY_F4);

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
  randomize_tennis_ball_placements();

  eval_in_progress = false;
  eval_thread = new AgEvalThread();
  eval_thread->run();

  return 1;
}

////////////////////////////////////////////////////////////////////////////////
// start of the main loop
////////////////////////////////////////////////////////////////////////////////

int AppWorldLogic::update()
{
  // Write here code to be called before updating each render frame: specify all graphics-related functions you want to
  // be called every frame while your application executes.
  float ifps = Game::getIFps();

  last_screenshot += ifps;

  if (main_player->getControls()->getState(Controls::STATE_AUX_6)) {
    App::exit();
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
      setAutonomyMode(!auton_control);
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

  if (auton_control) {
    if (agql || agqr)
      setAutonomyMode(false);
    else
      updateAutonomy(ifps, agql, agqr);
  }

  if (agql || agqr) {
    const float wheel_seperation = 0.4685f;
    const float dual_rotation_circumference = 2.f * Math::Consts::PI * 0.5f * wheel_seperation;
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
        }
        else {
          amt = agql;
          agqr += amt;
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
        }
        else {
          amt = -agql;
          agqr -= amt;
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

    mul(tsfm, translate(agt), rotate(Vec3_up, agq));
    alligator->setTransform(tsfm);
  }

  return 1;
}

int AppWorldLogic::postUpdate()
{
  // The engine calls this function after updating each render frame: correct behavior after the state of the node has
  // been updated.

  captureAlligatorPOV();

  return 1;
}

int AppWorldLogic::updatePhysics()
{
  // Write here code to be called before updating each physics frame: control physics in your application and put
  // non-rendering calculations. The engine calls updatePhysics() with the fixed rate (60 times per second by default)
  // regardless of the FPS value. WARNING: do not create, delete or change transformations of nodes here, because
  // rendering is already in progress.
  return 1;
}

////////////////////////////////////////////////////////////////////////////////
// end of the main loop
////////////////////////////////////////////////////////////////////////////////

int AppWorldLogic::shutdown()
{
  // Write here code to be called on world shutdown: delete resources that were created during world script execution
  // to avoid memory leaks.

  // player->deleteLater();
  eval_thread->stop();
  delete eval_thread;

  return 1;
}

int AppWorldLogic::save(const Unigine::StreamPtr &stream)
{
  // Write here code to be called when the world is saving its state (i.e. state_save is called): save custom user
  // data to a file.
  UNIGINE_UNUSED(stream);
  return 1;
}

int AppWorldLogic::restore(const Unigine::StreamPtr &stream)
{
  // Write here code to be called when the world is restoring its state (i.e. state_restore is called): restore custom
  // user data to a file here.
  UNIGINE_UNUSED(stream);
  return 1;
}

int AppWorldLogic::initCamera()
{
  // ComponentSystem::get()->addComponent<AGSpectator>(alligator);

  // ComponentSystem::get
  // camera = AGSpectator::create();

  // set default camera position
  // player->setPosition(Vec3(-3.f, -3.f, 0.25f));
  // player->setDirection(vec3_forward, vec3_up);

  // camera->setFov(60.f);
  // camera->setZNear(0.01f);
  // camera->setZFar(10000.f);

  // camera->setPosition(Math::Vec3(-3.f, -3.f, 0.25f));
  // camera->setDirection(Math::Vec3(1.f, 0.f, 0.f), Math::Vec3_up);

  // camera->set

  // Game::setPlayer(player);
  // Log::message("\nPlayer Initialized OK!\n");
  return 1;
}

const char *const IMAGE_PATH_FORMAT = "/home/simpson/data/tennis_court/JPEGImages/ss_%i.jpg";
const char *const ANNOTATION_PATH_FORMAT = "/home/simpson/data/tennis_court/Annotations/ss_%i.xml";
const char *const SCREENSHOT_PATH = "/home/simpson/proj/tennis_court/screenshot.jpg";
const char *const INFERENCE_RESULT_PATH = "/home/simpson/proj/tennis_court/inference_result.txt";

int AppWorldLogic::annotateScreen(int capture_index)
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

void saveTextureToFile(TexturePtr &texture, const char *image_path)
{
  // texture->copy2D();
  ImagePtr image = Image::create();
  texture->getImage(image);
  if (!Render::isFlipped())
    image->flipY();
  image->convertToFormat(Image::FORMAT_RGB8);

  // Save to file
  image->save(image_path);
}

void AppWorldLogic::createAnnotatedSample()
{
  if (last_screenshot < 0.45f)
    return;
  last_screenshot = 0.f;

  static Vec3 last_cam_position;
  static quat last_cam_rotation;
  if (main_player != ag_player ||
      ag_player->getWorldPosition() == last_cam_position && ag_player->getWorldRotation() == last_cam_rotation)
    return;
  last_cam_position = ag_player->getWorldPosition();
  last_cam_rotation = ag_player->getWorldRotation();

  static int capture_index = 640;
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

void AppWorldLogic::randomize_tennis_ball_placements()
{
  static Random rng;

  int show = rng.getInt(2, 28);
  if (show > 14)
    show -= rng.getInt(0, 7);
  printf("Repositioned %i balls (randomly)\n", show);

  for (auto tb : tennis_balls) {
    --show;
    if (show < 0) {
      tb->setPosition(Vec3(0.f, 0.f, 10000.f));
      continue;
    }

    tb->setPosition(Vec3(rng.getFloat(-0.5f, -16.f), rng.getFloat(-7.f, 7.f), 0.034f));
    tb->setRotation(quat(rng.getDir(), rng.getFloat(0.f, 360.f)));
  }
}

void AppWorldLogic::captureAlligatorPOV()
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
  if (App::getWidth() < App::getWidth() || screenshot->getHeight() < App::getHeight()) {
    screenshot->create2D(App::getWidth(), App::getHeight(), Texture::FORMAT_RGB8,
                         Texture::FILTER_POINT | Texture::USAGE_RENDER);

    // adjust sprite size
    sprite->setWidth(320);
    sprite->setHeight(180);
    sprite->arrange();
  }

  // rendering and image from the camera of the current mirror to the texture
  ag_viewport->renderTexture2D(ag_player->getCamera(), screenshot);

  // restoring back render stateRS
  RenderState::setPolygonFront(0);
  RenderState::restoreState();
}

void AppWorldLogic::setAutonomyMode(bool autonomy) { auton_control = autonomy; }

void evaluationCallback(std::vector<DetectedTennisBall> &result)
{
  printf("evaluationCallback() detected_count=%lu\n", result.size());

  eval_in_progress = false;
}

void AppWorldLogic::updateAutonomy(float ifps, float &agql, float &agqr)
{
  if (!eval_in_progress) {
    // Integrate the results of the previous evaluation
    // TODO

    // Queue another evaluation
    // -- Store Data
    eval_agt = alligator->getTransform();
    eval_agc_proj = ag_player->getCamera()->getProjection();

    // -- Queue
    eval_in_progress = true;
    eval_thread->queueEvaluation(screenshot, evaluationCallback);

    // Formulate a new planned route
    // TODO
  }

  // Continue along prescribed path
  agql = ifps * 0.05f;
  agqr = ifps * 0.1f;
}

bool AgEvalThread::queueEvaluation(TexturePtr screenshot, void (*callback)(std::vector<DetectedTennisBall> &))
{
  ScopedLock atomic(lock);

  if (eval_queued) {
    return false;
  }
  puts("screenshot_saved");
  saveTextureToFile(screenshot, SCREENSHOT_PATH);
  eval_callback = callback;
  eval_queued = true;

  return true;
}

void AgEvalThread::process()
{
  while (isRunning()) {
    lock.lock();
    if (eval_queued) {
      lock.unlock();

      // char cmd[512];
      // sprintf(cmd, "python3 ~/proj/pytorch-ssd/ssd_inference.py %s %s", SCREENSHOT_PATH, INFERENCE_RESULT_PATH);
      // system(cmd);
      // Create a vector of inputs.
      std::vector<torch::jit::IValue> inputs;
      auto ip0 = torch::ones({1, 3, 300, 300}).cuda();
      inputs.push_back(ip0);
      puts("ab");

      // Execute the model and turn its output into a tensor.
      {
        mb1ssd.forward(inputs);
        puts("ac");
      }

      ip0.cpu();
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