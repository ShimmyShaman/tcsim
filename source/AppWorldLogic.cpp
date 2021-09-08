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

#include <fstream>
#include <iostream>

#include "AGSpectator.h"
#include "UnigineApp.h"
#include "UniginePlayers.h"

using namespace Math;

// World logic, it takes effect only when the world is loaded.
// These methods are called right after corresponding world script's (UnigineScript) methods.

AppWorldLogic::AppWorldLogic() {}

AppWorldLogic::~AppWorldLogic() {}

NodePtr alligator;
float agq;
Vec3 agt;
bool alligator_mode;

int AppWorldLogic::init()
{
  // Write here code to be called on world initialization: initialize resources for your world scene during the world
  // start.
  // initCamera();

  Log::message("init()\n");
  screenshot = nullptr;
  last_screenshot = -4.75f;

  ControlsApp::setStateKey(Controls::STATE_AUX_0, (int)'t');
  ControlsApp::setStateKey(Controls::STATE_AUX_1, (int)'u');
  ControlsApp::setStateKey(Controls::STATE_AUX_2, (int)'o');
  ControlsApp::setStateKey(Controls::STATE_AUX_3, (int)'j');
  ControlsApp::setStateKey(Controls::STATE_AUX_4, (int)'l');

  main_player = ag_camera = Game::getPlayer();
  spectator = PlayerSpectatorPtr(static_cast<PlayerSpectator *>(World::getNodeByName("player_spectator").get()));

  alligator = World::getNodeByName("alligator");
  // ags = alligator->getScale();
  agq = alligator->getRotation().getAngle(vec3_up);
  agt = alligator->getPosition();

  // printf("dir: %.1f, %.1f, %.1f\n", alligator->getDirection().x, alligator->getDirection().y,
  //        alligator->getDirection().z);
  // alligator->setDirection(Vec3_right, Vec3_up);

  // printf("dir: %.1f, %.1f, %.1f\n", alligator->getDirection().x, alligator->getDirection().y,
  //        alligator->getDirection().z);

  NodePtr tb_root = World::getNodeByName("TennisBalls");
  for (int a = 0; a < tb_root->getNumChildren(); ++a) {
    ObjectMeshStaticPtr tb_ptr = ObjectMeshStaticPtr(static_cast<ObjectMeshStatic *>(tb_root->getChild(a).get()));

    tennis_balls.push_back(tb_ptr);

    BoundBox bb = tb_ptr->getBoundBox();
    Vec3 t = tb_ptr->getPosition();
    printf("{%.3f %.3f %.3f} {%.3f %.3f %.3f}\n", t.x + bb.getMin().x, t.y + bb.getMin().y, t.z + bb.getMin().z,
           t.x + bb.getMax().x, t.y + bb.getMax().y, t.z + bb.getMax().z);
  }

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

  if (main_player->getControls()->getState(Controls::STATE_AUX_0) != prev_AUX0_state) {
    if (prev_AUX0_state) {
      // Released
      prev_AUX0_state = 0;

      // Change Cameras
      if (main_player == ag_camera) {
        main_player = spectator;

        // spectator->flushTransform TODO
        ag_camera->setEnabled(false);
        spectator->setEnabled(true);

        alligator_mode = false;
      }
      else {
        main_player = ag_camera;

        spectator->setEnabled(false);
        ag_camera->setEnabled(true);

        alligator_mode = true;
      }
      Game::setPlayer(main_player);
    }
    else {
      // Pressed
      prev_AUX0_state = 1;
    }
  }

  // In meters wheel movement
  float agql = 0, agqr = 0;
  const float SPEED = 0.38f;
  if (main_player->getControls()->getState(Controls::STATE_AUX_1)) {
    agql = ifps * SPEED;
    // while (agql < 0) agql += 360.f;
    // while (agql > 360.f) agql -= 360.f;
  }
  if (main_player->getControls()->getState(Controls::STATE_AUX_2)) {
    agqr = ifps * SPEED;
    // while (agqr < 0) agqr += 360.f;
    // while (agqr >= 360.f) agqr -= 360.f;
  }
  if (main_player->getControls()->getState(Controls::STATE_AUX_3)) {
    agql = -ifps * SPEED;
    // while (agql < 0) agql += 360.f;
    // while (agql > 360.f) agql -= 360.f;
  }
  if (main_player->getControls()->getState(Controls::STATE_AUX_4)) {
    agqr = -ifps * SPEED;
    // while (agqr < 0) agqr += 360.f;
    // while (agqr >= 360.f) agqr -= 360.f;
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
  // Write here code to be called on world shutdown: delete resources that were created during world script execution to
  // avoid memory leaks.

  // player->deleteLater();

  return 1;
}

int AppWorldLogic::save(const Unigine::StreamPtr &stream)
{
  // Write here code to be called when the world is saving its state (i.e. state_save is called): save custom user data
  // to a file.
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

const char *const image_path = "/home/simpson/proj/unigine/tennis_court/screenshot.png";
const char *const result_path = "/home/simpson/proj/unigine/tennis_court/inference_result.txt";

void evaluateImage(ImagePtr screenshot_image)
{
  char cmd[512];
  sprintf(cmd, "python3 ~/proj/pytorch-ssd/ssd_inference.py %s %s", image_path, result_path);
  system(cmd);

  std::string line;
  std::ifstream myfile;
  myfile.open(result_path);

  if (!myfile.is_open()) {
    perror("Error open");
    exit(EXIT_FAILURE);
  }
  std::cout << "printing results:" << std::endl;
  while (getline(myfile, line)) {
    std::cout << line << std::endl;
  }
  myfile.close();
}

void AppWorldLogic::annotateScreenshot()
{
  puts("----------------------");
  // quat cameraAngle = ag_camera->getWorldRotation();
  // printf("Camera: %.2f\n", ag_camera->getWorldRotation().getAngle(Vec3_up));
  vec3 tangent = ag_camera->getWorldRotation().getTangent();
  mul(tangent, tangent, 0.034f);
  // float tangentAngle = getAngle(ag_camera)
  // printf("Camera Tangent: %.2f %.2f %.2f\n", tangent.x, tangent.y, tangent.z);

  vec3 pos, tp;
  for (auto tb : tennis_balls) {
    int x0, y0, x1, y1;

    pos = tb->getPosition();
    sub(tp, pos, tangent);
    tp.z += 0.035f;
    if (!ag_camera->getScreenPosition(x0, y0, tp))
      continue;
    add(tp, pos, tangent);
    tp.z -= 0.035f;
    if (!ag_camera->getScreenPosition(x1, y1, tp))
      continue;

    if (x1 <= 4 || x0 >= App::getWidth() - 4 || y1 <= 4 || y0 >= App::getHeight() - 4)
      continue;

    printf("tennisball-: [%i, %i, %i, %i]\n", x0, y0, x1 - x0, y1 - y0);
  }
}

void AppWorldLogic::screenGrabCheck()
{
  if (last_screenshot < 0.25f)
    return;
  last_screenshot = -5.f;

  if (screenshot)
    return;

  // static int file_index = 0;
  // ++file_index;

  // DEBUG
  annotateScreenshot();

  return;
  // DEBUG

  if (!screenshot) {
    // GuiPtr gui = Gui::get();
    // sprite = WidgetSprite::create(gui);
    // gui->addChild(sprite, Gui::ALIGN_OVERLAP | Gui::ALIGN_BACKGROUND);
    // sprite->setPosition(0, 0);

    screenshot = Texture::create();
    // sprite->setRender(screenshot, !Render::isFlipped());
  }

  // adjust screenshot size
  if (screenshot->getWidth() != App::getWidth() || screenshot->getHeight() != App::getHeight())
    screenshot->create2D(App::getWidth(), App::getHeight(), Texture::FORMAT_RGBA8,
                         Texture::FILTER_POINT | Texture::USAGE_RENDER);

  // // adjust sprite size
  // sprite->setWidth(App::getWidth() / 3);
  // sprite->setHeight(App::getHeight() / 3);
  // sprite->arrange();

  screenshot->copy2D();

  ImagePtr screenshot_image = Image::create();
  screenshot->getImage(screenshot_image);
  if (!Render::isFlipped())
    screenshot_image->flipY();
  screenshot_image->convertToFormat(Image::FORMAT_RGB8);

  // Save to file
  screenshot_image->save(image_path);
  Log::message("screenshot taken\n");

  evaluateImage(screenshot_image);
}