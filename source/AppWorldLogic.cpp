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
Vec3 ags, agt;
quat agq;
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
  ControlsApp::setStateKey(Controls::STATE_AUX_1, (int)'j');
  ControlsApp::setStateKey(Controls::STATE_AUX_2, (int)'l');
  ControlsApp::setStateKey(Controls::STATE_AUX_3, (int)'u');

  main_player = ag_camera = Game::getPlayer();
  spectator = PlayerSpectatorPtr(static_cast<PlayerSpectator *>(World::getNodeByName("player_spectator").get()));

  alligator = World::getNodeByName("alligator");
  ags = alligator->getScale();
  agq = alligator->getRotation();
  agt = alligator->getPosition();

  // printf("dir: %.1f, %.1f, %.1f\n", alligator->getDirection().x, alligator->getDirection().y,
  //        alligator->getDirection().z);
  // alligator->setDirection(Vec3_right, Vec3_up);

  // printf("dir: %.1f, %.1f, %.1f\n", alligator->getDirection().x, alligator->getDirection().y,
  //        alligator->getDirection().z);

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

  Mat4 transform;
  if (main_player->getControls()->getState(Controls::STATE_AUX_1)) {
    quat c(Vec3_up, -ifps * 40.f);
    mul(agq, agq, c);

    // v' = v-invscale(p-invrotate(p))
    Vec3 p(-0.23425f, 0.26262f, 0.05f);

    // mul(transform, translate(p), rotate(inverse(agq)));
    // p = p - transform.getColumn3(3);
    // Vec3 vp = agt - p;

    mul(transform, rotate(agq), translate(p));
    mul(transform, translate(agt - p), transform);
    agt = transform.getTranslate();
    agq = transform.getRotate();
    alligator->setTransform(transform);
  }
  if (main_player->getControls()->getState(Controls::STATE_AUX_2)) {
    quat c(Vec3_up, -ifps * 40.f);
    mul(agq, agq, c);

    // v' = v-invscale(p-invrotate(p))
    Vec3 p(0.23425f, 0.26262f, 0.05f);

    // mul(transform, translate(p), rotate(inverse(agq)));
    // p = p - transform.getColumn3(3);
    // Vec3 vp = agt - p;

    mul(transform, rotate(agq), translate(p));
    mul(transform, translate(agt - p), transform);
    agt = transform.getTranslate();
    agq = transform.getRotate();
    alligator->setTransform(transform);
  }
  // if (main_player->getControls()->getState(Controls::STATE_AUX_2)) {
  //   // Translate
  //   Vec3 direction;
  //   mul(transform, rotate(agq), translate(Vec3_forward));
  //   // composeTransform(transform, Vec4(Vec3_forward), agq);
  //   direction = transform.getColumn3(3);
  //   // mul(direction, agq, Vec3_forward);
  //   // mul(direction, Vec3_forwarddirection, inverse(agq));

  //   mul(direction, direction, ifps * 0.4f);
  //   add(agt, agt, direction);

  //   // Rotate
  //   quat c(Vec3_up, -ifps * 40.f);
  //   mul(agq, agq, c);

  //   // puts("attempting");
  // }

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

void evaluateImage(ImagePtr screenshot_image)
{
  return;
  const char *const image_path = "/home/simpson/proj/unigine/tennis_court/screenshot.png";
  const char *const result_path = "/home/simpson/proj/unigine/tennis_court/inference_result.txt";
  // Save to file
  screenshot_image->save(image_path);
  Log::message("screenshot taken\n");

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

void AppWorldLogic::screenGrabCheck()
{
  if (last_screenshot < 0.25f)
    return;
  last_screenshot = 0.f;

  if (screenshot)
    return;

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

  evaluateImage(screenshot_image);
}