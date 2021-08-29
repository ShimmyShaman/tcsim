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

#include "AGSpectator.h"
#include "UnigineApp.h"

using namespace Math;

// World logic, it takes effect only when the world is loaded.
// These methods are called right after corresponding world script's (UnigineScript) methods.

AppWorldLogic::AppWorldLogic() {}

AppWorldLogic::~AppWorldLogic() {}

int AppWorldLogic::init()
{
  // Write here code to be called on world initialization: initialize resources for your world scene during the world
  // start.
  initCamera();

  Log::message("init()\n");
  screenshot = nullptr;
  last_screenshot = -4.75f;

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

  player->deleteLater();

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
  player = PlayerDummy::create();
  ComponentSystem::get()->addComponent<AGSpectator>(player);

  // ComponentSystem::get
  // camera = AGSpectator::create();

  // set default camera position
  player->setPosition(Vec3(-3.f, -3.f, 0.25f));
  player->setDirection(vec3_forward, vec3_up);

  // camera->setFov(60.f);
  // camera->setZNear(0.01f);
  // camera->setZFar(10000.f);

  // camera->setPosition(Math::Vec3(-3.f, -3.f, 0.25f));
  // camera->setDirection(Math::Vec3(1.f, 0.f, 0.f), Math::Vec3_up);

  // camera->set

  Game::setPlayer(player);
  Log::message("\nPlayer Initialized OK!\n");

  return 1;
}

void AppWorldLogic::screenGrabCheck()
{
  if (last_screenshot < 0.25f)
    return;
  last_screenshot = 0.f;

  if (screenshot)
    return;

  if (!screenshot) {
    GuiPtr gui = Gui::get();
    sprite = WidgetSprite::create(gui);
    gui->addChild(sprite, Gui::ALIGN_OVERLAP | Gui::ALIGN_BACKGROUND);
    sprite->setPosition(0, 0);

    screenshot = Texture::create();
    sprite->setRender(screenshot, !Render::isFlipped());
  }

  // adjust screenshot size
  if (screenshot->getWidth() != App::getWidth() || screenshot->getHeight() != App::getHeight())
    screenshot->create2D(App::getWidth(), App::getHeight(), Texture::FORMAT_RGBA8,
                         Texture::FILTER_POINT | Texture::USAGE_RENDER);

  // adjust sprite size
  sprite->setWidth(App::getWidth() / 3);
  sprite->setHeight(App::getHeight() / 3);
  sprite->arrange();

  screenshot->copy2D();

  ImagePtr screenshot_image = Image::create();
  screenshot->getImage(screenshot_image);
  if (!Render::isFlipped())
    screenshot_image->flipY();
  screenshot_image->convertToFormat(Image::FORMAT_RGB8);

  screenshot_image->save("/home/simpson/proj/unigine/tennis_court/screenshot.png");
  Log::message("screenshot taken\n");
}