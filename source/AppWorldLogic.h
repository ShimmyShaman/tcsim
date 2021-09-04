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
#include <UnigineWidgets.h>

using namespace Unigine;

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

  void screenGrabCheck();

 private:
  int initCamera();

  PlayerPtr main_player, ag_camera;
  PlayerSpectatorPtr spectator;

  int prev_AUX0_state = 0;

  float last_screenshot;
  Unigine::TexturePtr screenshot;
  // Unigine::WidgetSpritePtr sprite;
};

#endif /* APPWORLDLOGIC */
