#include "Alligator.h"

#include <UnigineApp.h>
#include <UnigineGeometry.h>
#include <UniginePlayers.h>

#include <atomic>
#include <fstream>
#include <iostream>

using namespace Unigine;
using namespace Unigine::Math;

vec3 eval_agc_t;
mat4 eval_agc_ivp;
vec2 eval_img_size;
std::atomic_bool eval_in_progress;
NodePtr markerPole, mark;

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

void Alligator::init()
{
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
  randomize_tennis_ball_placements(16);

  // -- Store Data
  eval_agc_t = ag_player->getCamera()->getPosition();
  mul(eval_agc_ivp, ag_player->getCamera()->getProjection(),
      lookAt(eval_agc_t, ag_player->getViewDirection(), Vec3_up));
  inverse4(eval_agc_ivp, eval_agc_ivp);

  eval_in_progress = false;
  eval_thread = new AgEvalThread();
  eval_thread->run();

  markerPole = World::getNodeByName("MarkerPole");
  mark = World::getNodeByName("Mark");
  markerPole->setPosition(vec3(-10.9f, -4.5f, 0.f));
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

void Alligator::setAutonomyMode(bool autonomy) { auton_control = autonomy; }

void evaluationCallback(std::vector<DetectedTennisBall> &result)
{
  // printf("evaluationCallback() detected_count=%lu\n", result.size());

  for (auto b : result) {
    // printf("Ball:(%i%%) [%i %i %i %i]\n", (int)(b.prob * 100), b.left, b.top, b.right - b.left, b.bottom - b.top);
  }

  if (result.size() > 0) {
    auto b = result[0];
    printf("Ball:(%i%%) [%i %i %i %i]\n", (int)(b.prob * 100), b.left, b.top, b.right, b.bottom);
    // Obtain the world position
    float x = 0.5f * (result[0].left + result[0].right), y = result[0].top;
    vec4 near((float)(x / eval_img_size.x - 0.5f)*2.f,
              (float)(y / eval_img_size.y - 0.5f)*2.f, -1.f, 1.f);
    vec4 far((float)(x / eval_img_size.x - 0.5f)*2.f,
             (float)(y / eval_img_size.y - 0.5f)*2.f, 1.f, 1.f);

    mul(near, eval_agc_ivp, near);
    mul(far, eval_agc_ivp, far);

    near.x /= near.w;
    near.y /= near.w;
    near.z /= near.w;
    far.x /= far.w;
    far.y /= far.w;
    far.z /= far.w;
    vec3 dir = normalize(far.xyz - near.xyz);

    vec3 pred(eval_agc_t.x + dir.x * eval_agc_t.z / -dir.z, eval_agc_t.y + dir.y * eval_agc_t.z / -dir.z, 0.f);

    // eval_pred = pred;
    printf("dir: %.2f %.2f %.2f\n", dir.x, dir.y, dir.z);
    printf("prd: %.2f %.2f 0\n", pred.x, pred.y);
    // printf("mrk: %.2f %.2f 0\n", mark->getPosition().x, mark->getPosition().y);
    markerPole->setPosition(pred);
  }

  // Toggle
  eval_in_progress = false;
}

int once = 0;
void Alligator::updateAutonomy(float ifps, float &agql, float &agqr)
{
  once++;
  if (once == 20) {
    saveTextureToFile(screenshot, "/home/simpson/proj/tennis_court/screenshot.jpg");
    puts("screenshot-saved");
  }
  if (!eval_in_progress) {
    saveTextureToFile(screenshot, "/home/simpson/proj/tennis_court/screenshot.jpg");
    // Integrate the results of the previous evaluation
    // TODO

    // Queue another evaluation

    // -- Queue
    if (eval_thread->queueEvaluation(screenshot, evaluationCallback)) {
      eval_in_progress = true;

      // -- Store Data
      eval_agc_t = ag_player->getCamera()->getPosition();
      eval_img_size = vec2(screenshot->getWidth(), screenshot->getHeight());

      printMatrix("cameraProjection", ag_player->getCamera()->getProjection());
      printMatrix("playerProjection", ag_player->getProjection());
      printMatrix("aspectProjection", ag_player->getAspectCorrectedProjection());

      mul(eval_agc_ivp, ag_player->getAspectCorrectedProjection(),
          lookAt(eval_agc_t, eval_agc_t + ag_player->getViewDirection(), Vec3_up));
      printf("eval_agc_t: %.2f %.2f %.2f\n", eval_agc_t.x, eval_agc_t.y, eval_agc_t.z);
      printf("view-dir: %.2f %.2f %.2f\n", ag_player->getViewDirection().x, ag_player->getViewDirection().y,
             ag_player->getViewDirection().z);
      eval_agc_ivp = inverse(eval_agc_ivp);
    }

    // Formulate a new planned route
    // TODO
  }

  // Continue along prescribed path
  // agql = ifps * 0.05f;
  // agqr = ifps * 0.1f;
}