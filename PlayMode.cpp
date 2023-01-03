#include "PlayMode.hpp"

#include "LitColorTextureProgram.hpp"

#include "DrawLines.hpp"
#include "Mesh.hpp"
#include "Load.hpp"
#include "gl_errors.hpp"
#include "data_path.hpp"

#include <glm/gtc/type_ptr.hpp>

#include <random>
#include <sstream>

GLuint meshes_for_lit_color_texture_program = 0;
Load<MeshBuffer> hexapod_meshes(LoadTagDefault, []() -> MeshBuffer const * {
  MeshBuffer const *ret = new MeshBuffer(data_path("game2.pnct"));
  meshes_for_lit_color_texture_program = ret->make_vao_for_program(lit_color_texture_program->program);
  return ret;
});

Load<Scene> loaded_scene(LoadTagDefault, []() -> Scene const * {
  return new Scene(data_path("game2.scene"),
                   [&](Scene &scene, Scene::Transform *transform, std::string const &mesh_name) {
                     Mesh const &mesh = hexapod_meshes->lookup(mesh_name);

                     scene.drawables.emplace_back(transform);
                     Scene::Drawable &drawable = scene.drawables.back();

                     drawable.pipeline = lit_color_texture_program_pipeline;

                     drawable.pipeline.vao = meshes_for_lit_color_texture_program;
                     drawable.pipeline.type = mesh.type;
                     drawable.pipeline.start = mesh.start;
                     drawable.pipeline.count = mesh.count;

                   });
});

namespace { // helpers

template<class T>
std::string to_str(const T &x) {
  std::stringstream ss;
  ss << x;
  return ss.str();
}

float random_real(float min_value = 0.0f, float max_value = 1.0f) {
  static std::random_device dev{};
  static std::mt19937 rng{dev()};
  return std::uniform_real_distribution<float>{min_value, max_value}(rng);
}

} // helpers end

void Enemy::respawn(glm::vec2 position) {
  hit_point = 4;
  wobble = 0.0f;
  transform_body->position.x = position.x;
  transform_body->position.y = position.y;
  transform_body->position.z = 1.16f;
}

void Enemy::respawn_random(glm::vec2 center, float distance) {
  auto theta = random_real() * glm::pi<float>() * 2.0f;
  respawn({center.x + distance * std::cos(theta), center.y + distance * std::sin(theta)});
}

PlayMode::PlayMode() : scene(*loaded_scene) {
  //get pointers for enemies:
  for (auto &transform : scene.transforms) {
    auto type_offset = transform.name.find('.');
    auto type_str = transform.name.substr(0, type_offset);
    if (type_str == "Body") {
      auto idx = std::stoull(transform.name.substr(type_offset + 1)) - 1;
      if (enemies.size() < idx + 1) enemies.resize(idx + 1);
      enemies[idx].transform_body = &transform;
    } else if (type_str == "LeftArm") {
      auto idx = std::stoull(transform.name.substr(type_offset + 1)) - 1;
      if (enemies.size() < idx + 1) enemies.resize(idx + 1);
      enemies[idx].transform_left_arm = &transform;
      transform.rotation = glm::angleAxis(-glm::pi<float>() / 2.5f, glm::vec3{0.0f, 0.0f, 1.0f});
    } else if (type_str == "RightArm") {
      auto idx = std::stoull(transform.name.substr(type_offset + 1)) - 1;
      if (enemies.size() < idx + 1) enemies.resize(idx + 1);
      enemies[idx].transform_right_arm = &transform;
      transform.rotation = glm::angleAxis(glm::pi<float>() / 2.5f, glm::vec3{0.0f, 0.0f, 1.0f});
    } else if (type_str == "LeftLeg") {
      auto idx = std::stoull(transform.name.substr(type_offset + 1)) - 1;
      if (enemies.size() < idx + 1) enemies.resize(idx + 1);
      enemies[idx].transform_left_leg = &transform;
    } else if (type_str == "RightLeg") {
      auto idx = std::stoull(transform.name.substr(type_offset + 1)) - 1;
      if (enemies.size() < idx + 1) enemies.resize(idx + 1);
      enemies[idx].transform_right_leg = &transform;
    }
  }

  //get pointer to camera for convenience:
  if (scene.cameras.size() != 1)
    throw std::runtime_error(
        "Expecting scene to have exactly one camera, but it has " + std::to_string(scene.cameras.size()));
  camera = &scene.cameras.front();
  init_rotation = glm::angleAxis(glm::pi<float>() / 2.0f, glm::vec3(1.0f, 0.0f, 0.0f));
}

PlayMode::~PlayMode() {
}

bool PlayMode::handle_event(SDL_Event const &evt, glm::uvec2 const &window_size) {

  if (evt.type == SDL_KEYDOWN) {
    if (evt.key.keysym.sym == SDLK_ESCAPE) {
      SDL_SetRelativeMouseMode(SDL_FALSE);
      return true;
    } else if (evt.key.keysym.sym == SDLK_a) {
      left.downs += 1;
      left.pressed = true;
      return true;
    } else if (evt.key.keysym.sym == SDLK_d) {
      right.downs += 1;
      right.pressed = true;
      return true;
    } else if (evt.key.keysym.sym == SDLK_w) {
      up.downs += 1;
      up.pressed = true;
      return true;
    } else if (evt.key.keysym.sym == SDLK_s) {
      down.downs += 1;
      down.pressed = true;
      return true;
    } else if (evt.key.keysym.sym == SDLK_r) {
      restart.downs += 1;
      restart.pressed = true;
      return true;
    }
  } else if (evt.type == SDL_KEYUP) {
    if (evt.key.keysym.sym == SDLK_a) {
      left.pressed = false;
      return true;
    } else if (evt.key.keysym.sym == SDLK_d) {
      right.pressed = false;
      return true;
    } else if (evt.key.keysym.sym == SDLK_w) {
      up.pressed = false;
      return true;
    } else if (evt.key.keysym.sym == SDLK_s) {
      down.pressed = false;
      return true;
    } else if (evt.key.keysym.sym == SDLK_r) {
      restart.pressed = false;
      return true;
    }
  } else if (evt.type == SDL_MOUSEBUTTONDOWN) {
    if (SDL_GetRelativeMouseMode() == SDL_FALSE) {
      SDL_SetRelativeMouseMode(SDL_TRUE);
      return true;
    } else if (evt.button.button == SDL_BUTTON_LEFT) {
      shoot.downs += 1;
      shoot.pressed = true;
      return true;
    }
  } else if (evt.type == SDL_MOUSEBUTTONUP) {
    if (evt.button.button == SDL_BUTTON_LEFT) {
      shoot.pressed = false;
      return true;
    }
  } else if (evt.type == SDL_MOUSEMOTION) {
    if (SDL_GetRelativeMouseMode() == SDL_TRUE) {
      camera_rotation_angle += glm::vec2{
          float(evt.motion.xrel),
          float(-evt.motion.yrel)
      } / float(window_size.y) * camera->fovy;
      camera_rotation_angle.y = std::clamp(camera_rotation_angle.y, -glm::pi<float>() / 2.0f, glm::pi<float>() / 2.0f);
      camera->transform->rotation = glm::normalize(
          init_rotation
              * glm::angleAxis(-camera_rotation_angle.x, glm::vec3(0.0f, 1.0f, 0.0f))
              * glm::angleAxis(camera_rotation_angle.y, glm::vec3(1.0f, 0.0f, 0.0f)));
      return true;
    }
  }

  return false;
}

void PlayMode::update(float elapsed) {

  if (restart.pressed && restart_button_released) {
    restart_button_released = false;
    restart_game();
  } else if (!restart.pressed) {
    restart_button_released = true;
  }

  if (!running) return;

  //move camera:
  {

    //combine inputs into a move:
    glm::vec2 move = glm::vec2(0.0f);
    if (left.pressed && !right.pressed) move.x = -1.0f;
    if (!left.pressed && right.pressed) move.x = 1.0f;
    if (down.pressed && !up.pressed) move.y = -1.0f;
    if (!down.pressed && up.pressed) move.y = 1.0f;

    //make it so that moving diagonally doesn't go faster:
    if (move != glm::vec2(0.0f)) move = glm::normalize(move) * player_speed * elapsed;

    glm::mat4x3 frame = camera->transform->make_local_to_parent();
    glm::vec3 frame_right = frame[0];
    glm::vec3 frame_forward = -frame[2];
    frame_right = glm::normalize(glm::vec3{frame_right.x, frame_right.y, 0.0f});
    frame_forward = glm::normalize((glm::vec3{frame_forward.x, frame_forward.y, 0.0f}));
    camera->transform->position += move.x * frame_right + move.y * frame_forward;
  }

  for (auto &enemy : enemies) {
    auto &d = enemy.direction;
    d = camera->transform->position - enemy.transform_body->position;
    d = glm::normalize(d);
    auto angle = std::atan2(d.y, d.x);
    enemy.transform_body->rotation = glm::angleAxis(angle + glm::pi<float>() / 2.0f, glm::vec3{0.0f, 0.0f, 1.0f});
    enemy.transform_body->position += glm::vec3{d, 0.0f} * enemy_speed * elapsed;

    enemy.wobble += elapsed;
    enemy.transform_left_leg->rotation =
        glm::angleAxis(glm::pi<float>() / 6.0f * std::sin(4.0f * enemy.wobble), glm::vec3{1.0f, 0.0f, 0.0f});
    enemy.transform_right_leg->rotation =
        glm::angleAxis(-glm::pi<float>() / 6.0f * std::sin(4.0f * enemy.wobble), glm::vec3{1.0f, 0.0f, 0.0f});
  }

  { // shoot
    time_since_last_shoot += elapsed;
    if (!shoot.pressed) {
      trigger_released = true;
    } else if (shoot.pressed && trigger_released && time_since_last_shoot >= shoot_interval) {

      time_since_last_shoot = 0.0f;
      trigger_released = false;
      auto frame = camera->transform->make_local_to_parent();

      Scene::Transform *hit_object_transform{};
      auto distance = std::numeric_limits<float>::infinity();

      for (const auto &drawable : scene.drawables) {
        auto &transform = *(drawable.transform);
        auto type_offset = transform.name.find('.');
        auto type_str = transform.name.substr(0, type_offset);

        const Mesh *mesh{};
        try {
          mesh = &hexapod_meshes->lookup(type_str);
        } catch (std::exception &e) {
          std::cout << e.what() << '\n';
        }

        if (mesh) {
          auto mat = transform.make_world_to_local();

          auto origin = mat * glm::vec4{camera->transform->position, 1.0f};
          auto direction = mat * glm::vec4{-frame[2], 0.0f};
          direction = glm::normalize(direction);

          glm::vec3 t_min{};
          glm::vec3 t_max{};

          for (int i = 0; i < 3; ++i) {
            t_min[i] = (mesh->min[i] - origin[i]) / direction[i];
            t_max[i] = (mesh->max[i] - origin[i]) / direction[i];
            if (t_min[i] > t_max[i]) std::swap(t_min[i], t_max[i]);
          }

          t_min[0] = std::max(t_min[0], std::max(t_min[1], t_min[2]));
          t_max[0] = std::min(t_max[0], std::min(t_max[1], t_max[2]));

          if (t_min[0] > t_max[0]) continue;
          if (t_min[0] < 0.0f) t_min[0] = t_max[0];

          if (t_min[0] >= 0.0f && t_min[0] < distance) {
            distance = t_min[0];
            hit_object_transform = &transform;
          }
        }
      }

      if (hit_object_transform) {
        auto type_offset = hit_object_transform->name.find('.');
        auto type_str = hit_object_transform->name.substr(0, type_offset);
        auto idx = std::stoi(hit_object_transform->name.substr(type_offset + 1)) - 1;
        auto &enemy = enemies[idx];
        if (type_str == "Head") {
          enemy.hit_point -= 4;
        } else if (type_str == "Body") {
          enemy.hit_point -= 2;
        } else {
          enemy.hit_point -= 1;
        }

        if (enemy.hit_point <= 0) {
          ++score;
          enemy.respawn_random(camera->transform->position, 10.0f);
          enemy_speed += enemy_speed_addition_per_kill;
        }
      }
    }
  } // shoot end

  // game over
  for (const auto &enemy : enemies) {
    auto l = enemy.transform_body->position - camera->transform->position;
    if (glm::length(glm::vec2{l}) < 0.5f) {
      running = false;
    }
  }

  //reset button press counters:
  left.downs = 0;
  right.downs = 0;
  up.downs = 0;
  down.downs = 0;
  restart.downs = 0;
}

void PlayMode::draw(glm::uvec2 const &drawable_size) {
  //update camera aspect ratio for drawable:
  camera->aspect = float(drawable_size.x) / float(drawable_size.y);

  //set up light type and position for lit_color_texture_program:
  // TODO: consider using the Light(s) in the scene to do this
  glm::mat4x3 frame = camera->transform->make_local_to_parent();
  auto frame_front = glm::normalize(glm::vec3{-frame[2].x, -frame[2].y, 0.0f});
  auto frame_right = glm::normalize(glm::vec3{-frame[0].x, -frame[0].y, 0.0f});
  auto direction = glm::normalize(frame_front + frame_right);
  direction.z = -1.0f;
  glUseProgram(lit_color_texture_program->program);
  glUniform1i(lit_color_texture_program->LIGHT_TYPE_int, 1);
  glUniform3fv(lit_color_texture_program->LIGHT_DIRECTION_vec3, 1, glm::value_ptr(direction));
  glUniform3fv(lit_color_texture_program->LIGHT_ENERGY_vec3, 1, glm::value_ptr(glm::vec3(1.0f, 1.0f, 1.0f)));
  glUseProgram(0);

  glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
  glClearDepth(1.0f); //1.0 is actually the default value to clear the depth buffer to, but FYI you can change it.
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS); //this is the default depth comparison function, but FYI you can change it.

  GL_ERRORS(); //print any errors produced by this setup code

  if (running) {
    scene.draw(*camera);
  }

  { //use DrawLines to overlay some text:
    glDisable(GL_DEPTH_TEST);
    float aspect = float(drawable_size.x) / float(drawable_size.y);
    DrawLines lines(glm::mat4(
        1.0f / aspect, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    ));

    float H = 0.09f;
    float ofs = 2.0f / float(drawable_size.y);
    lines.draw_text("Mouse motion rotates camera; Left mouse button shoots; WASD moves; escape ungrabs mouse",
                    glm::vec3(-aspect + 0.1f * H, -1.0 + 0.1f * H, 0.0),
                    glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f),
                    glm::u8vec4(0x00, 0x00, 0x00, 0x00));
    lines.draw_text("Mouse motion rotates camera; Left mouse button shoots; WASD moves; escape ungrabs mouse",
                    glm::vec3(-aspect + 0.1f * H + ofs, -1.0 + +0.1f * H + ofs, 0.0),
                    glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f),
                    glm::u8vec4(0xff, 0xff, 0xff, 0x00));
    lines.draw_text("Score: " + to_str(score),
                    glm::vec3(-aspect + 0.1f * H, 1.0f - 1.1f * H, 0.0),
                    glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f),
                    glm::u8vec4(0x00, 0x00, 0x00, 0x00));
    lines.draw_text("Score: " + to_str(score),
                    glm::vec3(-aspect + 0.1f * H + ofs, 1.0f - 1.1f * H + ofs, 0.0),
                    glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f),
                    glm::u8vec4(0xff, 0xff, 0xff, 0x00));

    // draw crosshair
    float crosshair_size{0.02f};
    if (running) {
      const glm::u8vec4 crosshair_color{0x00, 0xff, 0x00, 0xff};
      lines.draw(glm::vec3{-crosshair_size, 0.0f, 0.0f},
                 glm::vec3{crosshair_size, 0.0f, 0.0f},
                 crosshair_color);
      lines.draw(glm::vec3{0.0f, -crosshair_size, 0.0f},
                 glm::vec3{0.0f, crosshair_size, 0.0f},
                 crosshair_color);
    }

    // prompt player to restart (or start) the game
    if (!running) {
      lines.draw_text("Press 'R' to start the game.",
                      glm::vec3(-14 * H, -0.5f * H, 0.0),
                      glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f),
                      glm::u8vec4(0x00, 0x00, 0x00, 0x00));
      lines.draw_text("Press 'R' to start the game.",
                      glm::vec3(-14 * H + ofs, -0.5f * H + ofs, 0.0),
                      glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f),
                      glm::u8vec4(0xff, 0xff, 0xff, 0x00));
    }
  }
}

void PlayMode::restart_game() {
  camera->transform->rotation = init_rotation;
  camera->transform->position = glm::vec3{0.0f, 0.0f, 1.7f};
  camera_rotation_angle = glm::vec2{};

  for (auto &enemy : enemies) {
    if (!enemy.transform_body
        || !enemy.transform_left_arm || !enemy.transform_right_arm
        || !enemy.transform_left_leg || !enemy.transform_right_leg) {
      throw std::runtime_error{"NULL transform pointers in enemies."};
    } else {
      enemy.respawn_random(camera->transform->position, 10.0f);
    }
  }

  score = 0;
  enemy_speed = init_enemy_speed;
  time_since_last_shoot = 0.0f;
  trigger_released = true;
  running = true;
}
