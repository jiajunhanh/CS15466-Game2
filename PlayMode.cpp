#include "PlayMode.hpp"

#include "LitColorTextureProgram.hpp"

#include "DrawLines.hpp"
#include "Mesh.hpp"
#include "Load.hpp"
#include "gl_errors.hpp"
#include "data_path.hpp"

#include <glm/gtc/type_ptr.hpp>

#include <random>

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

PlayMode::PlayMode() : scene(*loaded_scene) {
  //get pointers for enemies:
  for (auto &transform : scene.transforms) {
    auto type_offset = transform.name.find('.');
    auto type_str = transform.name.substr(0, type_offset);
    if (type_str == "Body") {
      auto idx = std::stoi(transform.name.substr(type_offset + 1)) - 1;
      if (enemies.size() < idx + 1) enemies.resize(idx + 1);
      enemies[idx].transform_body = &transform;
    } else if (type_str == "LeftArm") {
      auto idx = std::stoi(transform.name.substr(type_offset + 1)) - 1;
      if (enemies.size() < idx + 1) enemies.resize(idx + 1);
      enemies[idx].transform_left_arm = &transform;
      transform.rotation = glm::angleAxis(-glm::pi<float>() / 3.0f, glm::vec3{0.0f, 0.0f, 1.0f});
    } else if (type_str == "RightArm") {
      auto idx = std::stoi(transform.name.substr(type_offset + 1)) - 1;
      if (enemies.size() < idx + 1) enemies.resize(idx + 1);
      enemies[idx].transform_right_arm = &transform;
      transform.rotation = glm::angleAxis(glm::pi<float>() / 3.0f, glm::vec3{0.0f, 0.0f, 1.0f});
    } else if (type_str == "LeftLeg") {
      auto idx = std::stoi(transform.name.substr(type_offset + 1)) - 1;
      if (enemies.size() < idx + 1) enemies.resize(idx + 1);
      enemies[idx].transform_left_leg = &transform;
    } else if (type_str == "RightLeg") {
      auto idx = std::stoi(transform.name.substr(type_offset + 1)) - 1;
      if (enemies.size() < idx + 1) enemies.resize(idx + 1);
      enemies[idx].transform_right_leg = &transform;
    }
  }

  for (auto &enemy : enemies) {
    if (!enemy.transform_body
        || !enemy.transform_left_arm || !enemy.transform_right_arm
        || !enemy.transform_left_leg || !enemy.transform_right_leg) {
      throw std::runtime_error{"NULL transform pointers in enemies."};
    }
    enemy.hit_point = 4;
  }

  //get pointer to camera for convenience:
  if (scene.cameras.size() != 1)
    throw std::runtime_error(
        "Expecting scene to have exactly one camera, but it has " + std::to_string(scene.cameras.size()));
  camera = &scene.cameras.front();
  auto init_rotation_euler = glm::eulerAngles(camera->transform->rotation);
  init_rotation = glm::angleAxis(init_rotation_euler[2], glm::vec3(0.0f, 0.0f, 1.0f))
      * glm::angleAxis(glm::pi<float>() / 2.0f, glm::vec3(1.0f, 0.0f, 0.0f));
  camera->transform->rotation = init_rotation;
  camera->transform->position.z = 1.7f;
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

          if (t_min[0] <= t_max[0] && t_min[0] < distance) {
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
          enemy.transform_body->position.x = 0.0f;
          enemy.transform_body->position.y = 0.0f;
          enemy.hit_point = 4;
        }
      }
    }
  } // shoot end

  //reset button press counters:
  left.downs = 0;
  right.downs = 0;
  up.downs = 0;
  down.downs = 0;
}

void PlayMode::draw(glm::uvec2 const &drawable_size) {
  //update camera aspect ratio for drawable:
  camera->aspect = float(drawable_size.x) / float(drawable_size.y);

  //set up light type and position for lit_color_texture_program:
  // TODO: consider using the Light(s) in the scene to do this
  glm::mat4x3 frame = camera->transform->make_local_to_parent();
  glUseProgram(lit_color_texture_program->program);
  glUniform1i(lit_color_texture_program->LIGHT_TYPE_int, 2);
  glUniform3fv(lit_color_texture_program->LIGHT_LOCATION_vec3, 1, glm::value_ptr(camera->transform->position));
  glUniform3fv(lit_color_texture_program->LIGHT_DIRECTION_vec3, 1, glm::value_ptr(-frame[2]));
  glUniform1f(lit_color_texture_program->LIGHT_CUTOFF_float, 0.95f);
  glUniform3fv(lit_color_texture_program->LIGHT_ENERGY_vec3, 1, glm::value_ptr(glm::vec3(1.0f, 1.0f, 1.0f)));
  glUseProgram(0);

  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClearDepth(1.0f); //1.0 is actually the default value to clear the depth buffer to, but FYI you can change it.
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS); //this is the default depth comparison function, but FYI you can change it.

  GL_ERRORS(); //print any errors produced by this setup code

  scene.draw(*camera);

  { //use DrawLines to overlay some text:
    glDisable(GL_DEPTH_TEST);
    float aspect = float(drawable_size.x) / float(drawable_size.y);
    DrawLines lines(glm::mat4(
        1.0f / aspect, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    ));

    constexpr float H = 0.09f;
    lines.draw_text("Mouse motion rotates camera; WASD moves; escape ungrabs mouse",
                    glm::vec3(-aspect + 0.1f * H, -1.0 + 0.1f * H, 0.0),
                    glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f),
                    glm::u8vec4(0x00, 0x00, 0x00, 0x00));
    float ofs = 2.0f / drawable_size.y;
    lines.draw_text("Mouse motion rotates camera; WASD moves; escape ungrabs mouse",
                    glm::vec3(-aspect + 0.1f * H + ofs, -1.0 + +0.1f * H + ofs, 0.0),
                    glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f),
                    glm::u8vec4(0xff, 0xff, 0xff, 0x00));
  }
}
