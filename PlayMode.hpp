#include "Mode.hpp"

#include "Scene.hpp"

#include <glm/glm.hpp>

#include <vector>
#include <deque>
#include <random>

struct Enemy {
  glm::vec2 direction{};
  int hit_point{};
  float wobble{};

  Scene::Transform *transform_body{};
  Scene::Transform *transform_left_arm{};
  Scene::Transform *transform_right_arm{};
  Scene::Transform *transform_left_leg{};
  Scene::Transform *transform_right_leg{};

  void respawn(glm::vec2 position) {
    hit_point = 4;
    wobble = 0.0f;
    transform_body->position.x = position.x;
    transform_body->position.y = position.y;
  }

  void respawn_random(glm::vec2 center, float distance) {
    static std::random_device dev{};
    static std::mt19937 rng{dev()};
    static std::uniform_real_distribution<float> dist{0.0f, glm::pi<float>() * 2.0f};

    auto theta = dist(rng);
    respawn({center.x + distance * std::cos(theta), center.y + distance * std::sin(theta)});
  }
};

struct PlayMode : Mode {
  PlayMode();
  virtual ~PlayMode();

  //functions called by main loop:
  virtual bool handle_event(SDL_Event const &, glm::uvec2 const &window_size) override;
  virtual void update(float elapsed) override;
  virtual void draw(glm::uvec2 const &drawable_size) override;

  //settings:
  static constexpr float shoot_interval{0.5f};
  static constexpr float player_speed{3.0f};
  static constexpr float enemy_speed{2.5f};

  //----- game state -----

  float time_since_last_shoot{};
  bool trigger_released{true};

  //input tracking:
  struct Button {
    uint8_t downs = 0;
    uint8_t pressed = 0;
  } left, right, down, up, shoot;

  //local copy of the game scene (so code can change it during gameplay):
  Scene scene;

  std::vector<Enemy> enemies;

  //camera:
  Scene::Camera *camera = nullptr;
  glm::quat init_rotation{};
  glm::vec2 camera_rotation_angle{};
};
