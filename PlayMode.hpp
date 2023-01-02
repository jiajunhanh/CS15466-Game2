#include "Mode.hpp"

#include "Scene.hpp"

#include <glm/glm.hpp>

#include <vector>
#include <deque>

struct Enemy {
  glm::vec2 direction{};
  int hit_point{};
  float wobble{};

  Scene::Transform *transform_body{};
  Scene::Transform *transform_left_arm{};
  Scene::Transform *transform_right_arm{};
  Scene::Transform *transform_left_leg{};
  Scene::Transform *transform_right_leg{};
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
