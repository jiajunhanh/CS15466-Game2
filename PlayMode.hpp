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

  void respawn(glm::vec2 position);
  void respawn_random(glm::vec2 center, float distance);
};

struct PlayMode : Mode {
  PlayMode();
  virtual ~PlayMode();

  //functions called by main loop:
  virtual bool handle_event(SDL_Event const &, glm::uvec2 const &window_size) override;
  virtual void update(float elapsed) override;
  virtual void draw(glm::uvec2 const &drawable_size) override;

  //reset function
  void restart_game();

  //settings:
  static constexpr float shoot_interval{0.05f};
  static constexpr float player_speed{3.0f};
  static constexpr float init_enemy_speed{0.5f};
  static constexpr float enemy_speed_addition_per_kill{0.1f};
  static constexpr float default_sensitivity{1.0f};

  //----- game state -----

  unsigned int score{};
  float enemy_speed{};
  float time_since_last_shoot{};
  float mouse_sensitivity{1.0f};
  bool trigger_released{};
  bool restart_button_released{};
  bool running{};

  //input tracking:
  struct Button {
    uint8_t downs = 0;
    uint8_t pressed = 0;
  } left, right, down, up, shoot, restart;

  //local copy of the game scene (so code can change it during gameplay):
  Scene scene;
  std::vector<Enemy> enemies;

  //camera:
  Scene::Camera *camera = nullptr;
  glm::quat init_rotation{};
  glm::vec2 camera_rotation_angle{};
};
