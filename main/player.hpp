#pragma once

#include <cstdint>
#include <string>

#include "camera.hpp"
#include "world.hpp"

struct PlayerInfo {
  std::string name{""};
  size_t id{0};
  float life{100.0f};

  PlayerInfo() {}
  explicit PlayerInfo(const PlayerInfo &s) = default;
  explicit PlayerInfo(std::string_view n, size_t i)
      : name(n)
      , id(i) {}

  PlayerInfo &operator=(const PlayerInfo &s) = default;

  void SetName(std::string_view n) { name = n; }
  void SetID(size_t i) { id = i; }
  void SetLife(float l) { life = l; }

  bool operator==(const PlayerInfo &b) const {
    if (id == b.id) {
      return true;
    } else {
      return false;
    }
  }
  bool operator!=(const PlayerInfo &b) const { return !(*this == b); }
};

class Player {
private:
  bool registered{false};
  PlayerInfo info{};
  Camera eye{};
  World level{};

public:
  Player() {}
  explicit Player(const PlayerInfo &s)
      : info(s) {}
  explicit Player(const Player &s) = default;

  Player &operator=(const Player &s) = default;

  const PlayerInfo &Info() const { return info; }
  void Info(const PlayerInfo &s) { info = s; }

  Camera &Eye() { return eye; }
  void Eye(const Camera &e) { eye = e; }

  World &Level() { return level; }
  void Level(const World &l) { level = l; }
  void Level(const long id) { level = World(id); }

  void Register() { registered = true; }
  void Leave() { registered = false; }
  bool Registered() const { return registered; }
};
