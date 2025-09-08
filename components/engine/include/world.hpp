#pragma once

#include <vector>

#include "object.hpp"

using namespace std;

class World {

private:
  std::vector<Object> objects{};
  long id{0};

public:
  World() {}

  explicit World(long worldID) { changeWorld(id); }

  World(const World &other) = default;

  World &operator=(const World &other) = default;

  void changeWorld(long worldID);

  // returns renderList
  std::vector<Object> &GetObjectList();

  ///////////////////////////////////////////////////////
  ///////////////// Custom Worlds///////////////////////
  //////////////////////////////////////////////////////

  // basic layout
  void basicWorld();

  void boxedIn();
};
