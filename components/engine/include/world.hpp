#pragma once

#include <vector>

#include "object.hpp"

using namespace std;

class World {

private:
  std::vector<Object> master{};
  std::vector<Object> temp{};
  long id{0};

public:
  // constructor
  World() {}

  // alternate constructor
  explicit World(long worldID)
      : id(worldID) {
    library(id);
  }

  World(const World &other) = default;

  World &operator=(const World &other) = default;

  // allows world change
  void changeWorld(long worldID);

  // clear Master
  void masterClear();

  // clear Temp
  void tempClear();

  // adds correct world to temp list
  void library(long worldID);

  // Updates Temp list with any changes to the master list
  bool updateList();

  // returns renderList
  std::vector<Object> GetObjectList();

  ///////////////////////////////////////////////////////
  ///////////////// Custom Worlds///////////////////////
  //////////////////////////////////////////////////////

  // basic layout
  void basicWorld();

  void boxedIn();
};
