#pragma once

#include <vector>

#include "object.hpp"

using namespace std;

class World {

private:
  std::vector<Object> master;
  std::vector<Object> temp;
  long id;

public:
  // constructor
  World();

  // alternate constructor
  World(long worldID);

  // destructor
  ~World();

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
  std::vector<Object> GetRenderList();

  ///////////////////////////////////////////////////////
  ///////////////// Custom Worlds///////////////////////
  //////////////////////////////////////////////////////

  // basic layout
  void basicWorld();

  void boxedIn();
};
