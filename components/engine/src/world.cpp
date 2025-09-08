#include "world.hpp"

void World::changeWorld(long worldID) {
  id = worldID;
  objects.clear();
  switch (worldID) {
  case 0:
    basicWorld();
    break;
  case 1:
    boxedIn();
    break;
  default:
    break;
  }
}

// returns renderList
std::vector<Object> &World::GetObjectList() { return objects; }

///////////////////////////////////////////////////////
///////////////// Custom Worlds///////////////////////
//////////////////////////////////////////////////////

// Pre-condition: objects list is already cleared
// basic world
void World::basicWorld() {
  Object testobj = Object(box_tex, box_tex_width, box_tex_height, Vector3D(), Point3D(-10, -5, 15));
  Object testobj2 = Object(box_tex, box_tex_width, box_tex_height, Vector3D(), Point3D(10, -5, 15));
  Object testobj3 = Object(stone_tex, stone_tex_width, stone_tex_height);

  testobj.GenerateCube();
  testobj2.GenerateCube();
  testobj2.SetRenderType(COLORED);
  testobj3.GenerateFloor(75, -10);

  objects.push_back(testobj);
  objects.push_back(testobj2);
  objects.push_back(testobj3);
}

void World::boxedIn() {
  float length = 75;
  float depth = -10;

  Object testobj = Object(box_tex, box_tex_width, box_tex_height, Vector3D(), Point3D(-10, -5, 15));
  Object testobj2 =
      Object(box_tex, box_tex_width, box_tex_height, Vector3D(), Point3D(-10, -5, 15));
  testobj.GenerateCube(15);
  testobj2.GenerateCube();
  // testobj2.SetRenderType(COLORED);

  Object wall1 = Object(stone_tex, stone_tex_width, stone_tex_height);
  Object wall2 = Object(stone_tex, stone_tex_width, stone_tex_height);
  Object wall3 = Object(stone_tex, stone_tex_width, stone_tex_height);
  Object wall4 = Object(stone_tex, stone_tex_width, stone_tex_height);
  Object floor = Object(wood_tex, wood_tex_width, wood_tex_height);
  Object ceiling = Object(ceiling_tex, ceiling_tex_width, ceiling_tex_height);

  wall1.GenerateWall(0, length, length + depth);
  wall2.GenerateWall(1, length, length + depth);
  wall3.GenerateWall(2, length, length + depth);
  wall4.GenerateWall(3, length, length + depth);

  floor.GenerateFloor(length, depth);
  ceiling.GenerateCeiling(length, (length * 2) + depth);

  objects.push_back(wall1);
  objects.push_back(wall2);
  objects.push_back(wall3);
  objects.push_back(wall4);
  objects.push_back(floor);
  objects.push_back(ceiling);
  objects.push_back(testobj);
  objects.push_back(testobj2);
}
