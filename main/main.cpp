#include <chrono>
#include <thread>

#include "esp-box.hpp"
using hal = espp::EspBox;

#include "logger.hpp"
#include "task.hpp"

#include "camera.hpp"
#include "main.hpp"
#include "object.hpp"
#include "world.hpp"

#include "file_system.hpp"
#include "jpeg.hpp"

static constexpr size_t MAX_NAME_LEN = 32;

using namespace std::chrono_literals;
using DisplayDriver = hal::DisplayDriver;

static espp::Logger logger({.tag = "Simple3d", .level = espp::Logger::Verbosity::DEBUG});
static bool display_z_buffer = false;

// frame buffers for decoding into
static uint8_t *fb0 = nullptr;
static uint8_t *fb1 = nullptr;
// DRAM for actual vram (used by SPI to send to LCD)
static uint8_t *vram0 = nullptr;
static uint8_t *vram1 = nullptr;

// video
static std::unique_ptr<espp::Task> video_task_{nullptr};
static QueueHandle_t video_queue_{nullptr};
static bool initialize_video();
static bool video_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);
static void clear_screen();
static void push_frame(const void *frame);

// rendering
float *z_buffer{nullptr};          // [SIZE_X * SIZE_Y];
uint16_t *display_buffer{nullptr}; // [SIZE_X * SIZE_Y];
static void updatePixels(uint16_t *dst);

static constexpr int num_rows_in_vram = 50;
static constexpr size_t vram_size = hal::lcd_width() * num_rows_in_vram * sizeof(hal::Pixel);
static constexpr size_t fb_size = hal::lcd_width() * hal::lcd_height() * sizeof(hal::Pixel);

Camera tempeye;
static Matrix worldToCamera = Matrix();
static Matrix perspectiveProjection = Matrix();
static Matrix projectionToPixel = Matrix();
static std::vector<Object> objectlist;  // used for the static world objects
static std::vector<Object> dynamiclist; // used for dynamic objects received from server
static std::vector<Poly> renderlist;    // aggregate polygon list to be rendered

uint16_t *defaulttexture = nullptr;
size_t defaulttexture_width = 0;
size_t defaulttexture_height = 0;

uint16_t *box_tex = nullptr;
size_t box_tex_width = 0;
size_t box_tex_height = 0;

uint16_t *stone_tex = nullptr;
size_t stone_tex_width = 0;
size_t stone_tex_height = 0;

uint16_t *wood_tex = nullptr;
size_t wood_tex_width = 0;
size_t wood_tex_height = 0;

uint16_t *ceiling_tex = nullptr;
size_t ceiling_tex_width = 0;
size_t ceiling_tex_height = 0;

enum ObjectType { // These are the types of dynamic objects which need to be tracked by the server
  PLAYER,
  SHOT
};

struct Object_s {
  ObjectType type{SHOT};
  size_t id{0};
  double x, y, z, // position vector
      theta, phi, // heading vector
      life,       // time to live
      vx, vy, vz; // velocity vector
  std::string content_;
  std::string killedby_;

  Object_s() {}
  Object_s(ObjectType t, size_t i, std::string_view c)
      : type(t)
      , id(i)
      , content_(c) {}

  Object_s(const Object_s &a) = default;
  Object_s &operator=(const Object_s &a) = default;

  void SetID(size_t i) { id = i; }
  void SetType(ObjectType t) { type = t; }
  void SetContent(std::string_view n) { content_ = n; }
  void SetKilledby(std::string_view k) { killedby_ = k; }
  void SetPos(const double _x, const double _y, const double _z) {
    x = _x;
    y = _y;
    z = _z;
  }
  void SetHeading(const double _t, const double _p) {
    theta = _t;
    phi = _p;
  }
  void SetLife(const double _l) { life = _l; }
  void SetVelocity(const double _x, const double _y, const double _z) {
    vx = _x;
    vy = _y;
    vz = _z;
  }

  bool Update(const double time) {
    double tr = cos(phi);
    double tx = tr * sin(theta), ty = sin(phi), tz = tr * cos(theta);
    double mag = sqrt(tx * tx + ty * ty + tz * tz);
    tx = tx / mag;
    ty = ty / mag;
    tz = tz / mag;
    x += tx * vz * time;
    y += ty * vz * time;
    z += tz * vz * time;
    life = life - time;
    return (life > 0);
  }

  bool operator==(const Object_s &b) {
    if (id == b.id && type == b.type) {
      return true;
    } else {
      return false;
    }
  }
};

struct Player_s {
  std::string name;
  size_t id{0};
  double x, y, z, // position vector
      theta, phi, // heading vector
      life,       // time to live
      vx, vy, vz; // velocity vector

  Player_s() {}
  Player_s(const Player_s &s) = default;
  Player_s(std::string_view n, size_t i)
      : name(n)
      , id(i) {}

  Player_s &operator=(const Player_s &s) = default;

  void SetName(std::string_view n) { name = n; }
  void SetID(size_t i) { id = i; }
  void SetPos(const double _x, const double _y, const double _z) {
    x = _x;
    y = _y;
    z = _z;
  }
  void SetHeading(const double _t, const double _p) {
    theta = _t;
    phi = _p;
  }
  void SetLife(const double _l) { life = _l; }
  void SetVelocity(const double _x, const double _y, const double _z) {
    vx = _x;
    vy = _y;
    vz = _z;
  }

  bool operator==(const Player_s &b) {
    if (id == b.id && name == b.name) {
      return true;
    } else {
      return false;
    }
  }
};

class Player_c {
private:
  bool registered{false};
  Player_s info;
  std::vector<Object_s> objects;
  Camera eye;
  World level;

public:
  Player_c()
      : info()
      , eye() {}
  Player_c(const Player_s &s)
      : info(s)
      , eye() {}
  Player_c(Player_c &s) { *this = s; }
  ~Player_c() {}

  Player_c &operator=(Player_c &s) = default;

  Player_s Info() const { return info; }
  void Info(const Player_s &s) { info = s; }

  std::vector<Object_s> Objects() { return objects; }

  Camera Eye() const { return eye; }
  void Eye(const Camera &e) {
    eye = e;
    tempeye = eye;
  }

  World &Level() { return level; }
  void Level(const World &l) {
    level = l;
    objectlist = level.GetRenderList();
  }
  void Level(const long id) {
    level = World(id);
    objectlist = level.GetRenderList();
  }

  void Register() { registered = true; }
  void Leave() { registered = false; }
  bool Registered() { return registered; }

  void Create(Object_s &a) { objects.push_back(a); }

  void Move(Object_s &a) {

    if (a.id == info.id && a.type == PLAYER) {
      info.life = a.life;
    } else {
      for (auto &obj : objects) {
        if (obj.id == a.id && obj.type == a.type) {
          obj.SetPos(a.x, a.y, a.z);
          obj.SetHeading(a.theta, a.phi);
          obj.SetLife(a.life);
          obj.SetVelocity(a.vx, a.vy, a.vz);
          return;
        }
      }
    }
  }

  void Update(const double time) {
    for (auto &object : objects) {
      if (object.type != PLAYER) {
        object.Update(time);
      }
    }
  }

  void Remove(const ObjectType t, const size_t _id) {
    for (auto it = objects.begin(); it != objects.end();) {
      if (it->type == t && it->id == _id) {
        it = objects.erase(it);
      } else {
        ++it;
      }
    }
  }
};

static std::unique_ptr<Player_c> player;

extern "C" void app_main(void) {
  logger.info("Bootup");

  // initialize the file system
  auto &fs = espp::FileSystem::get();
  // NOTE: partition label is configured by menuconfig and should match the
  //       partition label in the partition table (partitions.csv).
  // returns a const char*
  auto partition_label = fs.get_partition_label();
  // returns a std::string
  auto mount_point = fs.get_mount_point();
  // returns a std::filesystem::path
  auto root_path = fs.get_root_path();
  namespace stdfs = std::filesystem;
  const stdfs::path texture_dir = root_path / stdfs::path{"textures"};

  logger.info("Partition label: {}", partition_label);
  logger.info("Mount point:     {}", mount_point);
  logger.info("Root path:       {}", root_path.string());
  // human_readable returns a string with the size and unit, e.g. 1.2 MB
  auto total_space = fs.human_readable(fs.get_total_space());
  auto free_space = fs.human_readable(fs.get_free_space());
  auto used_space = fs.human_readable(fs.get_used_space());
  logger.info("Total space: {}", total_space);
  logger.info("Free space:  {}", free_space);
  logger.info("Used space:  {}", used_space);

  // check that it exists - IT SHOULDN'T
  std::error_code ec;
  logger.info("Directory {} exists: {}", texture_dir.string(), stdfs::exists(texture_dir, ec));

  Jpeg decoder;

  struct TexInfo {
    uint16_t **ptr;
    size_t *width;
    size_t *height;
  };

  std::unordered_map<stdfs::path, TexInfo> textures = {
      {texture_dir / "default.jpg",
       {&defaulttexture, &defaulttexture_width, &defaulttexture_height}},
      {texture_dir / "box.jpg", {&box_tex, &box_tex_width, &box_tex_height}},
      {texture_dir / "stone.jpg", {&stone_tex, &stone_tex_width, &stone_tex_height}},
      {texture_dir / "wood.jpg", {&wood_tex, &wood_tex_width, &wood_tex_height}},
      {texture_dir / "ceiling.jpg", {&ceiling_tex, &ceiling_tex_width, &ceiling_tex_height}},
  };

  // now go through each of the elements, decode them, and update them
  for (const auto &[path, info] : textures) {
    logger.debug("Loading texture file {}...", path.string());
    if (!stdfs::exists(path, ec)) {
      logger.error("Texture file {} does not exist", path.string());
      continue;
    }

    // decode the jpeg file
    if (!decoder.decode(path.c_str())) {
      logger.error("Couldn't decode {}", path.string());
      continue;
    }
    *info.width = decoder.get_width();
    *info.height = decoder.get_height();
    // now make a copy of the decoded pixels and update the pointer
    *info.ptr =
        (uint16_t *)heap_caps_malloc(decoder.get_width() * decoder.get_height() * sizeof(uint16_t),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!*info.ptr) {
      logger.error("Could not allocate memory for texture {}", path.string());
      continue;
    }
    std::memcpy(*info.ptr, decoder.get_decoded_data(),
                decoder.get_width() * decoder.get_height() * sizeof(uint16_t));

    logger.info("Loaded texture {}: {}x{}", path.string(), decoder.get_width(),
                decoder.get_height());
    logger.info("Texture pointer: {}", fmt::ptr(*info.ptr));
  }

  // initialize the hardware abstraction layer
  auto &hw = hal::get();
  if (!hw.initialize_lcd()) {
    logger.error("Could not initialize LCD");
    return;
  }

  // allocate some frame buffers for jpeg decoding, which should be screen-size
  // and in PSRAM
  fb0 = (uint8_t *)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  fb1 = (uint8_t *)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!fb0 || !fb1) {
    logger.error("Could not allocate frame buffers for LCD");
    if (fb0) {
      heap_caps_free(fb0);
    }
    if (fb1) {
      heap_caps_free(fb1);
    }
    return;
  }

  // allocate the required z-buffer for the engine
  constexpr size_t z_buffer_size = SIZE_X * SIZE_Y * sizeof(float);
  z_buffer = (float *)heap_caps_malloc(z_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!z_buffer) {
    logger.error("Could not allocate z-buffer");
    return;
  }

  // allocate some DMA-capable VRAM for jpeg decoding / display operations
  vram0 = (uint8_t *)heap_caps_malloc(vram_size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  vram1 = (uint8_t *)heap_caps_malloc(vram_size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  if (!vram0 || !vram1) {
    logger.error("Could not allocate VRAM for LCD");
    if (vram0) {
      heap_caps_free(vram0);
    }
    if (vram1) {
      heap_caps_free(vram1);
    }
    return;
  }

  logger.info("Allocated z-buffer: {} B", z_buffer_size);
  logger.info("Allocated frame buffers: fb0 = {} B, fb1 = {} B", fb_size, fb_size);
  logger.info("Allocated VRAM: vram0 = {} B, vram1 = {} B", vram_size, vram_size);

  // initialize the video task
  if (!initialize_video()) {
    logger.error("Could not initialize video task");
    return;
  }

  // clear the screen
  logger.info("Clearing screen");
  clear_screen();

  // make the player
  player = std::make_unique<Player_c>(Player_s("Player1", 1));
  player->Level(1); // load level 1

  // now initialize the engine

  // Structure of a transformation matrix:
  // ( r=rotation, p=projection, t=translation )
  // [ x y z w ]  | r r r p | = [ x' y' z' w']
  //              | r r r p |
  //              | r r r p |
  //              | t t t s |
  //        or
  // | r r r t | | x | = | x' |
  // | r r r t | | y |   | y' |
  // | r r r t | | z |   | z' |
  // | p p p s | | w |   | w' |

  // We use ROW vector notation

  perspectiveProjection.data[3][2] = -1; // translate z
  perspectiveProjection.data[2][3] = 1;  // project z

  projectionToPixel.data[3][0] = (double)SIZE_X * 0.5; // translate x
  projectionToPixel.data[3][1] = (double)SIZE_Y * 0.5; // translate y
  projectionToPixel.data[0][0] = (double)SIZE_X * 0.5; // scale x
  projectionToPixel.data[1][1] = (double)SIZE_Y * 0.5; // scale y

  // make a simple task that prints "Hello World!" every second
  espp::Task task({.callback = [&](auto &m, auto &cv) -> bool {
                     static int fb_index =
                         0; // frame buffer index, used to swap between fb0 and fb1
                     // select the frame buffer
                     uint16_t *fb_ptr =
                         (uint16_t *)((uint32_t)fb0 * (fb_index ^ 0x01) + (uint32_t)fb0 * fb_index);
                     // swap the frame buffer index
                     fb_index = fb_index ^ 0x01;
                     // render the scene
                     updatePixels(fb_ptr);
                     // push the frame to the video task
                     push_frame(fb_ptr);
                     logger.info("Pushed frame to video task");
                     // we don't want to stop the task, so return false
                     return false;
                   },
                   .task_config = {
                       .name = "Hello World",
                       .stack_size_bytes = 4096,
                   }});
  task.start();

  while (true) {
    // TODO[William]: Input processing
    std::this_thread::sleep_for(1s);
  }
}

// copy an image data to texture buffer, this updates what is rendered
void updatePixels(uint16_t *dst) {
  static uint16_t color = 0;

  if (!dst)
    return;

  // set the display_buffer to point to dst, so that the engine will use it.
  display_buffer = dst;

  for (int y = 0; y < SIZE_Y; y++) {
    for (int x = 0; x < SIZE_X; x++) {
      display_buffer[x + y * SIZE_X] = BACKGROUND_COLOR;
      z_buffer[x + y * SIZE_X] = DEFAULT_Z_BUFFER;
    }
  }

  renderlist.clear();

  dynamiclist.clear();
  std::vector<Object_s> dynamic = player->Objects();
  Object tempobj;
  for (const auto &it : dynamic) {
    switch (it.type) {
    case PLAYER:
      tempobj.GeneratePlayer(Point3D(it.x, it.y, it.z), it.theta, it.phi);
      break;
    case SHOT:
      tempobj.GenerateShot(Point3D(it.x, it.y, it.z), it.theta, it.phi);
      break;
    default:
      break;
    }
    tempobj.SetVelocity(Vector3D(it.vx, it.vy, it.vz));
    dynamiclist.push_back(tempobj);
  }

  for (auto &it : dynamiclist) {
    it.updateList();
    Vector3D tmppos = it.GetPosition() - player->Eye().GetPosition();
    it.TranslateTemp(tmppos);
    worldToCamera.SetIdentity();
    worldToCamera = worldToCamera * player->Eye().GetWorldToCamera();
    it.TransformToCamera(worldToCamera);
    it.TransformToPerspective(perspectiveProjection);
    std::vector<Poly> templist = it.GetRenderList();
    renderlist.insert(renderlist.end(), templist.begin(), templist.end());
  }

  for (auto &it : objectlist) {
    it.updateList();
    Vector3D tmppos = it.GetPosition() - player->Eye().GetPosition();
    it.TranslateTemp(tmppos);
    worldToCamera.SetIdentity();
    worldToCamera = worldToCamera * player->Eye().GetWorldToCamera();
    it.TransformToCamera(worldToCamera);
    it.TransformToPerspective(perspectiveProjection);
    std::vector<Poly> templist = it.GetRenderList();
    renderlist.insert(renderlist.end(), templist.begin(), templist.end());
  }

  for (auto &it : renderlist) {
    it.Clip();
    it.HomogeneousDivide();
    it.TransformToPixel(projectionToPixel);
    it.SetupRasterization(); // for speed optimization
  }

  for (int y = SIZE_Y - 1; y >= 0; y--) {
    for (auto &it : renderlist) {
      it.RasterizeFast(y);
    }
  }

  if (display_z_buffer) {
    // only used if we display the z-buffer
    uint16_t *ptr = (uint16_t *)dst;

    // display the z buffer
    for (size_t i = 0; i < IMAGE_HEIGHT; ++i) {
      for (size_t j = 0; j < IMAGE_WIDTH; ++j) {
        // copy 4 bytes at once
        *ptr = ((int)(z_buffer[j + i * SIZE_X]) << 16) + ((int)(z_buffer[j + i * SIZE_X]) << 8) +
               (int)(z_buffer[j + i * SIZE_X]);
        ++ptr;
      }
    }
  }
}

/////////////////////////////
// Video Related Functions
/////////////////////////////

bool initialize_video() {
  if (video_queue_ || video_task_) {
    return true;
  }

  video_queue_ = xQueueCreate(1, sizeof(uint16_t *));
  using namespace std::placeholders;
  video_task_ = espp::Task::make_unique({
      .callback = std::bind(video_task_callback, _1, _2, _3),
      .task_config =
          {.name = "video task", .stack_size_bytes = 4 * 1024, .priority = 20, .core_id = 1},
  });
  video_task_->start();
  return true;
}

void clear_screen() {
  static int buffer = 0;
  xQueueSend(video_queue_, &buffer, portMAX_DELAY);
}

void IRAM_ATTR push_frame(const void *frame) { xQueueSend(video_queue_, &frame, portMAX_DELAY); }

bool video_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
  const void *_frame_ptr;
  if (xQueueReceive(video_queue_, &_frame_ptr, portMAX_DELAY) != pdTRUE) {
    return false;
  }
  static constexpr int num_lines_to_write = num_rows_in_vram;
  using Pixel = hal::Pixel;

  static auto &hw = hal::get();

  auto lcd_height = hw.lcd_height();
  auto lcd_width = hw.lcd_width();
  int x_offset = 0;
  int y_offset = 0;
  DisplayDriver::get_offset(x_offset, y_offset);

  static uint16_t vram_index = 0; // has to be static so that it persists between calls

  // special case: if _frame_ptr is null, then we simply fill the screen with 0
  if (_frame_ptr == nullptr) {
    for (int y = 0; y < lcd_height; y += num_lines_to_write) {
      Pixel *_buf = (Pixel *)((uint32_t)vram0 * (vram_index ^ 0x01) + (uint32_t)vram1 * vram_index);
      int num_lines = std::min<int>(num_lines_to_write, lcd_height - y);
      // memset the buffer to 0
      memset(_buf, 0, lcd_width * num_lines * sizeof(Pixel));
      hw.write_lcd_lines(x_offset, y + y_offset, x_offset + lcd_width - 1,
                         y + y_offset + num_lines - 1, (uint8_t *)&_buf[0], 0);
      vram_index = vram_index ^ 0x01;
    }

    // now return
    return false;
  }

  for (int y = 0; y < lcd_height; y += num_lines_to_write) {
    uint16_t *_buf =
        (uint16_t *)((uint32_t)vram0 * (vram_index ^ 0x01) + (uint32_t)vram1 * vram_index);
    int num_lines = std::min<int>(num_lines_to_write, lcd_height - y);
    const uint16_t *_frame = (const uint16_t *)_frame_ptr;
    for (int i = 0; i < num_lines; i++) {
      // write two pixels (32 bits) at a time because it's faster
      for (int j = 0; j < lcd_width; j += 2) {
        uint32_t *src = (uint32_t *)&_frame[(y + i) * lcd_width + j];
        uint32_t *dst = (uint32_t *)&_buf[i * lcd_width + j];
        dst[0] = src[0]; // copy two pixels (32 bits) at a time
      }
    }
    hw.write_lcd_lines(x_offset, y + y_offset, x_offset + lcd_width - 1,
                       y + y_offset + num_lines - 1, (uint8_t *)&_buf[0], 0);
    vram_index = vram_index ^ 0x01;
  }

  return false;
}
