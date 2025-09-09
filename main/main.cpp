#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "esp-box.hpp"
using hal = espp::EspBox;

#include "logger.hpp"
#include "task.hpp"

#include "camera.hpp"
#include "main.hpp"
#include "object.hpp"
#include "world.hpp"

#include "asset_loader.hpp"
#include "file_system.hpp"
#include "jpeg.hpp"
#include "png.hpp"
#include <filesystem>

#include "player.hpp"

static constexpr size_t MAX_NAME_LEN = 32;

using namespace std::chrono_literals;
using DisplayDriver = hal::DisplayDriver;

static espp::Logger logger({.tag = "Simple3d", .level = espp::Logger::Verbosity::INFO});

// frame buffers for decoding into
static uint8_t *fb0 = nullptr;
static uint8_t *fb1 = nullptr;
// DRAM for actual vram (used by SPI to send to LCD)
static uint8_t *vram0 = nullptr;
static uint8_t *vram1 = nullptr;

static int frame_count = 0;
static float FPS = 0;
static auto start = esp_timer_get_time();

// object index
static int object_index = 0;
static std::vector<Object> modelObjs;
static std::mutex object_mutex;

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
static void render();

static constexpr int num_rows_in_vram = 50;
static constexpr size_t vram_size = hal::lcd_width() * num_rows_in_vram * sizeof(hal::Pixel);
static constexpr size_t fb_size = hal::lcd_width() * hal::lcd_height() * sizeof(hal::Pixel);

static Matrix worldToCamera = Matrix();
static Matrix perspectiveProjection = Matrix();
static Matrix projectionToPixel = Matrix();
static std::vector<Object> objectlist; // used for the static world objects

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

struct Bounds {
  float minx = std::numeric_limits<float>::infinity();
  float miny = std::numeric_limits<float>::infinity();
  float minz = std::numeric_limits<float>::infinity();
  float maxx = -std::numeric_limits<float>::infinity();
  float maxy = -std::numeric_limits<float>::infinity();
  float maxz = -std::numeric_limits<float>::infinity();
  bool is_valid() const { return minx <= maxx && miny <= maxy && minz <= maxz; }
  bool is_set() const {
    return minx != std::numeric_limits<float>::infinity() &&
           miny != std::numeric_limits<float>::infinity() &&
           minz != std::numeric_limits<float>::infinity() &&
           maxx != -std::numeric_limits<float>::infinity() &&
           maxy != -std::numeric_limits<float>::infinity() &&
           maxz != -std::numeric_limits<float>::infinity();
  }
};

Bounds get_bounds() {
  Bounds b;
  for (auto &obj : objectlist) {
    Point3D mn, mx;
    if (obj.GetWorldBounds(mn, mx)) {
      b.minx = std::min(b.minx, mn.x);
      b.miny = std::min(b.miny, mn.y);
      b.minz = std::min(b.minz, mn.z);
      b.maxx = std::max(b.maxx, mx.x);
      b.maxy = std::max(b.maxy, mx.y);
      b.maxz = std::max(b.maxz, mx.z);
    }
    // Only care about the first object, not the axes
    break;
  }
  fmt::print("World bounds: min({:.2f}, {:.2f}, {:.2f}), max({:.2f}, {:.2f}, {:.2f})\n", b.minx,
             b.miny, b.minz, b.maxx, b.maxy, b.maxz);
  return b;
};
static Bounds bounds;

static std::unique_ptr<Player> player;

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
  const stdfs::path models_dir = root_path / stdfs::path{"models"};

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
  logger.info("Models dir {} exists: {}", models_dir.string(), stdfs::exists(models_dir, ec));

  static Jpeg decoder;
  static Png png_decoder;

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

  auto button_callback = [&](const auto &event) {
    if (event.active) {
      logger.info("FPS = {:0.02f}", FPS);
      // reset the FPS counter
      frame_count = 0;
      start = esp_timer_get_time();

      // increment the object index and update the render list
      std::lock_guard<std::mutex> lock(object_mutex);
      object_index++;
      if (object_index >= modelObjs.size()) {
        object_index = 0; // wrap around
      }
      // clear the render list
      objectlist.clear();
      // add the selected object to the render list
      if (object_index < modelObjs.size()) {
        objectlist.emplace_back(modelObjs[object_index]);

        bounds = get_bounds();
        // set the size of the axes to be slightly larger than the bounds of the object
        float axis_size = 0.0f;
        if (bounds.is_set()) {
          axis_size = std::max(bounds.maxx - bounds.minx,
                               std::max(bounds.maxy - bounds.miny, bounds.maxz - bounds.minz)) *
                      0.5f;
        } else {
          axis_size = 1.0f; // default size if bounds are not set
        }

        // Add world axes helper for visualization at origin
        Object axes;
        axes.GenerateAxes(axis_size * 1.5f, axis_size * 0.05f); // axes with size and thickness

        objectlist.emplace_back(axes); // add axes for reference
        logger.info("Selected object: {}", object_index);
      } else {
        logger.warn("No object selected, using default world");
        player->Level(1); // load default level if no model found
        objectlist = player->Level().GetObjectList();
      }
    }
  };

  // initialize touch
  if (!hw.initialize_boot_button(button_callback)) {
    logger.error("Could not initialize button");
    return;
  }

  // clear the screen
  logger.info("Clearing screen");
  clear_screen();

  // make the player
  player = std::make_unique<Player>(PlayerInfo("Player1", 1));
  // Try to load a model from /models. If present, replace world with that object.
  {
    asset::TextureDecodeFn fileDecoder = [&](const std::string &path, uint16_t *&outPtr, int &w,
                                             int &h) -> bool {
      logger.info("Decoding texture from file {}", path);
      // get the file extension
      std::error_code ec;
      if (!stdfs::exists(path, ec)) {
        logger.error("Texture file {} does not exist", path);
        return false;
      }
      std::string ext = stdfs::path(path).extension();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
      if (ext != ".png" && ext != ".jpg" && ext != ".jpeg") {
        logger.error("Unsupported image format: {}", ext);
        return false;
      }
      // Try PNG first, then JPEG
      if (ext == ".png") {
        if (!png_decoder.decode(path.c_str()))
          return false;
        w = png_decoder.get_width();
        h = png_decoder.get_height();
        logger.info("Decoded PNG texture {}x{}", w, h);
        outPtr = (uint16_t *)heap_caps_malloc(w * h * sizeof(uint16_t),
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!outPtr)
          return false;
        std::memcpy(outPtr, png_decoder.get_decoded_data(), w * h * sizeof(uint16_t));
        return true;
      }
      if (ext == ".jpg" || ext == ".jpeg") {
        if (!decoder.decode(path.c_str()))
          return false;
        w = decoder.get_width();
        h = decoder.get_height();
        logger.info("Decoded JPEG texture {}x{}", w, h);
        outPtr = (uint16_t *)heap_caps_malloc(w * h * sizeof(uint16_t),
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!outPtr)
          return false;
        std::memcpy(outPtr, decoder.get_decoded_data(), w * h * sizeof(uint16_t));
        return true;
      }
      return false;
    };
    asset::TextureDecodeBytesFn bytesDecoder = [&](const uint8_t *bytes, size_t len,
                                                   const std::string &mime, uint16_t *&outPtr,
                                                   int &w, int &h) -> bool {
      logger.info("Decoding texture by bytes of len {} with mimetype {}", len, mime);
      if (mime != "image/png" && mime != "image/jpeg") {
        logger.error("Unsupported image format: {}", mime);
        return false;
      }
      // handle png (using libpng)
      if (mime == "image/png") {
        if (!png_decoder.decode_memory(bytes, len))
          return false;
        w = png_decoder.get_width();
        h = png_decoder.get_height();
        logger.info("Decoded PNG texture {}x{}", w, h);
        outPtr = (uint16_t *)heap_caps_malloc(w * h * sizeof(uint16_t),
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!outPtr)
          return false;
        std::memcpy(outPtr, png_decoder.get_decoded_data(), w * h * sizeof(uint16_t));
      }
      // handle jpeg
      if (mime == "image/jpeg") {
        if (!decoder.decode_memory(bytes, len))
          return false;
        w = decoder.get_width();
        h = decoder.get_height();
        logger.info("Decoded JPEG {}x{} texture", w, h);
        outPtr = (uint16_t *)heap_caps_malloc(w * h * sizeof(uint16_t),
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!outPtr)
          return false;
        std::memcpy(outPtr, decoder.get_decoded_data(), w * h * sizeof(uint16_t));
      }
      return true;
    };
    bool loaded = false;

    for (auto &entry : std::filesystem::directory_iterator(models_dir, ec)) {
      auto path = entry.path();
      if (!entry.is_regular_file())
        continue;
      auto ext = path.extension().string();
      if (ext == ".glb") {
        logger.info("Trying to load GLB file {}", path.string());
        MaterialInfo mi;
        modelObjs.push_back(Object());
        loaded = asset::LoadGLB(path.string(), modelObjs[modelObjs.size() - 1], &mi, fileDecoder,
                                bytesDecoder);
        if (loaded) {
          logger.info("Successfully loaded GLB model from {}", path.string());
        } else {
          logger.warn("Failed to load.");
        }
      }
      if (ext == ".obj") {
        logger.info("Trying to load OBJ file {}", path.string());
        MaterialInfo mi;
        modelObjs.push_back(Object());
        loaded = asset::LoadOBJ(path.string(), modelObjs[modelObjs.size() - 1], &mi, fileDecoder);
        if (loaded) {
          logger.info("Successfully loaded OBJ model from {}", path.string());
        } else {
          logger.warn("Failed to load.");
        }
      }
    }
    if (loaded) {
      logger.info("Loaded model from {}", models_dir.string());
      // Replace world with this single object
      objectlist.clear();
      objectlist.emplace_back(modelObjs[object_index]);

      // reset the FPS counter
      frame_count = 0;
      start = esp_timer_get_time();

      bounds = get_bounds();
      // set the size of the axes to be slightly larger than the bounds of the object
      float axis_size = 0.0f;
      if (bounds.is_set()) {
        axis_size = std::max(bounds.maxx - bounds.minx,
                             std::max(bounds.maxy - bounds.miny, bounds.maxz - bounds.minz)) *
                    0.5f;
      } else {
        axis_size = 1.0f; // default size if bounds are not set
      }

      // Add world axes helper for visualization at origin
      Object axes;
      axes.GenerateAxes(axis_size * 1.5f, axis_size * 0.05f); // axes with size and thickness

      objectlist.emplace_back(axes); // add axes for reference

    } else {
      player->Level(1); // load default level if no model found
      objectlist = player->Level().GetObjectList();
    }
  }

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

  // Build perspective projection from FoV, aspect, near, far for row-vector convention
  float fovY = 90.0f * (float)M_PI / 180.0f; // radians
  float aspect = (float)SIZE_X / (float)SIZE_Y;
  float zn = 0.01f;
  float zf = 1000.0f;
  float f = 1.0f / std::tan(fovY * 0.5f);
  perspectiveProjection.SetIdentity();
  // Row-vector projection (x', y', z', w') = (x, y, z, w) * P
  // P maps to NDC: x_ndc = x * f/aspect / w, y_ndc = y * f / w, z_ndc = (zf/(zf-zn)) +
  // (-zn*zf/(zf-zn))/w, w' = z
  //
  // NDC coordinates are in the range [-1, 1] for x and y, and [0, 1] for z
  // after perspective divide
  //
  // We want to invert the NDC to better match screen coordinates, so we invert
  // the x and y axes in the projection matrix
  perspectiveProjection[0][0] = -f / aspect;
  perspectiveProjection[1][1] = -f;
  perspectiveProjection[2][2] = zf / (zf - zn);
  perspectiveProjection[2][3] = 1.0f;                   // w' = z
  perspectiveProjection[3][2] = (-zn * zf) / (zf - zn); // z offset term

  // Viewport transform to pixel coordinates
  // P maps from NDC to pixel coordinates: x_pixel = (x_ndc + 1) * SIZE_X/2, y_pixel = (y_ndc + 1) *
  // SIZE_Y/2
  projectionToPixel.SetIdentity();
  projectionToPixel[0][0] = SIZE_X * 0.5f; // scale x
  projectionToPixel[1][1] = SIZE_Y * 0.5f; // scale y
  projectionToPixel[3][0] = SIZE_X * 0.5f; // translate x
  projectionToPixel[3][1] = SIZE_Y * 0.5f; // translate y

  // make a simple task that prints "Hello World!" every second
  espp::Task task(
      {.callback = [&](auto &m, auto &cv) -> bool {
         uint64_t now = esp_timer_get_time();
         std::lock_guard<std::mutex> lock(object_mutex);
         // Move camera to orbit around the loaded object and look at it
         Point3D target(0, 0, 0);
         float extentX = 0.0f, extentY = 0.0f, extentZ = 0.0f;
         if (bounds.is_set()) {
           // NOTE: we could compute the center of the object, but better to use
           // the model's origin for now, since it may not be centered in the
           // bounds
           //
           // target = Point3D((bounds.minx + bounds.maxx) * 0.5f,
           //                  (bounds.miny + bounds.maxy) * 0.5f,
           //                  (bounds.minz + bounds.maxz) * 0.5f);
           extentX = bounds.maxx; // std::max(bounds.maxx, std::abs(bounds.minx));
           extentY = std::max(bounds.maxy, std::abs(bounds.miny));
           extentZ = bounds.maxz; // std::max(bounds.maxz, std::abs(bounds.minz));
         } else {
           logger.warn(
               "Could not determine world bounds for object, using default camera position");
         }

         // rotate the camera around the target in the XZ plane
         static auto rotation_start = now;
         float t = (now - rotation_start) / 1'000'000.0f;
         float ang = t * 0.5f;
         float radius = std::max(extentX, extentZ);
         float orbitRadius = radius * 1.5f; // Not too close, or it will be slower
         float camX = target.x + std::cos(ang) * orbitRadius;
         float camZ = target.z + std::sin(ang) * orbitRadius;
         float camY = target.y + std::max(extentY * 0.8f, 1.0f);
         Camera &eye = player->Eye();
         auto eyePos = Point3D(camX, camY, camZ);
         // Look from orbit position to the target center using explicit LookAt
         eye.LookAt(eyePos, target, Vector3D(0, 1, 0));

         // we want to have the first object in the world move back and forth
         // along the world x-axis, so apply the transform
         auto new_pos =
             Point3D(std::sin(t * 0.5f) * std::max(bounds.maxx, std::abs(bounds.minx)), 0.0f, 0.0f);
         objectlist[0].SetPosition(new_pos);

         render();

         frame_count++;
         float frame_time = (now - start) / 1'000'000.0f;
         FPS = frame_count / frame_time;
         logger.debug("FPS = {:0.02f}", FPS);
         // we don't want to stop the task, so return false
         return false;
       },
       .task_config = {
           .name = "Render",
           .stack_size_bytes = 4096,
           .priority = 10,
       }});
  task.start();

  // TODO[William]: Setup touch input processing

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

// copy an image data to texture buffer, this updates what is rendered
void IRAM_ATTR render() {
  static int fb_index = 0; // frame buffer index, used to swap between fb0 and fb1
  // swap the frame buffer index
  fb_index = !fb_index;
  // select the frame buffer
  uint16_t *fb_ptr = (fb_index == 0) ? (uint16_t *)fb0 : (uint16_t *)fb1;

  // wait until the video queue is empty
  while (uxQueueMessagesWaiting(video_queue_) > 0) {
    std::this_thread::sleep_for(100us);
  }

  // set the display_buffer to point to dst, so that the engine will use it.
  display_buffer = fb_ptr;

  for (int i = 0; i < SIZE_X * SIZE_Y; i++) {
    display_buffer[i] = BACKGROUND_COLOR;
    z_buffer[i] = DEFAULT_Z_BUFFER;
  }

  worldToCamera = player->Eye().GetWorldToCamera();

  std::vector<Vertex> frameVertices;
  std::vector<uint32_t> frameIndices;
  std::vector<Object::DrawView> drawList;

  for (auto &it : objectlist) {
    // indexed pipeline append
    it.AppendDrawItems(worldToCamera, perspectiveProjection, projectionToPixel, frameVertices,
                       frameIndices, drawList);
  }

  logger.debug("Rendering {} indexed draws ({} tris)", drawList.size(), frameIndices.size() / 3);

  // Indexed pipeline rasterization
  if (!drawList.empty()) {
    for (const auto &d : drawList) {
      const size_t end = d.baseIndex + d.indexCount;
      for (size_t i = d.baseIndex; i + 2 < end; i += 3) {
        const uint32_t i0 = d.baseVertex + frameIndices[i + 0];
        const uint32_t i1 = d.baseVertex + frameIndices[i + 1];
        const uint32_t i2 = d.baseVertex + frameIndices[i + 2];
        const Vertex &a = frameVertices[i0];
        const Vertex &b = frameVertices[i1];
        const Vertex &c = frameVertices[i2];
        RasterizeTriangle(a, b, c, d.rType, d.texture, d.texwidth, d.texheight, d.r, d.g, d.b);
      }
    }
  }

  // push the frame to the video task
  push_frame(fb_ptr);
}

/////////////////////////////
// Video Related Functions
/////////////////////////////

bool initialize_video() {
  if (video_queue_ || video_task_) {
    return false;
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

bool IRAM_ATTR video_task_callback(std::mutex &m, std::condition_variable &cv,
                                   bool &task_notified) {
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
      vram_index = !vram_index;
      Pixel *_buf = (vram_index == 0) ? (Pixel *)vram0 : (Pixel *)vram1;
      int num_lines = std::min<int>(num_lines_to_write, lcd_height - y);
      // memset the buffer to 0
      memset(_buf, 0, lcd_width * num_lines * sizeof(Pixel));
      hw.write_lcd_lines(x_offset, y + y_offset, x_offset + lcd_width - 1,
                         y + y_offset + num_lines - 1, (uint8_t *)&_buf[0], 0);
    }

    // now return
    return false;
  }

  for (int y = 0; y < lcd_height; y += num_lines_to_write) {
    vram_index = !vram_index;
    Pixel *_buf = (vram_index == 0) ? (Pixel *)vram0 : (Pixel *)vram1;
    int num_lines = std::min<int>(num_lines_to_write, lcd_height - y);
    const uint16_t *_frame = (const uint16_t *)_frame_ptr;
    for (int i = 0; i < num_lines; i++) {
      // write two pixels (32 bits) at a time because it's faster
      for (int j = 0; j < lcd_width; j += 2) {
        const uint32_t *src = (const uint32_t *)&_frame[(y + i) * lcd_width + j];
        uint32_t *dst = (uint32_t *)&_buf[i * lcd_width + j];
        dst[0] = src[0]; // copy two pixels (32 bits) at a time
      }
    }
    hw.write_lcd_lines(x_offset, y + y_offset, x_offset + lcd_width - 1,
                       y + y_offset + num_lines - 1, (uint8_t *)&_buf[0], 0);
  }

  return false;
}
