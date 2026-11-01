#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <lvgl.h>
#include <esp_heap_caps.h>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstring>
#include <new>
#include <string>

// Set to 0 on the vehicle to enable Bluetooth and ELM327 polling.
#ifndef SIMULATION_MODE
#define SIMULATION_MODE 0
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <esp32s3/rom/cache.h>

// Elecrow CrowPanel 7-inch RGB panel pinout and timings.
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_RGB _panel;
  lgfx::Bus_RGB _bus;

 public:
  LGFX() {
    auto bus_config = _bus.config();
    bus_config.panel = &_panel;
    bus_config.pin_d0 = GPIO_NUM_15;
    bus_config.pin_d1 = GPIO_NUM_7;
    bus_config.pin_d2 = GPIO_NUM_6;
    bus_config.pin_d3 = GPIO_NUM_5;
    bus_config.pin_d4 = GPIO_NUM_4;
    bus_config.pin_d5 = GPIO_NUM_9;
    bus_config.pin_d6 = GPIO_NUM_46;
    bus_config.pin_d7 = GPIO_NUM_3;
    bus_config.pin_d8 = GPIO_NUM_8;
    bus_config.pin_d9 = GPIO_NUM_16;
    bus_config.pin_d10 = GPIO_NUM_1;
    bus_config.pin_d11 = GPIO_NUM_14;
    bus_config.pin_d12 = GPIO_NUM_21;
    bus_config.pin_d13 = GPIO_NUM_47;
    bus_config.pin_d14 = GPIO_NUM_48;
    bus_config.pin_d15 = GPIO_NUM_45;
    bus_config.pin_henable = GPIO_NUM_41;
    bus_config.pin_vsync = GPIO_NUM_40;
    bus_config.pin_hsync = GPIO_NUM_39;
    bus_config.pin_pclk = GPIO_NUM_0;
    bus_config.freq_write = 15000000;
    bus_config.hsync_polarity = 0;
    bus_config.hsync_front_porch = 40;
    bus_config.hsync_pulse_width = 48;
    bus_config.hsync_back_porch = 40;
    bus_config.vsync_polarity = 0;
    bus_config.vsync_front_porch = 1;
    bus_config.vsync_pulse_width = 31;
    bus_config.vsync_back_porch = 13;
    bus_config.pclk_active_neg = 1;
    bus_config.de_idle_high = 0;
    bus_config.pclk_idle_high = 0;
    _bus.config(bus_config);

    auto panel_config = _panel.config();
    panel_config.memory_width = 800;
    panel_config.memory_height = 480;
    panel_config.panel_width = 800;
    panel_config.panel_height = 480;
    panel_config.offset_x = 0;
    panel_config.offset_y = 0;
    _panel.config(panel_config);
    _panel.setBus(&_bus);
    setPanel(&_panel);
  }
};
#else
// SPI ILI9341 and pins match the checked-in Wokwi diagram.
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;
  lgfx::Bus_SPI _bus;

 public:
  LGFX() {
    auto bus_config = _bus.config();
    bus_config.spi_host = VSPI_HOST;
    bus_config.spi_mode = 0;
    bus_config.freq_write = 40000000;
    bus_config.freq_read = 16000000;
    bus_config.spi_3wire = true;
    bus_config.use_lock = true;
    bus_config.dma_channel = 1;
    bus_config.pin_sclk = 18;
    bus_config.pin_mosi = 23;
    bus_config.pin_miso = -1;
    bus_config.pin_dc = 2;
    _bus.config(bus_config);
    _panel.setBus(&_bus);

    auto panel_config = _panel.config();
    panel_config.pin_cs = 15;
    panel_config.pin_rst = -1;
    panel_config.pin_busy = -1;
    panel_config.memory_width = 240;
    panel_config.memory_height = 320;
    panel_config.panel_width = 240;
    panel_config.panel_height = 320;
    panel_config.offset_x = 0;
    panel_config.offset_y = 0;
    panel_config.offset_rotation = 0;
    panel_config.dummy_read_pixel = 8;
    panel_config.dummy_read_bits = 1;
    panel_config.readable = false;
    panel_config.invert = false;
    panel_config.rgb_order = false;
    panel_config.dlen_16bit = false;
    _panel.config(panel_config);
    setPanel(&_panel);
  }
};
#endif

static LGFX display;
static lv_disp_draw_buf_t draw_buffer;
static lv_color_t* draw_buffer_pixels = nullptr;
static lv_disp_drv_t lv_display_driver;
static bool display_ready = false;

static lv_obj_t* speed_label = nullptr;
static lv_obj_t* rpm_label = nullptr;
static lv_obj_t* oil_label = nullptr;
static lv_obj_t* rpm_bar = nullptr;
static lv_obj_t* oil_title = nullptr;
static lv_meter_indicator_t* coolant_needle = nullptr;
static lv_meter_indicator_t* load_needle = nullptr;
static lv_obj_t* coolant_meter = nullptr;
static lv_obj_t* load_meter = nullptr;
static lv_obj_t* coolant_title = nullptr;
static lv_obj_t* coolant_label = nullptr;
static lv_obj_t* load_title = nullptr;
static lv_obj_t* load_label = nullptr;
static lv_obj_t* status_label = nullptr;

static uint32_t last_lv_tick_ms = 0;

#if !SIMULATION_MODE
#include <NimBLEDevice.h>
#include "ELMduino.h"

enum class BleUiState : uint8_t {
  Starting,
  Scanning,
  Connecting,
  Initializing,
  Connected,
  Error
};

enum class ObdPidState : uint8_t { Rpm, Speed, Coolant, Load, OilTemp, ReadDTC };
static ObdPidState obd_state = ObdPidState::Rpm;

static std::atomic<BleUiState> ble_ui_state{BleUiState::Starting};
static std::atomic<bool> elm_ready{false};
static std::atomic<bool> link_dropped{false};
static std::atomic<bool> request_dtc_scan{false};

static NimBLEScan* ble_scan = nullptr;
static NimBLEClient* ble_client = nullptr;
static NimBLERemoteCharacteristic* ble_io_characteristic = nullptr;
static ELM327* myELM327 = nullptr;

static SemaphoreHandle_t gatt_write_mutex = nullptr;
static SemaphoreHandle_t elm_mutex = nullptr;
static portMUX_TYPE candidate_mux = portMUX_INITIALIZER_UNLOCKED;

static NimBLEAddress candidate_address;
static bool candidate_ready = false;

class BleElmStream final : public Stream {
  public:
   int available() override {
     portENTER_CRITICAL(&_buffer_mux);
     const int count = static_cast<int>((_rx_head + kBufferSize - _rx_tail) % kBufferSize);
     portEXIT_CRITICAL(&_buffer_mux);
     return count;
   }
   int read() override {
     portENTER_CRITICAL(&_buffer_mux);
     if (_rx_head == _rx_tail) {
       portEXIT_CRITICAL(&_buffer_mux);
       return -1;
     }
     const int value = _rx_buffer[_rx_tail];
     _rx_tail = (_rx_tail + 1) % kBufferSize;
     portEXIT_CRITICAL(&_buffer_mux);
     return value;
   }
   int peek() override {
     portENTER_CRITICAL(&_buffer_mux);
     const int value = _rx_head == _rx_tail ? -1 : _rx_buffer[_rx_tail];
     portEXIT_CRITICAL(&_buffer_mux);
     return value;
   }
   void flush() override {}
   size_t write(uint8_t value) override { return write(&value, 1); }
   size_t write(const uint8_t* data, size_t length) override {
     if (data == nullptr) return 0;
     portENTER_CRITICAL(&_buffer_mux);
     size_t written = 0;
     while (written < length) {
       const size_t next = (_tx_head + 1) % kBufferSize;
       if (next == _tx_tail) break;
       _tx_buffer[_tx_head] = data[written++];
       _tx_head = next;
     }
     portEXIT_CRITICAL(&_buffer_mux);
     return written;
   }
   void pushRx(const uint8_t* data, size_t length) {
     portENTER_CRITICAL(&_buffer_mux);
     for (size_t i = 0; i < length; ++i) {
       const size_t next = (_rx_head + 1) % kBufferSize;
       if (next == _rx_tail) break;
       _rx_buffer[_rx_head] = data[i];
       _rx_head = next;
     }
     portEXIT_CRITICAL(&_buffer_mux);
   }
   void clear() {
     portENTER_CRITICAL(&_buffer_mux);
     _rx_head = _rx_tail = 0;
     _tx_head = _tx_tail = 0;
     portEXIT_CRITICAL(&_buffer_mux);
   }
   void setTransmitCharacteristic(NimBLERemoteCharacteristic* characteristic) {
     _tx_characteristic = characteristic;
   }
   void pumpTx() {
     if (gatt_write_mutex == nullptr || xSemaphoreTake(gatt_write_mutex, 0) != pdTRUE) return;
     NimBLERemoteCharacteristic* characteristic = _tx_characteristic;
     if (characteristic != nullptr) {
       uint8_t packet[20];
       const size_t length = peekTx(packet, sizeof(packet));
       if (length > 0) {
         const bool use_response = characteristic->canWrite();
         if (characteristic->writeValue(packet, length, use_response)) {
           consumeTx(length);
         }
       }
     }
     xSemaphoreGive(gatt_write_mutex);
   }

  private:
   static constexpr size_t kBufferSize = 512;
   uint8_t _rx_buffer[kBufferSize]{};
   uint8_t _tx_buffer[kBufferSize]{};
   size_t _rx_head = 0;
   size_t _rx_tail = 0;
   size_t _tx_head = 0;
   size_t _tx_tail = 0;
   portMUX_TYPE _buffer_mux = portMUX_INITIALIZER_UNLOCKED;
   NimBLERemoteCharacteristic* _tx_characteristic = nullptr;

   size_t peekTx(uint8_t* destination, size_t capacity) {
     portENTER_CRITICAL(&_buffer_mux);
     size_t count = 0;
     size_t index = _tx_tail;
     while (index != _tx_head && count < capacity) {
       destination[count++] = _tx_buffer[index];
       index = (index + 1) % kBufferSize;
     }
     portEXIT_CRITICAL(&_buffer_mux);
     return count;
   }
   void consumeTx(size_t length) {
     portENTER_CRITICAL(&_buffer_mux);
     while (length-- > 0 && _tx_tail != _tx_head) {
       _tx_tail = (_tx_tail + 1) % kBufferSize;
     }
     portEXIT_CRITICAL(&_buffer_mux);
   }
};

static BleElmStream ble_elm_stream;

class HudAdvertisedDeviceCallbacks final : public NimBLEAdvertisedDeviceCallbacks {
 public:
  void onResult(NimBLEAdvertisedDevice* advertised_device) override {
    if (!advertised_device->haveName()) return;
    std::string name = advertised_device->getName();
    std::transform(name.begin(), name.end(), name.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
    if (name.find("veepeak") == std::string::npos && name.find("obd") == std::string::npos) return;

    NimBLEAddress address = advertised_device->getAddress();
    portENTER_CRITICAL(&candidate_mux);
    if (!candidate_ready) {
      candidate_address = address;
      candidate_ready = true;
    }
    portEXIT_CRITICAL(&candidate_mux);
    NimBLEDevice::getScan()->stop();
  }
};

class HudBleClientCallbacks final : public NimBLEClientCallbacks {
 public:
  void onConnect(NimBLEClient*) override {}
  void onDisconnect(NimBLEClient*) override {
    link_dropped.store(true);
    elm_ready.store(false);
  }
};

static HudAdvertisedDeviceCallbacks advertised_device_callbacks;
static HudBleClientCallbacks ble_client_callbacks;
#else
static uint32_t last_simulation_update_ms = 0;
static float simulation_phase = 0.0f;
#endif

static void display_flush(lv_disp_drv_t* driver, const lv_area_t* area,
                          lv_color_t* colors) {
  const uint32_t width = static_cast<uint32_t>(area->x2 - area->x1 + 1);
  const uint32_t height = static_cast<uint32_t>(area->y2 - area->y1 + 1);
  display.startWrite();
  display.pushImage(area->x1, area->y1, width, height,
                    reinterpret_cast<const uint16_t*>(colors));
  display.endWrite();
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  // The RGB panel scans the PSRAM framebuffer by DMA, so CPU-cached writes must reach memory.
  Cache_WriteBack_All();
#endif
  lv_disp_flush_ready(driver);
}

static bool initialize_lvgl_display() {
  display.init();
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  constexpr uint8_t backlight_pin = 2;
  constexpr uint8_t backlight_channel = 0;
  ledcSetup(backlight_channel, 5000, 8);
  ledcAttachPin(backlight_pin, backlight_channel);
  ledcWrite(backlight_channel, 166);
  display.setSwapBytes(true);
  display.setRotation(0);
#else
  display.setRotation(1);
#endif
  display.fillScreen(TFT_BLACK);

  lv_init();

  const uint32_t display_width = display.width();
  const uint32_t display_height = display.height();
  uint32_t buffer_lines = display_height < 32 ? display_height : 32;

  while (buffer_lines > 0 && draw_buffer_pixels == nullptr) {
  const size_t buffer_bytes = display_width * buffer_lines * sizeof(lv_color_t);
  
  // 1st Priority: Try to fit the draw buffer in ultra-fast internal SRAM
  draw_buffer_pixels = static_cast<lv_color_t*>(
      heap_caps_malloc(buffer_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
      
  // 2nd Priority: Fallback to external PSRAM only if internal is full
  if (draw_buffer_pixels == nullptr) {
    draw_buffer_pixels = static_cast<lv_color_t*>(
        heap_caps_malloc(buffer_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  }
  
  if (draw_buffer_pixels == nullptr) {
    --buffer_lines;
  }
}
  if (draw_buffer_pixels == nullptr) {
    Serial.println("Unable to allocate LVGL draw buffer");
    return false;
  }

  lv_disp_draw_buf_init(&draw_buffer, draw_buffer_pixels, nullptr,
                        display_width * buffer_lines);
  lv_disp_drv_init(&lv_display_driver);
  lv_display_driver.hor_res = display_width;
  lv_display_driver.ver_res = display_height;
  lv_display_driver.flush_cb = display_flush;
  lv_display_driver.draw_buf = &draw_buffer;
  lv_disp_drv_register(&lv_display_driver);
  return true;
}

constexpr float kRpmMax = 8000.0f;
constexpr int kRpmSegments = 48;
constexpr float kPi = 3.14159265f;

// Centerline of the bent RPM bar: vertical run, quarter-circle bend, horizontal run.
struct RpmBarGeometry {
  lv_coord_t thickness;
  lv_coord_t start_x;
  lv_coord_t bottom_y;
  lv_coord_t radius;
  lv_coord_t bend_x;
  lv_coord_t bend_y;
  float vertical_len;
  float arc_len;
  float horizontal_len;
  float total_len;
};

static RpmBarGeometry rpm_geometry{};
static int rpm_lit_segments = 0;

static lv_color_t rpm_segment_color(float fraction) {
  const lv_color_t blue = lv_color_hex(0x2A6BFF);
  const lv_color_t green = lv_color_hex(0x19D45A);
  const lv_color_t yellow = lv_color_hex(0xFFE000);
  const lv_color_t orange = lv_color_hex(0xFF8A1E);
  const lv_color_t red = lv_color_hex(0xF03030);
  if (fraction >= 0.875f) return red;
  if (fraction < 0.35f) return lv_color_mix(green, blue, static_cast<uint8_t>(fraction / 0.35f * 255.0f));
  if (fraction < 0.70f) return lv_color_mix(yellow, green, static_cast<uint8_t>((fraction - 0.35f) / 0.35f * 255.0f));
  return lv_color_mix(orange, yellow, static_cast<uint8_t>((fraction - 0.70f) / 0.175f * 255.0f));
}

static void rpm_bar_draw(lv_event_t* event) {
  lv_draw_ctx_t* draw_ctx = lv_event_get_draw_ctx(event);
  lv_obj_t* obj = lv_event_get_target(event);
  const RpmBarGeometry& g = rpm_geometry;
  const lv_coord_t origin_x = obj->coords.x1;
  const lv_coord_t origin_y = obj->coords.y1;
  const lv_point_t bend_center = {static_cast<lv_coord_t>(origin_x + g.bend_x),
                                  static_cast<lv_coord_t>(origin_y + g.bend_y)};

  lv_draw_rect_dsc_t rect_dsc;
  lv_draw_rect_dsc_init(&rect_dsc);
  rect_dsc.radius = 0;
  rect_dsc.border_width = 0;
  lv_draw_arc_dsc_t arc_dsc;
  lv_draw_arc_dsc_init(&arc_dsc);
  arc_dsc.width = g.thickness;

  const float segment_len = g.total_len / kRpmSegments;
  const float arc_start = g.vertical_len;
  const float horizontal_start = g.vertical_len + g.arc_len;
  const lv_color_t background = lv_color_hex(0x080D12);

  for (int i = 0; i < kRpmSegments; ++i) {
    const float a = i * segment_len;
    const float b = a + segment_len * 0.8f;
    const lv_color_t color = rpm_segment_color((i + 0.5f) / kRpmSegments);
    const lv_color_t shown = i < rpm_lit_segments ? color : lv_color_mix(color, background, 40);

    const float v0 = a;
    const float v1 = b < arc_start ? b : arc_start;
    if (v1 > v0) {
      rect_dsc.bg_color = shown;
      lv_area_t area;
      area.x1 = origin_x + g.start_x - g.thickness / 2;
      area.x2 = area.x1 + g.thickness - 1;
      area.y1 = origin_y + g.bottom_y - static_cast<lv_coord_t>(v1);
      area.y2 = origin_y + g.bottom_y - static_cast<lv_coord_t>(v0);
      lv_draw_rect(draw_ctx, &rect_dsc, &area);
    }

    const float a0 = a > arc_start ? a : arc_start;
    const float a1 = b < horizontal_start ? b : horizontal_start;
    if (a1 > a0) {
      arc_dsc.color = shown;
      const float degrees_per_len = 180.0f / kPi / g.radius;
      lv_draw_arc(draw_ctx, &arc_dsc, &bend_center,
                  static_cast<uint16_t>(g.radius + g.thickness / 2),
                  static_cast<uint16_t>(180.0f + (a0 - arc_start) * degrees_per_len),
                  static_cast<uint16_t>(180.0f + (a1 - arc_start) * degrees_per_len));
    }

    const float h0 = a > horizontal_start ? a : horizontal_start;
    if (b > h0) {
      rect_dsc.bg_color = shown;
      lv_area_t area;
      area.x1 = origin_x + g.bend_x + static_cast<lv_coord_t>(h0 - horizontal_start);
      area.x2 = origin_x + g.bend_x + static_cast<lv_coord_t>(b - horizontal_start);
      area.y1 = origin_y + g.bend_y - g.radius - g.thickness / 2;
      area.y2 = area.y1 + g.thickness - 1;
      lv_draw_rect(draw_ctx, &rect_dsc, &area);
    }
  }
}

static lv_obj_t* create_semicircle_gauge(lv_obj_t* screen, lv_coord_t x, lv_coord_t center_y,
                                         lv_coord_t diameter, bool wide_display,
                                         int32_t min_value, int32_t max_value,
                                         int32_t warn_value, int32_t danger_value,
                                         lv_meter_indicator_t** needle) {
  lv_obj_t* meter = lv_meter_create(screen);
  lv_obj_set_size(meter, diameter, diameter);
  lv_obj_set_style_bg_opa(meter, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_opa(meter, LV_OPA_TRANSP, 0);
  lv_obj_set_style_pad_all(meter, 4, 0);
  lv_obj_set_pos(meter, x, center_y - diameter / 2);

  const lv_coord_t band_width = wide_display ? 18 : 8;
  lv_meter_scale_t* scale = lv_meter_add_scale(meter);
  lv_meter_set_scale_range(meter, scale, min_value, max_value, 180, 180);
  lv_meter_set_scale_ticks(meter, scale, 21, 2, band_width, lv_color_hex(0x080D12));

  lv_meter_indicator_t* green = lv_meter_add_arc(meter, scale, band_width, lv_color_hex(0x19D45A), 0);
  lv_meter_set_indicator_start_value(meter, green, min_value);
  lv_meter_set_indicator_end_value(meter, green, warn_value);
  lv_meter_indicator_t* orange = lv_meter_add_arc(meter, scale, band_width, lv_color_hex(0xFF8A1E), 0);
  lv_meter_set_indicator_start_value(meter, orange, warn_value);
  lv_meter_set_indicator_end_value(meter, orange, danger_value);
  lv_meter_indicator_t* red = lv_meter_add_arc(meter, scale, band_width, lv_color_hex(0xF03030), 0);
  lv_meter_set_indicator_start_value(meter, red, danger_value);
  lv_meter_set_indicator_end_value(meter, red, max_value);

  *needle = lv_meter_add_needle_line(meter, scale, wide_display ? 4 : 2,
                                     lv_color_hex(0xF2F6F7), -band_width - 6);
  return meter;
}

static void build_hud_ui() {
  lv_obj_t* screen = lv_scr_act();
  const lv_coord_t screen_width = lv_disp_get_hor_res(nullptr);
  const lv_coord_t screen_height = lv_disp_get_ver_res(nullptr);
  const bool wide_display = screen_width >= 640;
  const lv_color_t white = lv_color_hex(0xF2F6F7);
  const lv_color_t green = lv_color_hex(0x5CE65C);
  const lv_font_t* small_font = wide_display ? &lv_font_montserrat_16 : &lv_font_montserrat_12;
  const lv_font_t* medium_font = wide_display ? &lv_font_montserrat_24 : &lv_font_montserrat_14;

  lv_obj_set_style_bg_color(screen, lv_color_hex(0x080D12), 0);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

  RpmBarGeometry& g = rpm_geometry;
  g.thickness = static_cast<lv_coord_t>(screen_width * 0.043f);
  g.radius = static_cast<lv_coord_t>(screen_width * 0.15f);
  g.start_x = static_cast<lv_coord_t>(screen_width * 0.04f) + g.thickness / 2;
  g.bottom_y = static_cast<lv_coord_t>(screen_height * 0.46f);
  g.bend_x = g.start_x + g.radius;
  g.bend_y = static_cast<lv_coord_t>(screen_height * 0.04f) + g.thickness / 2 + g.radius;
  g.vertical_len = static_cast<float>(g.bottom_y - g.bend_y);
  g.arc_len = kPi * g.radius / 2.0f;
  g.horizontal_len = static_cast<float>(static_cast<lv_coord_t>(screen_width * 0.96f) - g.bend_x);
  g.total_len = g.vertical_len + g.arc_len + g.horizontal_len;

  rpm_bar = lv_obj_create(screen);
  lv_obj_set_size(rpm_bar, screen_width, static_cast<lv_coord_t>(screen_height * 0.5f));
  lv_obj_set_pos(rpm_bar, 0, 0);
  lv_obj_set_style_bg_opa(rpm_bar, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(rpm_bar, 0, 0);
  lv_obj_clear_flag(rpm_bar, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(rpm_bar, rpm_bar_draw, LV_EVENT_DRAW_MAIN, nullptr);

  // Scale numbers 0-8 sit on the inner side of the bar.
  const lv_coord_t label_offset = g.thickness / 2 + (wide_display ? 16 : 8);
  for (int tick = 0; tick <= 8; ++tick) {
    const float s = g.total_len * tick / 8.0f;
    float x;
    float y;
    if (s <= g.vertical_len) {
      x = g.start_x + label_offset;
      y = g.bottom_y - s;
    } else if (s <= g.vertical_len + g.arc_len) {
      const float angle = kPi + (s - g.vertical_len) / g.radius;
      x = g.bend_x + (g.radius - label_offset) * cosf(angle);
      y = g.bend_y + (g.radius - label_offset) * sinf(angle);
    } else {
      x = g.bend_x + (s - g.vertical_len - g.arc_len);
      y = g.bend_y - g.radius + label_offset;
    }
    lv_obj_t* tick_label = lv_label_create(screen);
    lv_obj_set_style_text_font(tick_label, small_font, 0);
    lv_obj_set_style_text_color(tick_label, white, 0);
    lv_label_set_text_fmt(tick_label, "%d", tick);
    lv_obj_align(tick_label, LV_ALIGN_TOP_LEFT, static_cast<lv_coord_t>(x) - (wide_display ? 6 : 3),
                 static_cast<lv_coord_t>(y) - (wide_display ? 10 : 6));
  }

  rpm_label = lv_label_create(screen);
  lv_obj_set_style_text_font(rpm_label, medium_font, 0);
  lv_obj_set_style_text_color(rpm_label, white, 0);
  lv_label_set_text(rpm_label, "RPM  0");
  lv_obj_align(rpm_label, LV_ALIGN_TOP_LEFT, static_cast<lv_coord_t>(screen_width * 0.16f),
               static_cast<lv_coord_t>(screen_height * 0.38f));

  speed_label = lv_label_create(screen);
  lv_obj_set_style_text_font(speed_label, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(speed_label, white, 0);
  lv_label_set_text(speed_label, "0");
  lv_obj_align(speed_label, LV_ALIGN_CENTER, 0, static_cast<lv_coord_t>(-screen_height * 0.02f));

  lv_obj_t* speed_units = lv_label_create(screen);
  lv_obj_set_style_text_font(speed_units, medium_font, 0);
  lv_obj_set_style_text_color(speed_units, white, 0);
  lv_label_set_text(speed_units, "MPH");
  lv_obj_align(speed_units, LV_ALIGN_CENTER, 0, static_cast<lv_coord_t>(screen_height * 0.12f));

  oil_title = lv_label_create(screen);
  lv_obj_set_style_text_font(oil_title, small_font, 0);
  lv_obj_set_style_text_color(oil_title, white, 0);
  lv_label_set_text(oil_title, "OIL F");
  lv_obj_align(oil_title, LV_ALIGN_TOP_RIGHT, -static_cast<lv_coord_t>(screen_width * 0.04f),
               static_cast<lv_coord_t>(screen_height * 0.30f));

  oil_label = lv_label_create(screen);
  lv_obj_set_style_text_font(oil_label, wide_display ? &lv_font_montserrat_32 : &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(oil_label, green, 0);
  lv_label_set_text(oil_label, "---");
  lv_obj_align_to(oil_label, oil_title, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 2);

  status_label = lv_label_create(screen);
  lv_obj_set_style_text_font(status_label, small_font, 0);
  lv_obj_set_style_text_color(status_label, lv_color_hex(0x83939B), 0);
  lv_label_set_text(status_label, SIMULATION_MODE ? "SIMULATION" : "BLE SCANNING");
  lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID, 0, -4);

  const lv_coord_t gauge_diameter = static_cast<lv_coord_t>(screen_width * 0.30f);
  const lv_coord_t gauge_margin = static_cast<lv_coord_t>(screen_width * 0.05f);
  const lv_coord_t gauge_center_y = static_cast<lv_coord_t>(screen_height * 0.80f);

  coolant_meter = create_semicircle_gauge(screen, gauge_margin, gauge_center_y, gauge_diameter,
                                          wide_display, 100, 260, 210, 230, &coolant_needle);
  load_meter = create_semicircle_gauge(screen, screen_width - gauge_margin - gauge_diameter,
                                       gauge_center_y, gauge_diameter, wide_display, 0, 100, 60, 85,
                                       &load_needle);

  coolant_label = lv_label_create(screen);
  lv_obj_set_width(coolant_label, gauge_diameter);
  lv_obj_set_style_text_align(coolant_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(coolant_label, medium_font, 0);
  lv_obj_set_style_text_color(coolant_label, white, 0);
  lv_label_set_text(coolant_label, "--");
  lv_obj_align_to(coolant_label, coolant_meter, LV_ALIGN_CENTER, 0, wide_display ? 24 : 12);

  coolant_title = lv_label_create(screen);
  lv_obj_set_width(coolant_title, gauge_diameter);
  lv_obj_set_style_text_align(coolant_title, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(coolant_title, small_font, 0);
  lv_obj_set_style_text_color(coolant_title, white, 0);
  lv_label_set_text(coolant_title, "COOLANT F");
  lv_obj_align_to(coolant_title, coolant_meter, LV_ALIGN_CENTER, 0, wide_display ? 56 : 28);

  load_label = lv_label_create(screen);
  lv_obj_set_width(load_label, gauge_diameter);
  lv_obj_set_style_text_align(load_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(load_label, medium_font, 0);
  lv_obj_set_style_text_color(load_label, white, 0);
  lv_label_set_text(load_label, "-- %");
  lv_obj_align_to(load_label, load_meter, LV_ALIGN_CENTER, 0, wide_display ? 24 : 12);

  load_title = lv_label_create(screen);
  lv_obj_set_width(load_title, gauge_diameter);
  lv_obj_set_style_text_align(load_title, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(load_title, small_font, 0);
  lv_obj_set_style_text_color(load_title, white, 0);
  lv_label_set_text(load_title, "ENGINE LOAD");
  lv_obj_align_to(load_title, load_meter, LV_ALIGN_CENTER, 0, wide_display ? 56 : 28);
}

static void update_rpm(float rpm) {
  const float bounded_rpm = rpm < 0.0f ? 0.0f : rpm > kRpmMax ? kRpmMax : rpm;
  const int lit_segments = static_cast<int>(bounded_rpm / kRpmMax * kRpmSegments + 0.5f);
  if (lit_segments != rpm_lit_segments) {
    rpm_lit_segments = lit_segments;
    lv_obj_invalidate(rpm_bar);
  }
  lv_label_set_text_fmt(rpm_label, "RPM  %ld", static_cast<long>(bounded_rpm));
}

static void update_speed(float mph) {
  lv_label_set_text_fmt(speed_label, "%ld", lroundf(mph < 0.0f ? 0.0f : mph));
}

static void update_coolant(float temp_c) {
  const float temp_f = temp_c * 9.0f / 5.0f + 32.0f;
  const float bounded_temp_f = temp_f < 100.0f ? 100.0f : temp_f > 260.0f ? 260.0f : temp_f;
  lv_meter_set_indicator_value(coolant_meter, coolant_needle,
                               static_cast<int32_t>(bounded_temp_f));
  lv_label_set_text_fmt(coolant_label, "%ld", lroundf(temp_f));
}

static void update_oil_temp(float temp_c) {
  const float temp_f = temp_c * 9.0f / 5.0f + 32.0f;
  lv_label_set_text_fmt(oil_label, "%ld", lroundf(temp_f));
}

static void update_load(float load_percent) {
  const float bounded_load = load_percent < 0.0f ? 0.0f : load_percent > 100.0f ? 100.0f : load_percent;
  lv_meter_set_indicator_value(load_meter, load_needle, static_cast<int32_t>(bounded_load));
  lv_label_set_text_fmt(load_label, "%ld %%", lroundf(bounded_load));
}

#if SIMULATION_MODE
static void update_simulated_telemetry(uint32_t now_ms) {
  constexpr uint32_t update_interval_ms = 100;
  if (now_ms - last_simulation_update_ms < update_interval_ms) {
    return;
  }

  const uint32_t elapsed_ms = now_ms - last_simulation_update_ms;
  last_simulation_update_ms = now_ms;
  simulation_phase += static_cast<float>(elapsed_ms) / 1000.0f;

  const float rpm = 3000.0f + 2700.0f * sinf(simulation_phase * 0.72f);
  const float mph = 48.0f + 42.0f * sinf(simulation_phase * 0.23f);
  const float coolant_c = 91.0f + 4.0f * sinf(simulation_phase * 0.08f);
  const float load_percent = 42.0f + 35.0f * sinf(simulation_phase * 0.51f);
  const float oil_temp_c = 103.0f + 6.0f * sinf(simulation_phase * 0.07f);

  update_rpm(rpm);
  update_speed(mph);
  update_coolant(coolant_c);
  update_oil_temp(oil_temp_c);
  update_load(load_percent);
}
#else
static void ble_notify_callback(BLERemoteCharacteristic*, uint8_t* data,
                                size_t length, bool) {
  ble_elm_stream.pushRx(data, length);
}

static bool take_candidate(NimBLEAddress* address) {
  portENTER_CRITICAL(&candidate_mux);
  const bool ready = candidate_ready;
  if (ready) {
    *address = candidate_address;
    candidate_ready = false;
  }
  portEXIT_CRITICAL(&candidate_mux);
  return ready;
}

struct BleGattTransport {
  NimBLERemoteCharacteristic* receive = nullptr;
  NimBLERemoteCharacteristic* transmit = nullptr;
};

static BleGattTransport find_obd_transport(NimBLEClient* client) {
  std::vector<NimBLERemoteService*>* services = client->getServices(true);
  if (services == nullptr) return {};

  BleGattTransport preferred;
  BleGattTransport fallback;
  const NimBLEUUID expected_service("FFE0");
  const NimBLEUUID expected_characteristic("FFE1");
  const NimBLEUUID veepeak_service("FFF0");

  for (NimBLERemoteService* service : *services) {
    std::vector<NimBLERemoteCharacteristic*>* characteristics =
      service->getCharacteristics(true);
    if (characteristics == nullptr) continue;

    const NimBLEUUID service_uuid = service->getUUID();
    const std::string service_text = service_uuid.toString();
    BleGattTransport service_transport;
    NimBLERemoteCharacteristic* standard_characteristic = nullptr;
    for (NimBLERemoteCharacteristic* characteristic : *characteristics) {
      const NimBLEUUID characteristic_uuid = characteristic->getUUID();
      const std::string characteristic_text = characteristic_uuid.toString();
      const bool writable = characteristic->canWrite() ||
                            characteristic->canWriteNoResponse();
      const bool notifiable = characteristic->canNotify();
      Serial.printf("BLE GATT service=%s char=%s write=%d notify=%d\n",
                    service_text.c_str(), characteristic_text.c_str(),
                    writable, notifiable);

      if (service_uuid == expected_service &&
          characteristic_uuid == expected_characteristic) {
        standard_characteristic = characteristic;
      }
      if (writable && service_transport.transmit == nullptr)
        service_transport.transmit = characteristic;
      if (notifiable && service_transport.receive == nullptr)
        service_transport.receive = characteristic;
    }

    if (standard_characteristic != nullptr &&
      (standard_characteristic->canWrite() ||
       standard_characteristic->canWriteNoResponse()) &&
        standard_characteristic->canNotify()) {
          preferred.receive = standard_characteristic;
          preferred.transmit = standard_characteristic;
    } else if (service_uuid == veepeak_service &&
               service_transport.receive != nullptr &&
               service_transport.transmit != nullptr) {
      preferred = service_transport;
    } else if (fallback.receive == nullptr &&
               service_transport.receive != nullptr &&
               service_transport.transmit != nullptr) {
      fallback = service_transport;
    }
  }

  return preferred.receive != nullptr && preferred.transmit != nullptr
             ? preferred
             : fallback;
}

static void ble_tx_task(void*) {
  for (;;) {
    ble_elm_stream.pumpTx();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void ble_connection_task(void*) {
  char service_address[18];
  NimBLEAddress address;

  for (;;) {
    ble_ui_state.store(BleUiState::Scanning);
    portENTER_CRITICAL(&candidate_mux);
    candidate_ready = false;
    portEXIT_CRITICAL(&candidate_mux);
    ble_scan->clearResults();

    if (!ble_scan->start(0, nullptr, false)) {
      ble_ui_state.store(BleUiState::Error);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    while (!take_candidate(&address)) {
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    const NimBLEAddress peer_address = address;
    const std::string address_text = peer_address.toString();
    strlcpy(service_address, address_text.c_str(), sizeof(service_address));
    Serial.printf("BLE adapter found: %s\n", service_address);
    ble_ui_state.store(BleUiState::Connecting);

    NimBLEClient* client = NimBLEDevice::createClient(peer_address);
    if (client == nullptr) {
      ble_ui_state.store(BleUiState::Error);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    client->setClientCallbacks(&ble_client_callbacks, false);

    if (!client->connect(peer_address)) {
      NimBLEDevice::deleteClient(client);
      ble_ui_state.store(BleUiState::Scanning);
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    ble_ui_state.store(BleUiState::Initializing);
    const BleGattTransport transport = find_obd_transport(client);
    if (transport.receive == nullptr || transport.transmit == nullptr) {
      Serial.println("No BLE OBD write/notify characteristic pair found");
      client->disconnect();
      NimBLEDevice::deleteClient(client);
      ble_ui_state.store(BleUiState::Error);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    if (!transport.receive->subscribe(true, ble_notify_callback, false)) {
      client->disconnect();
      NimBLEDevice::deleteClient(client);
      ble_ui_state.store(BleUiState::Error);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    ble_elm_stream.clear();
    xSemaphoreTake(gatt_write_mutex, portMAX_DELAY);
    ble_elm_stream.setTransmitCharacteristic(transport.transmit);
    xSemaphoreGive(gatt_write_mutex);

    ELM327* elm = new (std::nothrow) ELM327();
    if (elm == nullptr || !elm->begin(ble_elm_stream, false, 6000)) {
      Serial.println("ELM327 initialization failed; retrying BLE connection");
      delete elm;
      xSemaphoreTake(gatt_write_mutex, portMAX_DELAY);
      ble_elm_stream.setTransmitCharacteristic(nullptr);
      xSemaphoreGive(gatt_write_mutex);
      client->disconnect();
      NimBLEDevice::deleteClient(client);
      ble_ui_state.store(BleUiState::Error);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    xSemaphoreTake(elm_mutex, portMAX_DELAY);
    myELM327 = elm;
    elm_ready.store(true);
    xSemaphoreGive(elm_mutex);
    link_dropped.store(false);
    ble_ui_state.store(BleUiState::Connected);

    while (client->isConnected() && !link_dropped.load()) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    xSemaphoreTake(elm_mutex, portMAX_DELAY);
    elm_ready.store(false);
    delete myELM327;
    myELM327 = nullptr;
    xSemaphoreGive(elm_mutex);

    xSemaphoreTake(gatt_write_mutex, portMAX_DELAY);
    ble_elm_stream.setTransmitCharacteristic(nullptr);
    xSemaphoreGive(gatt_write_mutex);
    ble_elm_stream.clear();
    if (client->isConnected()) {
      client->disconnect();
    }
    NimBLEDevice::deleteClient(client);
    ble_ui_state.store(BleUiState::Scanning);
  }
}

// RPM is queried every other request; the slower-changing values rotate through the gaps.
static constexpr ObdPidState kPollSchedule[] = {
  ObdPidState::Rpm, ObdPidState::Speed, ObdPidState::Rpm, ObdPidState::Coolant,
  ObdPidState::Rpm, ObdPidState::Speed, ObdPidState::Rpm, ObdPidState::Load,
  ObdPidState::Rpm, ObdPidState::Speed, ObdPidState::Rpm, ObdPidState::OilTemp,
};
static size_t poll_step = 0;
static uint8_t oil_failures = 0;
constexpr uint8_t kOilFailureLimit = 3;

static void poll_obd2() {
  if (!elm_ready.load() || myELM327 == nullptr) {
    return;
  }

  obd_state = kPollSchedule[poll_step];
  // Once the vehicle has repeatedly not answered the oil PID, use the slot for coolant instead.
  if (obd_state == ObdPidState::OilTemp && oil_failures >= kOilFailureLimit) {
    obd_state = ObdPidState::Coolant;
  }
  bool success = false;

  switch (obd_state) {
    case ObdPidState::Rpm: {
      const float value = myELM327->rpm();
      success = myELM327->nb_rx_state == ELM_SUCCESS;
      if (success) update_rpm(value);
      break;
    }
    case ObdPidState::Speed: {
      const float value = myELM327->mph();
      success = myELM327->nb_rx_state == ELM_SUCCESS;
      if (success) update_speed(value);
      break;
    }
    case ObdPidState::Coolant: {
      const float value = myELM327->engineCoolantTemp();
      success = myELM327->nb_rx_state == ELM_SUCCESS;
      if (success) update_coolant(value);
      break;
    }
    case ObdPidState::Load: {
      const float value = myELM327->engineLoad();
      success = myELM327->nb_rx_state == ELM_SUCCESS;
      if (success) update_load(value);
      break;
    }
    case ObdPidState::OilTemp: {
      const float value = myELM327->oilTemp();
      success = myELM327->nb_rx_state == ELM_SUCCESS;
      if (success) {
        oil_failures = 0;
        update_oil_temp(value);
      } else if (myELM327->nb_rx_state != ELM_GETTING_MSG && oil_failures < kOilFailureLimit) {
        if (++oil_failures >= kOilFailureLimit) {
          lv_label_set_text(oil_label, "--");
        }
      }
      break;
    }
    default:
      break;
  }

  if (success || myELM327->nb_rx_state != ELM_GETTING_MSG) {
    poll_step = (poll_step + 1) % (sizeof(kPollSchedule) / sizeof(kPollSchedule[0]));
  }
}

static const char* ble_state_text(BleUiState state) {
  switch (state) {
    case BleUiState::Scanning: return "BLE SCANNING";
    case BleUiState::Connecting: return "BLE CONNECTING";
    case BleUiState::Initializing: return "INITIALIZING OBD";
    case BleUiState::Connected: return "OBD CONNECTED";
    case BleUiState::Error: return "OBD RETRYING";
    case BleUiState::Starting: default: return "BLE STARTING";
  }
}
#endif

void setup() {
  Serial.begin(115200);

  if (!initialize_lvgl_display()) {
    Serial.println("HUD display initialization failed");
    return;
  }
  display_ready = true;
  build_hud_ui();
  last_lv_tick_ms = millis();

#if !SIMULATION_MODE
  gatt_write_mutex = xSemaphoreCreateMutex();
  elm_mutex = xSemaphoreCreateMutex();
  if (gatt_write_mutex == nullptr || elm_mutex == nullptr) {
    ble_ui_state.store(BleUiState::Error);
    Serial.println("Unable to create BLE synchronization mutexes");
    return;
  }

  BLEDevice::init("RAM1500_HUD");
  ble_scan = BLEDevice::getScan();
  ble_scan->setActiveScan(true);
  ble_scan->setInterval(100);
  ble_scan->setWindow(80);
  ble_scan->setAdvertisedDeviceCallbacks(&advertised_device_callbacks, false);

  if (xTaskCreatePinnedToCore(ble_tx_task, "ble-elm-tx", 4096, nullptr, 1,
                              nullptr, 0) != pdPASS ||
      xTaskCreatePinnedToCore(ble_connection_task, "ble-elm-connect", 8192,
                               nullptr, 1, nullptr, 1) != pdPASS) {
    ble_ui_state.store(BleUiState::Error);
    Serial.println("Unable to start BLE worker tasks");
  }
#endif
}

void loop() {
  if (!display_ready) {
    yield();
    return;
  }

  const uint32_t now_ms = millis();
  lv_tick_inc(now_ms - last_lv_tick_ms);
  last_lv_tick_ms = now_ms;
  lv_timer_handler();

#if SIMULATION_MODE
  update_simulated_telemetry(now_ms);
#else
  static BleUiState previous_ble_state = BleUiState::Starting;
  const BleUiState current_ble_state = ble_ui_state.load();
  if (current_ble_state != previous_ble_state) {
    lv_label_set_text(status_label, ble_state_text(current_ble_state));
    previous_ble_state = current_ble_state;
  }
  if (elm_mutex != nullptr && xSemaphoreTake(elm_mutex, 0) == pdTRUE) {
    poll_obd2();
    xSemaphoreGive(elm_mutex);
  }
#endif

  yield();
}