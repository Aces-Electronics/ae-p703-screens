#define LGFX_USE_V1

#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <driver/adc.h>
#include <nvs_flash.h>
#include <Preferences.h>
#include <CircularBuffer.h>
#include "lvgl.h"
#include "ui.h"

// GPIO definitions
const int vin = 14;
const int hp1 = 10; // outputs
const int hp2 = 11;
const int lp1 = 12;
const int lp2 = 13;

bool hp1IOState = 0;
bool hp2IOState = 0;
bool lp1IOState = 0;
bool lp2IOState = 0;

String hp1Label = "HP1";
String hp2Label = "HP2";
String lp1Label = "LP1";
String lp2Label = "LP2";

String batteryState = "Stable";

bool hp1Changed = 0;
bool hp2Changed = 0;
bool lp1Changed = 0;
bool lp2Changed = 0;

int rawValue = 0;
float auxVoltage;
float lastReading;

const float r1 = 82000.0f; // R1 in ohm, 82k
const float r2 = 16000.0f; // R2 in ohm, 16k
float vRefScale = (3.3f / 4096.0f) * ((r1 + r2) / r2);
const int numReadings = 100;
int readings[numReadings];
int readIndex = 0;
long total = 0;
char vinResult[8];

unsigned long newtime = 0;

Preferences preferences;

CircularBuffer<float, 500> buffer;


class LGFX : public lgfx::LGFX_Device
{

  lgfx::Panel_ST7796 _panel_instance;
  lgfx::Bus_Parallel8 _bus_instance; // 8ビットパラレルバスのインスタンス (ESP32のみ)
  lgfx::Light_PWM _light_instance;
  lgfx::Touch_FT5x06 _touch_instance; // FT5206, FT5306, FT5406, FT6206, FT6236, FT6336, FT6436

public:
  LGFX(void)
  {
    {                                    // バス制御の設定を行います。
      auto cfg = _bus_instance.config(); // バス設定用の構造体を取得します。
                                         // 8ビットパラレルバスの設定
      // cfg.i2s_port = I2S_NUM_0;     // 使用するI2Sポートを選択 (I2S_NUM_0 or I2S_NUM_1) (ESP32のI2S LCDモードを使用します)
      cfg.freq_write = 20000000;              // 送信クロック (最大20MHz, 80MHzを整数で割った値に丸められます)
      cfg.pin_wr = 47;                        // WR を接続しているピン番号
      cfg.pin_rd = -1;                        // RD を接続しているピン番号
      cfg.pin_rs = 0;                         // RS(D/C)を接続しているピン番号
      cfg.pin_d0 = 9;                         // D0を接続しているピン番号
      cfg.pin_d1 = 46;                        // D1を接続しているピン番号
      cfg.pin_d2 = 3;                         // D2を接続しているピン番号
      cfg.pin_d3 = 8;                         // D3を接続しているピン番号
      cfg.pin_d4 = 18;                        // D4を接続しているピン番号
      cfg.pin_d5 = 17;                        // D5を接続しているピン番号
      cfg.pin_d6 = 16;                        // D6を接続しているピン番号
      cfg.pin_d7 = 15;                        // D7を接続しているピン番号
      _bus_instance.config(cfg);              // 設定値をバスに反映します。
      _panel_instance.setBus(&_bus_instance); // バスをパネルにセットします。
    }

    {                                      // 表示パネル制御の設定を行います。
      auto cfg = _panel_instance.config(); // 表示パネル設定用の構造体を取得します。

      cfg.pin_cs = -1;   // CSが接続されているピン番号   (-1 = disable)
      cfg.pin_rst = 4;   // RSTが接続されているピン番号  (-1 = disable)
      cfg.pin_busy = -1; // BUSYが接続されているピン番号 (-1 = disable)

      // ※ 以下の設定値はパネル毎に一般的な初期値が設定されていますので、不明な項目はコメントアウトして試してみてください。

      cfg.panel_width = 320;    // 実際に表示可能な幅
      cfg.panel_height = 480;   // 実際に表示可能な高さ
      cfg.offset_x = 0;         // パネルのX方向オフセット量
      cfg.offset_y = 0;         // パネルのY方向オフセット量
      cfg.offset_rotation = 0;  // 回転方向の値のオフセット 0~7 (4~7は上下反転)
      cfg.dummy_read_pixel = 8; // ピクセル読出し前のダミーリードのビット数
      cfg.dummy_read_bits = 1;  // ピクセル以外のデータ読出し前のダミーリードのビット数
      cfg.readable = true;      // データ読出しが可能な場合 trueに設定
      cfg.invert = true;        // パネルの明暗が反転してしまう場合 trueに設定
      cfg.rgb_order = false;    // パネルの赤と青が入れ替わってしまう場合 trueに設定
      cfg.dlen_16bit = false;   // 16bitパラレルやSPIでデータ長を16bit単位で送信するパネルの場合 trueに設定
      cfg.bus_shared = true;    // SDカードとバスを共有している場合 trueに設定(drawJpgFile等でバス制御を行います)

      // 以下はST7735やILI9163のようにピクセル数が可変のドライバで表示がずれる場合にのみ設定してください。
      //    cfg.memory_width     =   240;  // ドライバICがサポートしている最大の幅
      //    cfg.memory_height    =   320;  // ドライバICがサポートしている最大の高さ

      _panel_instance.config(cfg);
    }

    //*
    {                                      // バックライト制御の設定を行います。（必要なければ削除）
      auto cfg = _light_instance.config(); // バックライト設定用の構造体を取得します。

      cfg.pin_bl = 45;     // バックライトが接続されているピン番号
      cfg.invert = false;  // バックライトの輝度を反転させる場合 true
      cfg.freq = 44100;    // バックライトのPWM周波数
      cfg.pwm_channel = 7; // 使用するPWMのチャンネル番号

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance); // バックライトをパネルにセットします。
    }
    //*/

    //*
    { // タッチスクリーン制御の設定を行います。（必要なければ削除）
      auto cfg = _touch_instance.config();

      cfg.x_min = 0;           // タッチスクリーンから得られる最小のX値(生の値)
      cfg.x_max = 319;         // タッチスクリーンから得られる最大のX値(生の値)
      cfg.y_min = 0;           // タッチスクリーンから得られる最小のY値(生の値)
      cfg.y_max = 479;         // タッチスクリーンから得られる最大のY値(生の値)
      cfg.pin_int = 7;         // INTが接続されているピン番号
      cfg.bus_shared = true;   // 画面と共通のバスを使用している場合 trueを設定
      cfg.offset_rotation = 0; // 表示とタッチの向きのが一致しない場合の調整 0~7の値で設定
                               // I2C接続の場合
      cfg.i2c_port = 1;        // 使用するI2Cを選択 (0 or 1)
      cfg.i2c_addr = 0x38;     // I2Cデバイスアドレス番号
      cfg.pin_sda = 6;         // SDAが接続されているピン番号
      cfg.pin_scl = 5;         // SCLが接続されているピン番号
      cfg.freq = 400000;       // I2Cクロックを設定

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance); // タッチスクリーンをパネルにセットします。
    }
    //*/
    setPanel(&_panel_instance); // 使用するパネルをセットします。
  }
};

// 準備したクラスのインスタンスを作成します。

LGFX tft;

#define screenWidth 480
#define screenHeight 320

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.writePixels((lgfx::rgb565_t *)&color_p->full, w * h);
  tft.endWrite();
  lv_disp_flush_ready(disp);
}

void my_touch_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  uint16_t touchX, touchY;
  bool touched = tft.getTouch(&touchX, &touchY);
  if (!touched)
  {
    data->state = LV_INDEV_STATE_REL;
  }
  else
  {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchX;
    data->point.y = touchY;
  }
}

static lv_style_t label_style;
static lv_obj_t *headerLabel;
static lv_obj_t *label;

void savePreferences()
{
  preferences.begin("basic", false);
  preferences.putString("hp1Label", hp1Label);
  preferences.putString("hp2Label", hp2Label);
  preferences.putString("lp1Label", lp1Label);
  preferences.putString("lp2Label", lp2Label);

  preferences.putBool("hp1IOState", hp1IOState);
  preferences.putBool("hp2IOState", hp2IOState);
  preferences.putBool("lp1IOState", lp1IOState);
  preferences.putBool("lp2IOState", lp2IOState);

  preferences.end();
}

void set_screen_brightness(lv_event_t *e)
{
  lv_obj_t *slider = lv_event_get_target(e);

  int brightness = lv_slider_get_value(slider);
  if (brightness < 10)
    brightness = 10;
  tft.setBrightness(brightness * 2.55);
}

void toggleKeyboard(lv_event_t *e)
{
  lv_obj_clear_flag(ui_settingsKeyboard, LV_OBJ_FLAG_HIDDEN);
}

void ui_event_hp1TextArea(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_VALUE_CHANGED)
  {
    hp1Changed = true;
  }
  if (event_code == LV_EVENT_READY)
  {
    lv_obj_add_flag(ui_settingsKeyboard, LV_OBJ_FLAG_HIDDEN);
    if (hp1Changed)
    {
      hp1Label = lv_textarea_get_text(ui_hp1TextArea);
      lv_label_set_text(ui_hp1Label, hp1Label.c_str());
      lv_label_set_text(ui_ioLabel1, hp1Label.c_str());
      savePreferences();
    }
  }
  if (event_code == LV_EVENT_CLICKED)
  {
    _ui_keyboard_set_target(ui_settingsKeyboard, ui_hp1TextArea);
    toggleKeyboard(e);
  }
  hp1Changed = false;
}

void ui_event_hp2TextArea(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_VALUE_CHANGED)
  {
    hp2Changed = true;
  }
  if (event_code == LV_EVENT_READY)
  {
    lv_obj_add_flag(ui_settingsKeyboard, LV_OBJ_FLAG_HIDDEN);
    if (hp2Changed)
    {
      hp2Label = lv_textarea_get_text(ui_hp2TextArea);
      lv_label_set_text(ui_hp2Label, hp2Label.c_str());
      lv_label_set_text(ui_ioLabel2, hp2Label.c_str());
      savePreferences();
    }
  }
  if (event_code == LV_EVENT_CLICKED)
  {
    _ui_keyboard_set_target(ui_settingsKeyboard, ui_hp2TextArea);
    toggleKeyboard(e);
  }
  hp2Changed = false;
}

void ui_event_lp1TextArea(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_VALUE_CHANGED)
  {
    lp1Changed = true;
  }
  if (event_code == LV_EVENT_READY)
  {
    lv_obj_add_flag(ui_settingsKeyboard, LV_OBJ_FLAG_HIDDEN);
    if (lp1Changed)
    {
      lp1Label = lv_textarea_get_text(ui_lp1TextArea);
      lv_label_set_text(ui_lp1Label, lp1Label.c_str());
      lv_label_set_text(ui_ioLabel3, lp1Label.c_str());
      savePreferences();
    }
  }
  if (event_code == LV_EVENT_CLICKED)
  {
    _ui_keyboard_set_target(ui_settingsKeyboard, ui_lp1TextArea);
    toggleKeyboard(e);
  }
  lp1Changed = false;
}

void ui_event_lp2TextArea(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_VALUE_CHANGED)
  {
    lp2Changed = true;
  }
  if (event_code == LV_EVENT_READY)
  {
    lv_obj_add_flag(ui_settingsKeyboard, LV_OBJ_FLAG_HIDDEN);
    if (lp2Changed)
    {
      lp2Label = lv_textarea_get_text(ui_lp2TextArea);
      lv_label_set_text(ui_lp2Label, lp2Label.c_str());
      lv_label_set_text(ui_ioLabel4, lp2Label.c_str());
      savePreferences();
    }
  }
  if (event_code == LV_EVENT_CLICKED)
  {
    _ui_keyboard_set_target(ui_settingsKeyboard, ui_lp2TextArea);
    toggleKeyboard(e);
  }
  lp2Changed = false;
}

void hp1ToggleFunction(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    hp1IOState = !hp1IOState;
    if (hp1IOState == 1)
    {
      lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io1, LV_STATE_CHECKED);
      digitalWrite(hp1, hp1IOState);
      savePreferences();
    }
    else
    {
      lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io1, LV_STATE_CHECKED);
      digitalWrite(hp1, hp1IOState);
      savePreferences();
    }
  }
}

void hp2ToggleFunction(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    hp2IOState = !hp2IOState;
    if (hp2IOState == 1)
    {
      lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io2, LV_STATE_CHECKED);
      digitalWrite(hp2, hp2IOState);
      savePreferences();
    }
    else
    {
      lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io2, LV_STATE_CHECKED);
      digitalWrite(hp2, hp2IOState);
      savePreferences();
    }
  }
}

void lp1ToggleFunction(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    lp1IOState = !lp1IOState;
    if (lp1IOState == 1)
    {
      lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io3, LV_STATE_CHECKED);
      digitalWrite(lp1, lp1IOState);
      savePreferences();
    }
    else
    {
      lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io3, LV_STATE_CHECKED);
      digitalWrite(lp1, lp1IOState);
      savePreferences();
    }
  }
}

void lp2ToggleFunction(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    lp2IOState = !lp2IOState;
    if (lp2IOState == 1)
    {
      lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io4, LV_STATE_CHECKED);
      digitalWrite(lp2, lp2IOState);
      savePreferences();
    }
    else
    {
      lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io4, LV_STATE_CHECKED);
      digitalWrite(lp2, lp2IOState);
      savePreferences();
    }
  }
}

void factoryReset(lv_event_t *e)
{
  nvs_flash_erase(); // erase the NVS partition.
  nvs_flash_init();  // initialize the NVS partition.
  delay(500);
  ESP.restart(); // reset to clear memory
}

void readAnalogVoltage()
{                  /* function readAnalogSmooth */
  analogRead(vin); // turn and burn
}

long smooth()
{ /* function smooth */
  ////Perform average on sensor readings
  long average;
  // subtract the last reading:
  total = total - readings[readIndex];
  // read the sensor:
  readings[readIndex] = analogRead(vin);
  // add value to total:
  total = total + readings[readIndex];
  // handle index
  readIndex = readIndex + 1;
  if (readIndex >= numReadings)
  {
    readIndex = 0;
  }
  // calculate the average:
  average = total / numReadings;

  return average;
}

void checkVin()
{
  readAnalogVoltage(); // no calibration
  float tempAuxVoltage = smooth() * (vRefScale * 1.006);
  buffer.push(tempAuxVoltage); // fill the circular buffer for super smooth values

  if (millis() - newtime >= 500) {
		newtime = millis();
		float avg = 0.0;
		// the following ensures using the right type for the index variable
		using index_t = decltype(buffer)::index_t;
		for (index_t i = 0; i < buffer.size(); i++) {
			avg += buffer[i] / buffer.size();
		}
    auxVoltage = avg;
    dtostrf(auxVoltage, 6, 2, vinResult);
    char tmp[2] = "V";
    strcat(vinResult, tmp);

    if (buffer.size() > 499)
    {
      if (avg > lastReading)
      {
        batteryState = "Charging";
      }
      else if (avg < lastReading)
      {
        batteryState = "Discharging";
      }
      else
      {
        batteryState = "Stable";
      }
    }
    else
    {
      batteryState = "Checking...";
    }
    lv_label_set_text(ui_auxState, batteryState.c_str());
	}

  lv_label_set_text(ui_auxBattVoltageLabel, vinResult);
  if (auxVoltage >= 14.55)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "100%");
    lv_arc_set_value(ui_auxBattVoltageArc, 100);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage >= 13.55)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "100%");
    lv_arc_set_value(ui_auxBattVoltageArc, 100);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage >= 13.4)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "99%");
    lv_arc_set_value(ui_auxBattVoltageArc, 99);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage >= 13.3)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "90%");
    lv_arc_set_value(ui_auxBattVoltageArc, 90);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage >= 13.15)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "70%");
    lv_arc_set_value(ui_auxBattVoltageArc, 70);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage >= 13.1)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "40%");
    lv_arc_set_value(ui_auxBattVoltageArc, 40);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xE3ED00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xE3ED00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xE3ED00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xE3ED00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage >= 13.0)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "30%");
    lv_arc_set_value(ui_auxBattVoltageArc, 30);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xE8B23B), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xE8B23B), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xE8B23B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xE8B23B), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage >= 12.9)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "20%");
    lv_arc_set_value(ui_auxBattVoltageArc, 20);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xE8B23B), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xE8B23B), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xE8B23B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xE8B23B), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage >= 12.8)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "17%");
    lv_arc_set_value(ui_auxBattVoltageArc, 17);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xF06319), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xF06319), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xF06319), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xF06319), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage >= 12.5)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "14%");
    lv_arc_set_value(ui_auxBattVoltageArc, 14);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage >= 12.0)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "9%");
    lv_arc_set_value(ui_auxBattVoltageArc, 9);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage >= 10)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "0%");
    lv_arc_set_value(ui_auxBattVoltageArc, 0);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
}

void loadPreferences()
{
  if (preferences.begin("basic", false))
  {
    if (preferences.getString("hp1Label", "None") != "None")
    {
      hp1Label = preferences.getString("hp1Label");
      lv_label_set_text(ui_hp1Label, hp1Label.c_str());
      lv_label_set_text(ui_ioLabel1, hp1Label.c_str());

      hp2Label = preferences.getString("hp2Label");
      lv_label_set_text(ui_hp2Label, hp2Label.c_str());
      lv_label_set_text(ui_ioLabel2, hp2Label.c_str());

      lp1Label = preferences.getString("lp1Label");
      lv_label_set_text(ui_lp1Label, lp1Label.c_str());
      lv_label_set_text(ui_ioLabel3, lp1Label.c_str());

      lp2Label = preferences.getString("lp2Label");
      lv_label_set_text(ui_lp2Label, lp2Label.c_str());
      lv_label_set_text(ui_ioLabel4, lp2Label.c_str());

      hp1IOState = preferences.getBool("hp1IOState", false);
      if (hp1IOState == 1)
      {
        lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_add_state(ui_io1, LV_STATE_CHECKED);
        digitalWrite(hp1, hp1IOState);
      }
      else
      {
        lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_clear_state(ui_io1, LV_STATE_CHECKED);
        digitalWrite(hp1, hp1IOState);
      }

      hp2IOState = preferences.getBool("hp2IOState", false);
      if (hp2IOState == 1)
      {
        lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_add_state(ui_io2, LV_STATE_CHECKED);
        digitalWrite(hp2, hp2IOState);
      }
      else
      {
        lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_clear_state(ui_io2, LV_STATE_CHECKED);
        digitalWrite(hp2, hp2IOState);
      }

      lp1IOState = preferences.getBool("lp1IOState", false);
      if (lp1IOState == 1)
      {
        lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_add_state(ui_io3, LV_STATE_CHECKED);
        digitalWrite(lp1, lp1IOState);
      }
      else
      {
        lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_clear_state(ui_io3, LV_STATE_CHECKED);
        digitalWrite(lp1, lp1IOState);
      }

      lp2IOState = preferences.getBool("lp2IOState", false);
      if (lp2IOState == 1)
      {
        lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_add_state(ui_io4, LV_STATE_CHECKED);
        digitalWrite(lp2, lp2IOState);
      }
      else
      {
        lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_clear_state(ui_io4, LV_STATE_CHECKED);
        digitalWrite(lp2, lp2IOState);
      }
    }
    else
    {
      preferences.putString("hp1Label", hp1Label);
      preferences.putString("hp2Label", hp2Label);
      preferences.putString("lp1Label", lp1Label);
      preferences.putString("lp2Label", lp2Label);

      preferences.putBool("hp1IOState", hp1IOState);
      preferences.putBool("hp2IOState", hp2IOState);
      preferences.putBool("lp1IOState", lp1IOState);
      preferences.putBool("lp2IOState", lp2IOState);
    }
  }
  preferences.end();
}

void setup()
{
  Serial.begin(115200);

  pinMode(hp1, OUTPUT);
  pinMode(hp2, OUTPUT);
  pinMode(lp1, OUTPUT);
  pinMode(lp2, OUTPUT);
  pinMode(vin, INPUT);

  tft.begin();
  tft.setRotation(3);     // 3 = upside down
  tft.setBrightness(255); // ToDo: make this an NVRAM setting and read it in on boot

  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touch_read;
  lv_indev_drv_register(&indev_drv);

  ui_init();

  loadPreferences();
  newtime = millis();
}

void loop()
{
  lv_timer_handler();
  checkVin();
  delay(5);
}
