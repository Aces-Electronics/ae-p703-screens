// REAR IO Basic
#define LGFX_USE_V1

#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <driver/adc.h>
#include <nvs_flash.h>
#include <Preferences.h>
#include <CircularBuffer.h>
#include "lvgl.h"
#include "ui.h"
#include <esp_now.h>
#include <WiFi.h>

// GPIO definitions
const int vin = 14;
const int hp1 = 10; // outputs
const int hp2 = 11;
const int lp1 = 12;
const int lp2 = 13;

String rearDeviceState = "Stable";

int rawValue = 0;
float auxVoltage;
float oldAuxVoltage;
float lastReading;

int loopCounter;

const float r1 = 82000.0f; // R1 in ohm, 82k
const float r2 = 16000.0f; // R2 in ohm, 16k
float vRefScale = (3.3f / 4096.0f) * ((r1 + r2) / r2);
const int numReadings = 100;
int readings[numReadings];
int readIndex = 0;
long total = 0;
String vinResult = "WAIT!";

unsigned long newtime = 0;

Preferences preferences;

CircularBuffer<float, 500> buffer;

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Variable to store if sending data was successful
String success;

bool priorityMessage = 1;

typedef struct struct_message_priority {
  int messageID = 255;
} struct_message_priority;

//Must match the receiver structure
typedef struct struct_message_rear0 { // Rear hardware message
  int messageID = 0; // sets the message ID
  bool dataChanged = 0; // stores whether or not the data in the struct has changed
  int rearIO1 = -1; // rear: basic/pro IO1
  String rearIO1Name = "15A-1";
  int rearIO2 = -1; // rear: basic/pro IO2
  String rearIO2Name = "15A-2";
  int rearIO3 = -1; // rear: basic/pro IO3
  String rearIO3Name = "10A-1";
  int rearIO4 = -1; // rear: basic/pro IO4
  String rearIO4Name = "10A-2";
  int rearIO5 = -1; // rear: pro IO5
  String rearIO5Name = "-1";
  int rearIO6 = -1; // rear: pro IO6
  String rearIO6Name = "-1";
  int rearIO7 = -1; // rear: pro IO7
  String rearIO7Name = "-1";
  int rearIO8 = -1; // rear: pro IO8
  String rearIO8Name = "-1";
  int rearIO9 = -1; // rear: pro IO9
  String rearIO9Name = "-1";
  int rearIO10 = -1; // rear: pro IO10
  String rearIO10Name = "-1";

  String rearDeviceState = "WAIT!";
} struct_message_rear0;

typedef struct struct_message_aux0 { // Aux hardware message
  int messageID = 1;
  bool dataChanged = 0; // stores whether or not the data in the struct has changed
  String auxIO10Name = "-1";;
  int auxIO11 = -1; // aux: IO11
  String auxIO11Name = "-1";
  int auxIO12 = -1; // aux: IO12
  String auxIO12Name = "-1";
  int auxIO13 = -1; // aux: IO13
  String auxIO13Name = "-1";
  int auxIO14 = -1; // aux: IO14
  String auxIO14Name = "-1";
  int auxIO15 = -1; // aux: IO15
  String auxIO15Name = "-1";
  int auxIO16 = -1; // aux: IO16
  String auxIO16Name = "-1";
  int auxIO17 = -1; // aux: IO17
  String auxIO17Name = "-1";
  int auxIO18 = -1; // aux: IO18
  String auxIO18Name = "-1";
  int auxIO19 = -1; // aux: IO19
  String auxIO19Name = "-1";
  int auxIO20 = -1; // aux: IO20
  String auxIO20Name = "-1";

  String auxDeviceState = "WAIT!";
} struct_message_aux0;

typedef struct struct_message_front0 { // Front hardware message
  int messageID = 2;
  bool dataChanged = 0; // stores whether or not the data in the struct has changed
  int frontIO21 = -1; // front: basic/pro IO1
  String frontIO21Name = "-1";
  int frontIO22 = -1; // front: basic/pro IO2
  String frontIO22Name = "-1";
  int frontIO23 = -1; // front: basic/pro IO3
  String frontIO23Name = "-1";
  int frontIO24 = -1; // front: basic/pro IO4
  String frontIO24Name = "-1";
  int frontIO25 = -1; // front: pro IO5
  String frontIO25Name = "-1";
  int frontIO26 = -1; // front: pro IO6
  String frontIO26Name = "-1";
  int frontIO27 = -1; // front: pro IO7
  String frontIO28Name = "-1";
  int frontIO28 = -1; // front: pro IO8
  String frontIO29Name = "-1";
  int frontIO29 = -1; // front: pro IO9
  String frontIO30Name = "-1";
  int frontIO30 = -1; // front: pro IO10

  String frontDeviceState = "WAIT!";
} struct_message_front0;

typedef struct struct_message_voltage0 { // Voltage message
  int messageID = 3;
  bool dataChanged = 0; // stores whether or not the data in the struct has changed
  float frontMainBatt1V = -1;
  float frontAuxBatt1V = -1;
  float rearMainBatt1V = -1;
  float rearAuxBatt1V = -1;
  float frontMainBatt1I = -1;
  float frontAuxBatt1I = -1;
  float rearMainBatt1I = -1;
  float rearAuxBatt1I = -1; 
} struct_message_voltage0;

struct_message_priority priorityMessageStruct;

// Create a struct_message called localReadings to hold sensor readings
struct_message_rear0 localRear0Struct;
struct_message_aux0 localAux0Struct;
struct_message_front0 localFront0Struct;
struct_message_voltage0 localVoltage0Struct;

// Create a struct_message to hold incoming sensor readings
struct_message_rear0 remoteRear0Struct;
struct_message_aux0 remoteAux0Struct;
struct_message_front0 remoteFront0Struct;
struct_message_voltage0 remoteVoltage0Struct;

esp_now_peer_info_t peerInfo;

void sendMessage ()
{
  // create the data
  //localReadings0StructStruct.incomingio1Name[0] = 'Test';

  if (priorityMessage)
  {
    // Send message via ESP-NOW
    esp_err_t result0 = esp_now_send(broadcastAddress, (uint8_t *) &priorityMessageStruct, sizeof(priorityMessageStruct));
    if (result0 == ESP_OK) {
      Serial.println("Sent priority message with success");
    }
    else {
      Serial.println("Error sending the data");
    }
    priorityMessage = 0;
  }
  else
  {
    // Send message via ESP-NOW
    esp_err_t result0 = esp_now_send(broadcastAddress, (uint8_t *) &localRear0Struct, sizeof(localRear0Struct));
    if (result0 == ESP_OK) {
      Serial.println("Sent message 0 with success");
    }
    else {
      Serial.println("Error sending the data");
    }
    delay(200);
    esp_err_t result1 = esp_now_send(broadcastAddress, (uint8_t *) &localVoltage0Struct, sizeof(localVoltage0Struct));
    if (result1 == ESP_OK) {
      Serial.println("Sent message 1 with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  uint8_t type = incomingData[0]; 

  switch (type) {
    case 0 : // message ID 0
      memcpy(&remoteRear0Struct, incomingData, sizeof(remoteRear0Struct));
      Serial.print("0: Bytes received: "); Serial.println(len);
      
      localRear0Struct.messageID = remoteRear0Struct.messageID; // stores the message ID
      localRear0Struct.dataChanged = remoteRear0Struct.dataChanged; // stores whether or not the data in the struct has changed

      if (remoteRear0Struct.rearIO1 != -1)
      {
        localRear0Struct.rearIO1 = remoteRear0Struct.rearIO1; // rear: basic/pro IO1
      } 

      if (remoteRear0Struct.rearIO1Name != "-1")
      {
        localRear0Struct.rearIO1Name = remoteRear0Struct.rearIO1Name;
      } 

      if (remoteRear0Struct.rearIO2 != -1)
      {
        localRear0Struct.rearIO2 = remoteRear0Struct.rearIO2; // rear: basic/pro IO1
      } 

      if (remoteRear0Struct.rearIO2Name != "-1")
      {
        localRear0Struct.rearIO2Name = remoteRear0Struct.rearIO2Name;
      } 
       
      if (remoteRear0Struct.rearIO3 != -1)
      {
        localRear0Struct.rearIO3 = remoteRear0Struct.rearIO3; // rear: basic/pro IO1
      } 

      if (remoteRear0Struct.rearIO3Name != "-1")
      {
        localRear0Struct.rearIO3Name = remoteRear0Struct.rearIO3Name;
      } 

      if (remoteRear0Struct.rearIO4 != -1)
      {
        localRear0Struct.rearIO4 = remoteRear0Struct.rearIO4; // rear: basic/pro IO1
      }

      if (remoteRear0Struct.rearIO4Name != "-1")
      {
        localRear0Struct.rearIO4Name = remoteRear0Struct.rearIO4Name;
      } 

      if (remoteRear0Struct.rearIO5 != -1)
      {
        localRear0Struct.rearIO5 = remoteRear0Struct.rearIO5; // rear: basic/pro IO1
      }

      if (remoteRear0Struct.rearIO5Name != "-1")
      {
        localRear0Struct.rearIO5Name = remoteRear0Struct.rearIO5Name;
      } 

      if (remoteRear0Struct.rearIO6 != -1)
      {
        localRear0Struct.rearIO6 = remoteRear0Struct.rearIO6; // rear: basic/pro IO1
      }

      if (remoteRear0Struct.rearIO6Name != "-1")
      {
        localRear0Struct.rearIO6Name = remoteRear0Struct.rearIO6Name;
      } 

      if (remoteRear0Struct.rearIO7 != -1)
      {
        localRear0Struct.rearIO7 = remoteRear0Struct.rearIO7; // rear: basic/pro IO1
      }

      if (remoteRear0Struct.rearIO7Name != "-1")
      {
        localRear0Struct.rearIO7Name = remoteRear0Struct.rearIO7Name;
      } 

      if (remoteRear0Struct.rearIO8 != -1)
      {
        localRear0Struct.rearIO8 = remoteRear0Struct.rearIO8; // rear: basic/pro IO1
      }

      if (remoteRear0Struct.rearIO8Name != "-1")
      {
        localRear0Struct.rearIO8Name = remoteRear0Struct.rearIO8Name;
      } 

      if (remoteRear0Struct.rearIO9 != -1)
      {
        localRear0Struct.rearIO9 = remoteRear0Struct.rearIO9; // rear: basic/pro IO1
      }

      if (remoteRear0Struct.rearIO9Name != "-1")
      {
        localRear0Struct.rearIO9Name = remoteRear0Struct.rearIO9Name;
      } 

      if (remoteRear0Struct.rearIO10 != -1)
      {
        localRear0Struct.rearIO10 = remoteRear0Struct.rearIO10; // rear: basic/pro IO1
      }

      if (remoteRear0Struct.rearIO10Name != "-1")
      {
        localRear0Struct.rearIO10Name = remoteRear0Struct.rearIO10Name;
      } 

      if (localRear0Struct.rearDeviceState != "-1")
      {
        localRear0Struct.rearDeviceState = remoteRear0Struct.rearDeviceState;
        lv_label_set_text(ui_auxState, localRear0Struct.rearDeviceState.c_str());
      } 
    break;
    break;

    case 3 : // message ID 1
      memcpy(&remoteVoltage0Struct, incomingData, sizeof(remoteVoltage0Struct));
      Serial.print("1: Bytes received: "); Serial.println(len);
      if (remoteVoltage0Struct.frontMainBatt1V != -1)
      {
        localVoltage0Struct.frontMainBatt1V = remoteVoltage0Struct.frontMainBatt1V;
      }

      if (remoteVoltage0Struct.frontAuxBatt1V != -1)
      {
        localVoltage0Struct.frontAuxBatt1V = remoteVoltage0Struct.frontAuxBatt1V;
      }

      if (remoteVoltage0Struct.rearMainBatt1V != -1)
      {
        localVoltage0Struct.rearMainBatt1V = remoteVoltage0Struct.rearMainBatt1V;
      }

      if (remoteVoltage0Struct.rearAuxBatt1V != -1)
      {
        localVoltage0Struct.rearAuxBatt1V = remoteVoltage0Struct.rearAuxBatt1V;
      }

      if (remoteVoltage0Struct.rearAuxBatt1I != -1)
      {
        localVoltage0Struct.rearAuxBatt1I = remoteVoltage0Struct.rearAuxBatt1I;
      }

      if (remoteVoltage0Struct.frontAuxBatt1I != -1)
      {
        localVoltage0Struct.frontAuxBatt1I = remoteVoltage0Struct.frontAuxBatt1I;
      }

      if (remoteVoltage0Struct.rearMainBatt1I != -1)
      {
        localVoltage0Struct.rearMainBatt1I = remoteVoltage0Struct.rearMainBatt1I;
      }  

      if (remoteVoltage0Struct.rearAuxBatt1I != -1)
      {
        localVoltage0Struct.rearAuxBatt1I = remoteVoltage0Struct.rearAuxBatt1I;
      }     
    break;

    case 255 : // message ID 255: means that a device has rebooted and needs data outside the sync window
      sendMessage();
    break;
  }
}

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

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
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
  Serial.println("saving to flash");
  preferences.begin("basic", false);
  preferences.putString("hp1Label", localRear0Struct.rearIO1Name);
  preferences.putString("hp2Label", localRear0Struct.rearIO2Name);
  preferences.putString("lp1Label", localRear0Struct.rearIO3Name);
  preferences.putString("lp2Label", localRear0Struct.rearIO4Name);

  preferences.putInt("hp1IOState", localRear0Struct.rearIO1);
  preferences.putInt("hp2IOState", localRear0Struct.rearIO2);
  preferences.putInt("lp1IOState", localRear0Struct.rearIO3);
  preferences.putInt("lp2IOState", localRear0Struct.rearIO4);

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
  if (event_code == LV_EVENT_READY)
  {
    if (strlen(lv_textarea_get_text(ui_hp1TextArea)) !=0)
    {
      lv_obj_add_flag(ui_settingsKeyboard, LV_OBJ_FLAG_HIDDEN);
      localRear0Struct.rearIO1Name = lv_textarea_get_text(ui_hp1TextArea);
      lv_label_set_text(ui_hp1Label, localRear0Struct.rearIO1Name.c_str());
      lv_label_set_text(ui_ioLabel1, localRear0Struct.rearIO1Name.c_str());
      savePreferences();
      sendMessage();
    }
  }
  if (event_code == LV_EVENT_CLICKED)
  {
    _ui_keyboard_set_target(ui_settingsKeyboard, ui_hp1TextArea);
    toggleKeyboard(e);
  }
}

void ui_event_hp2TextArea(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_READY)
  {
    if (strlen(lv_textarea_get_text(ui_hp2TextArea)) !=0)
    {
      lv_obj_add_flag(ui_settingsKeyboard, LV_OBJ_FLAG_HIDDEN);
      localRear0Struct.rearIO2Name = lv_textarea_get_text(ui_hp2TextArea);
      lv_label_set_text(ui_hp2Label, localRear0Struct.rearIO2Name.c_str());
      lv_label_set_text(ui_ioLabel2, localRear0Struct.rearIO2Name.c_str());
      savePreferences();
    }
  }
  if (event_code == LV_EVENT_CLICKED)
  {
    _ui_keyboard_set_target(ui_settingsKeyboard, ui_hp2TextArea);
    toggleKeyboard(e);
  }
}

void ui_event_lp1TextArea(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_READY)
  {
    if (strlen(lv_textarea_get_text(ui_lp1TextArea)) !=0)
    {
      lv_obj_add_flag(ui_settingsKeyboard, LV_OBJ_FLAG_HIDDEN);
      localRear0Struct.rearIO3Name = lv_textarea_get_text(ui_lp1TextArea);
      lv_label_set_text(ui_lp1Label, localRear0Struct.rearIO3Name.c_str());
      lv_label_set_text(ui_ioLabel3, localRear0Struct.rearIO3Name.c_str());
      savePreferences();
    }
  }
  if (event_code == LV_EVENT_CLICKED)
  {
    _ui_keyboard_set_target(ui_settingsKeyboard, ui_lp1TextArea);
    toggleKeyboard(e);
  }
}

void ui_event_lp2TextArea(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_READY)
  {
    if (strlen(lv_textarea_get_text(ui_lp2TextArea)) !=0)
    {
      lv_obj_add_flag(ui_settingsKeyboard, LV_OBJ_FLAG_HIDDEN);
      localRear0Struct.rearIO4Name = lv_textarea_get_text(ui_lp2TextArea);
      lv_label_set_text(ui_lp2Label, localRear0Struct.rearIO4Name.c_str());
      lv_label_set_text(ui_ioLabel4, localRear0Struct.rearIO4Name.c_str());
      savePreferences();
    }
  }
  if (event_code == LV_EVENT_CLICKED)
  {
    _ui_keyboard_set_target(ui_settingsKeyboard, ui_lp2TextArea);
    toggleKeyboard(e);
  }
}

void hp1ToggleFunction(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    localRear0Struct.rearIO1 = !localRear0Struct.rearIO1; // ToDo: check this logic
    if (localRear0Struct.rearIO1 == 1)
    {
      lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io1, LV_STATE_CHECKED);
      savePreferences();
    }
    else
    {
      lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io1, LV_STATE_CHECKED);
      savePreferences();
    }
    digitalWrite(hp1, localRear0Struct.rearIO1);
    sendMessage();
  }
}

void hp2ToggleFunction(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    localRear0Struct.rearIO2 = !localRear0Struct.rearIO2;
    if (localRear0Struct.rearIO2 == 1)
    {
      lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io2, LV_STATE_CHECKED);
      savePreferences();
    }
    else
    {
      lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io2, LV_STATE_CHECKED);
      savePreferences();
    }
    digitalWrite(hp2, localRear0Struct.rearIO2);
    sendMessage();
  }
}

void lp1ToggleFunction(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    localRear0Struct.rearIO3 = !localRear0Struct.rearIO3;
    if (localRear0Struct.rearIO3 == 1)
    {
      lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io3, LV_STATE_CHECKED);
      savePreferences();
    }
    else
    {
      lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io3, LV_STATE_CHECKED);
      savePreferences();
    }
  }
  digitalWrite(lp1, localRear0Struct.rearIO3);
  sendMessage();
}

void lp2ToggleFunction(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    localRear0Struct.rearIO4 = !localRear0Struct.rearIO4;
    if (localRear0Struct.rearIO4 == 1)
    {
      lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io4, LV_STATE_CHECKED);
      savePreferences();
    }
    else
    {
      lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io4, LV_STATE_CHECKED);
      savePreferences();
    }
  }
  digitalWrite(lp2, localRear0Struct.rearIO4);
  sendMessage();
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

void checkData()
{
  readAnalogVoltage(); // no calibration
  buffer.push(smooth() * (vRefScale * 1.006)); // fill the circular buffer for super smooth values

  if (millis() - newtime >= 1000)
  {
    newtime = millis();
    float avg = 0.0;
    // the following ensures using the right type for the index variable
    using index_t = decltype(buffer)::index_t;
    for (index_t i = 0; i < buffer.size(); i++)
    {
      avg += buffer[i] / buffer.size();
    }

    localVoltage0Struct.rearAuxBatt1V = avg;
    if (localVoltage0Struct.rearAuxBatt1V <= 16.00)
    {
      if (buffer.size() > 499)
      {
        if (avg / lastReading <= 0.99995)
        {
          localRear0Struct.rearDeviceState = "Discharging";
        }
        else if (avg / lastReading >= 1.00005)
        {
          localRear0Struct.rearDeviceState = "Charging";
        }
        else
        {
          localRear0Struct.rearDeviceState = "Stable";
        }
      }
      else
      {
        localRear0Struct.rearDeviceState = "Checking...";
      }
      lastReading = avg;
    }
    else
    {
      localRear0Struct.rearDeviceState = "Error!!!";
    }
    localRear0Struct.rearDeviceState = rearDeviceState;
    lv_label_set_text(ui_auxState, localRear0Struct.rearDeviceState.c_str());

    if ((abs(oldAuxVoltage-localVoltage0Struct.rearAuxBatt1V) > 0.002) || ((abs(oldAuxVoltage-localVoltage0Struct.rearAuxBatt1V) < -0.002))) // if a (large?) voltage change, send out a message
    {
      Serial.println("Voltage change detected on AuxBatt, sending message...");
      sendMessage();
    }
    oldAuxVoltage = localVoltage0Struct.rearAuxBatt1V;
  }
  
  String ui_auxBattVoltageLabelText = String(localVoltage0Struct.rearAuxBatt1V);
  lv_label_set_text(ui_auxBattVoltageLabel, ui_auxBattVoltageLabelText.c_str());
  if (localVoltage0Struct.rearAuxBatt1V < 11)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "FLAT!");
    lv_arc_set_value(ui_auxBattVoltageArc, 5);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (localVoltage0Struct.rearAuxBatt1V <= 12.0)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "9%");
    lv_arc_set_value(ui_auxBattVoltageArc, 9);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (localVoltage0Struct.rearAuxBatt1V<= 12.5)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "14%");
    lv_arc_set_value(ui_auxBattVoltageArc, 14);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (localVoltage0Struct.rearAuxBatt1V <= 12.8)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "17%");
    lv_arc_set_value(ui_auxBattVoltageArc, 17);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xF06319), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xF06319), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xF06319), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xF06319), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (localVoltage0Struct.rearAuxBatt1V <= 12.9)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "20%");
    lv_arc_set_value(ui_auxBattVoltageArc, 20);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xE8B23B), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xE8B23B), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xE8B23B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xE8B23B), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (localVoltage0Struct.rearAuxBatt1V <= 13.0)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "30%");
    lv_arc_set_value(ui_auxBattVoltageArc, 30);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xE8B23B), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xE8B23B), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xE8B23B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xE8B23B), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (localVoltage0Struct.rearAuxBatt1V <= 13.1)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "40%");
    lv_arc_set_value(ui_auxBattVoltageArc, 40);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xE3ED00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xE3ED00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xE3ED00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xE3ED00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (localVoltage0Struct.rearAuxBatt1V <= 13.15)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "70%");
    lv_arc_set_value(ui_auxBattVoltageArc, 70);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (localVoltage0Struct.rearAuxBatt1V <= 13.3)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "90%");
    lv_arc_set_value(ui_auxBattVoltageArc, 90);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (localVoltage0Struct.rearAuxBatt1V <= 13.4)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "99%");
    lv_arc_set_value(ui_auxBattVoltageArc, 99);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (localVoltage0Struct.rearAuxBatt1V <= 13.55)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "100%");
    lv_arc_set_value(ui_auxBattVoltageArc, 100);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (localVoltage0Struct.rearAuxBatt1V <= 16.00)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "100%");
    lv_arc_set_value(ui_auxBattVoltageArc, 100);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "V HIGH!");
    lv_arc_set_value(ui_auxBattVoltageArc, 100);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }


  if (localVoltage0Struct.rearAuxBatt1V < 11.00)
  {
    digitalWrite(hp1, 0);
    lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    digitalWrite(hp2, 0);
    lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    digitalWrite(lp1, 0);
    lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    digitalWrite(lp2, 0);
    lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else 
  {
    if (localRear0Struct.rearIO1 == 1)
    {
      digitalWrite(hp1, 1);
      lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else
    {
      digitalWrite(hp1, 0);
      lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    if (localRear0Struct.rearIO2 == 1)
    {
      digitalWrite(hp2, 1);
      lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else
    {
      digitalWrite(hp2, 0);
      lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    if (localRear0Struct.rearIO3 == 1)
    {
      digitalWrite(lp1, 1);
      lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else
    {
      digitalWrite(lp1, 0);
      lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    if (localRear0Struct.rearIO4 == 1)
    {
      digitalWrite(lp2, 1);
      lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    else
    {
      digitalWrite(lp2, 0);
      lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
  }
}

void loadPreferences()
{
  if (preferences.begin("basic", false))
  {
    if (preferences.getString("hp1Label", "None") != "None")
    {
      localRear0Struct.rearIO1Name = preferences.getString("hp1Label");
      lv_label_set_text(ui_hp1Label, localRear0Struct.rearIO1Name.c_str());
      lv_label_set_text(ui_ioLabel1, localRear0Struct.rearIO1Name.c_str());

      localRear0Struct.rearIO2Name = preferences.getString("hp2Label");
      lv_label_set_text(ui_hp2Label, localRear0Struct.rearIO2Name.c_str());
      lv_label_set_text(ui_ioLabel2, localRear0Struct.rearIO2Name.c_str());

      localRear0Struct.rearIO3Name = preferences.getString("lp1Label");
      lv_label_set_text(ui_lp1Label, localRear0Struct.rearIO3Name.c_str());
      lv_label_set_text(ui_ioLabel3, localRear0Struct.rearIO3Name.c_str());

      localRear0Struct.rearIO4Name = preferences.getString("lp2Label");
      lv_label_set_text(ui_lp2Label, localRear0Struct.rearIO4Name.c_str());
      lv_label_set_text(ui_ioLabel4, localRear0Struct.rearIO4Name.c_str());

      localRear0Struct.rearIO1 = preferences.getInt("hp1IOState", false);
      if (localRear0Struct.rearIO1 == 1)
      {
        lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_add_state(ui_io1, LV_STATE_CHECKED);
      }
      else
      {
        lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_clear_state(ui_io1, LV_STATE_CHECKED);
      }
      digitalWrite(hp1, localRear0Struct.rearIO1);

      localRear0Struct.rearIO2 = preferences.getInt("hp2IOState", false);
      if (localRear0Struct.rearIO2 == 1)
      {
        lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_add_state(ui_io2, LV_STATE_CHECKED);
      }
      else
      {
        lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_clear_state(ui_io2, LV_STATE_CHECKED);
      }
      digitalWrite(hp2, localRear0Struct.rearIO2);

      localRear0Struct.rearIO3 = preferences.getInt("lp1IOState", false);
      if (localRear0Struct.rearIO3 == 1)
      {
        lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_add_state(ui_io3, LV_STATE_CHECKED);
      }
      else
      {
        lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_clear_state(ui_io3, LV_STATE_CHECKED);
      }
      digitalWrite(lp1, localRear0Struct.rearIO3);

      localRear0Struct.rearIO4 = preferences.getInt("lp2IOState", false);
      if (localRear0Struct.rearIO4 == 1)
      {
        lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_add_state(ui_io4, LV_STATE_CHECKED);
      }
      else
      {
        lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_clear_state(ui_io4, LV_STATE_CHECKED);
      }
      digitalWrite(lp2, localRear0Struct.rearIO4);
    }
    else
    {
      preferences.putString("hp1Label", localRear0Struct.rearIO1Name);
      preferences.putString("hp2Label", localRear0Struct.rearIO2Name);
      preferences.putString("lp1Label", localRear0Struct.rearIO3Name);
      preferences.putString("lp2Label", localRear0Struct.rearIO4Name);

      preferences.putInt("hp1IOState", localRear0Struct.rearIO1);
      preferences.putInt("hp2IOState", localRear0Struct.rearIO2);
      preferences.putInt("lp1IOState", localRear0Struct.rearIO3);
      preferences.putInt("lp2IOState", localRear0Struct.rearIO4);
    }
  }
  preferences.end();
}

void setup()
{
  Serial.begin(115200); /* prepare for possible serial debug */

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
  
  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
  
  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  ui_init();

  loadPreferences();
  newtime = millis();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());


  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  sendMessage(); // request nodes to send sync messages ASAP as priorityMessage = 1

  Serial.println("Setup done");
}

void loop()
{
  lv_timer_handler(); /* let the GUI do its work */

  if (loopCounter % 3 == 0) // ~1 secs
  {
    checkData();
  }

  if (loopCounter % 910 == 0) // ~60 secs
  {
    Serial.println("Sending sync message!");
    sendMessage();
    loopCounter = 0;
  }
  loopCounter++;
  delay(5);
}
