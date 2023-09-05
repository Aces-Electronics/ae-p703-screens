#define LGFX_USE_V1

#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <driver/adc.h>
#include "lvgl.h"
#include "ui.h"

#include "sdkconfig.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h" //https://www.freertos.org/a00106.html
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include <driver/adc.h>
#include <SimpleKalmanFilter.h> //https://www.arduino.cc/reference/en/libraries/simplekalmanfilter/

// GPIO definitions
const int vin = 14; // input

const int hp1 = 10; // outputs
const int hp2 = 11;
const int lp1 = 12;
const int lp2 = 13;

bool hp1IOState = 0;
bool hp2IOState = 0;
bool lp1IOState = 0;
bool lp2IOState = 0;

class LGFX : public lgfx::LGFX_Device
{

lgfx::Panel_ST7796      _panel_instance;
lgfx::Bus_Parallel8 _bus_instance;   // 8ビットパラレルバスのインスタンス (ESP32のみ)
lgfx::Light_PWM     _light_instance;
lgfx::Touch_FT5x06           _touch_instance; // FT5206, FT5306, FT5406, FT6206, FT6236, FT6336, FT6436

public:
  LGFX(void)
  {
    { // バス制御の設定を行います。
      auto cfg = _bus_instance.config();    // バス設定用の構造体を取得します。
// 8ビットパラレルバスの設定
      //cfg.i2s_port = I2S_NUM_0;     // 使用するI2Sポートを選択 (I2S_NUM_0 or I2S_NUM_1) (ESP32のI2S LCDモードを使用します)
      cfg.freq_write = 20000000;    // 送信クロック (最大20MHz, 80MHzを整数で割った値に丸められます)
      cfg.pin_wr =  47;              // WR を接続しているピン番号
      cfg.pin_rd =  -1;              // RD を接続しているピン番号
      cfg.pin_rs = 0;              // RS(D/C)を接続しているピン番号
      cfg.pin_d0 = 9;              // D0を接続しているピン番号
      cfg.pin_d1 = 46;              // D1を接続しているピン番号
      cfg.pin_d2 = 3;              // D2を接続しているピン番号
      cfg.pin_d3 = 8;              // D3を接続しているピン番号
      cfg.pin_d4 = 18;              // D4を接続しているピン番号
      cfg.pin_d5 = 17;              // D5を接続しているピン番号
      cfg.pin_d6 = 16;              // D6を接続しているピン番号
      cfg.pin_d7 = 15;              // D7を接続しているピン番号
      _bus_instance.config(cfg);    // 設定値をバスに反映します。
      _panel_instance.setBus(&_bus_instance);      // バスをパネルにセットします。
    }

    { // 表示パネル制御の設定を行います。
      auto cfg = _panel_instance.config();    // 表示パネル設定用の構造体を取得します。

      cfg.pin_cs           =    -1;  // CSが接続されているピン番号   (-1 = disable)
      cfg.pin_rst          =    4;  // RSTが接続されているピン番号  (-1 = disable)
      cfg.pin_busy         =    -1;  // BUSYが接続されているピン番号 (-1 = disable)

      // ※ 以下の設定値はパネル毎に一般的な初期値が設定されていますので、不明な項目はコメントアウトして試してみてください。

      cfg.panel_width      =   320;  // 実際に表示可能な幅
      cfg.panel_height     =   480;  // 実際に表示可能な高さ
      cfg.offset_x         =     0;  // パネルのX方向オフセット量
      cfg.offset_y         =     0;  // パネルのY方向オフセット量
      cfg.offset_rotation  =     0;  // 回転方向の値のオフセット 0~7 (4~7は上下反転)
      cfg.dummy_read_pixel =     8;  // ピクセル読出し前のダミーリードのビット数
      cfg.dummy_read_bits  =     1;  // ピクセル以外のデータ読出し前のダミーリードのビット数
      cfg.readable         =  true;  // データ読出しが可能な場合 trueに設定
      cfg.invert           = true;  // パネルの明暗が反転してしまう場合 trueに設定
      cfg.rgb_order        = false;  // パネルの赤と青が入れ替わってしまう場合 trueに設定
      cfg.dlen_16bit       = false;  // 16bitパラレルやSPIでデータ長を16bit単位で送信するパネルの場合 trueに設定
      cfg.bus_shared       =  true;  // SDカードとバスを共有している場合 trueに設定(drawJpgFile等でバス制御を行います)

// 以下はST7735やILI9163のようにピクセル数が可変のドライバで表示がずれる場合にのみ設定してください。
//    cfg.memory_width     =   240;  // ドライバICがサポートしている最大の幅
//    cfg.memory_height    =   320;  // ドライバICがサポートしている最大の高さ

      _panel_instance.config(cfg);
    }

//*
    { // バックライト制御の設定を行います。（必要なければ削除）
      auto cfg = _light_instance.config();    // バックライト設定用の構造体を取得します。

      cfg.pin_bl = 45;              // バックライトが接続されているピン番号
      cfg.invert = false;           // バックライトの輝度を反転させる場合 true
      cfg.freq   = 44100;           // バックライトのPWM周波数
      cfg.pwm_channel = 7;          // 使用するPWMのチャンネル番号

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);  // バックライトをパネルにセットします。
    }
//*/

//*
    { // タッチスクリーン制御の設定を行います。（必要なければ削除）
      auto cfg = _touch_instance.config();

      cfg.x_min      = 0;    // タッチスクリーンから得られる最小のX値(生の値)
      cfg.x_max      = 319;  // タッチスクリーンから得られる最大のX値(生の値)
      cfg.y_min      = 0;    // タッチスクリーンから得られる最小のY値(生の値)
      cfg.y_max      = 479;  // タッチスクリーンから得られる最大のY値(生の値)
      cfg.pin_int    = 7;   // INTが接続されているピン番号
      cfg.bus_shared = true; // 画面と共通のバスを使用している場合 trueを設定
      cfg.offset_rotation = 0;// 表示とタッチの向きのが一致しない場合の調整 0~7の値で設定
// I2C接続の場合
      cfg.i2c_port = 1;      // 使用するI2Cを選択 (0 or 1)
      cfg.i2c_addr = 0x38;   // I2Cデバイスアドレス番号
      cfg.pin_sda  = 6;     // SDAが接続されているピン番号
      cfg.pin_scl  = 5;     // SCLが接続されているピン番号
      cfg.freq = 400000;     // I2Cクロックを設定

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);  // タッチスクリーンをパネルにセットします。
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

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.writePixels((lgfx::rgb565_t *)&color_p->full, w * h);
  tft.endWrite();
  lv_disp_flush_ready(disp);
}

void my_touch_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    uint16_t touchX, touchY;
    bool touched = tft.getTouch(&touchX, &touchY);
    if (!touched) { data->state = LV_INDEV_STATE_REL; }
    else {
      data->state = LV_INDEV_STATE_PR;
      data->point.x = touchX;
      data->point.y = touchY;
    }
}


static lv_style_t label_style;
static lv_obj_t *headerLabel;
static lv_obj_t * label;

void set_screen_brightness(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target(e);

    int brightness = lv_slider_get_value(slider);
	  if (brightness < 10) brightness = 10;
    tft.setBrightness(brightness * 2.55);
}

void fReadBattery( void * parameter )
{
  int adcValue = 0;
  //https://ohmslawcalculator.com/voltage-divider-calculator
  const float r1 = 82000.0f; // R1 in ohm, 50K. For a LiFeP04 battery under charge
  const float r2 = 16000.0f; // R2 in ohm, 10k, make it a habit to use a 10K for R2
  float Vbatt = 0.0f;
  int read_raw;
  int printCount = 0;
  float vRefScale = (3.0f / 4096.0f) * ((r1 + r2) / r2);
  uint64_t TimePastKalman  = esp_timer_get_time(); // used by the Kalman filter UpdateProcessNoise, time since last kalman calculation
  SimpleKalmanFilter KF_ADC_b( 1.0f, 1.0f, .01f );
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1000; //delay for mS *NON-BLOCKING DELAY *
  for (;;)
  {
    adc2_get_raw(ADC2_CHANNEL_6, ADC_WIDTH_BIT_12, &read_raw); //read and discard
    adc2_get_raw(ADC2_CHANNEL_6, ADC_WIDTH_BIT_12, &adcValue); //take a raw ADC reading
    KF_ADC_b.setProcessNoise( (esp_timer_get_time() - TimePastKalman) / 1000000.0f ); //get time, in microsecods, since last readings
    adcValue = KF_ADC_b.updateEstimate( adcValue ); // apply simple Kalman filter
    Vbatt = adcValue * vRefScale;
    printCount++;
    if ( printCount == 3 )
    {
      //log_i( "Vbatt %f", Vbatt );// you may have to edit the code to use serial print.
      //log_i( "Vbatt_raw %f", read_raw );// you may have to edit the code to use serial print.
      printCount = 0;
      Serial.print("VBatt: "); Serial.println(Vbatt);
    }
    TimePastKalman = esp_timer_get_time(); // time of update complete
    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    //log_i( "fReadBattery %d",  uxTaskGetStackHighWaterMark( NULL ) );
  }
  vTaskDelete( NULL );
}

void toggle_outputs()
{
  if (hp1IOState) {
    digitalWrite(hp1, LOW);
    digitalWrite(hp2, LOW);
    digitalWrite(lp1, LOW);
    digitalWrite(lp2, LOW);
    Serial.println("LOW");
  }
  else 
  {
    digitalWrite(hp1, HIGH);
    digitalWrite(hp2, HIGH);
    digitalWrite(lp1, HIGH);
    digitalWrite(lp2, HIGH);
    Serial.println("HIGH");
  }
  hp1IOState = !hp1IOState;
  delay(2000);
}

void setup() {
  Serial.begin(115200);

  //xTaskCreatePinnedToCore( fReadBattery, "fReadBattery", 4000, NULL, 3, NULL, 1 );
  
  pinMode(hp1, OUTPUT);
  pinMode(hp2, OUTPUT);
  pinMode(lp1, OUTPUT);
  pinMode(lp2, OUTPUT);

  tft.begin();
  tft.setRotation(1); // 3 = upside down
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
}


void loop() {
  lv_timer_handler();
  //toggle_outputs();
  delay(5);
}
