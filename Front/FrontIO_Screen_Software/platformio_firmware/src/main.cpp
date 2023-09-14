// FRONT IO Basic

#define TOUCH_MODULES_CST_SELF
#include <Arduino.h>
#include <lvgl.h>
#include <driver/adc.h>
#include <nvs_flash.h>
#include <Preferences.h>
#include <CircularBuffer.h>
#include "ui.h"
#include <Arduino_GFX_Library.h>
#include <TouchLib.h>
#include <esp_now.h>
#include <WiFi.h>

#include "pin_config.h"

// GPIO definitions
const int vin = 14;
const int hp1 = 10; // outputs
const int hp2 = 11;
const int lp1 = 12;
const int lp2 = 13;

String hp1Label = "HP1";
String hp2Label = "HP2";
String lp1Label = "LP1";
String lp2Label = "LP2";

String frontDeviceState = "Stable";

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
char vinResult[8] = "WAIT!";

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

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message_in0 {
  int messageID = 0;
  int incomingio1 = -1; // rear: basic/pro io1
  char incomingio1Name[7];
  int incomingio2 = -1; // rear: basic/pro io2
  char incomingio2Name[7];
  int incomingio3 = -1; // rear: basic/pro io3
  char incomingio3Name[7];
  int incomingio4 = -1; // rear: basic/pro io4
  char incomingio4Name[7];
  int incomingio5 = -1; // rear: pro io5
  char incomingio5Name[7];
  int incomingio6 = -1; // rear: pro io6
  char incomingio6Name[7];
  int incomingio7 = -1; // rear: pro io7
  char incomingio7Name[7];
  int incomingio8 = -1; // rear: pro io8
  char incomingio8Name[7];
  int incomingio9 = -1; // rear: pro io9
  char incomingio9Name[7];
  int incomingio10 = -1; // rear: pro io10
  char incomingio10Name[7];
  int incomingio11 = -1; // aux: io11
  char incomingio11Name[7];
  int incomingio12 = -1; // aux: io12
  char incomingio12Name[7];
  int incomingio13 = -1; // aux: io13
  char incomingio13Name[7];
  int incomingio14 = -1; // aux: io14
  char incomingio14Name[7];
  int incomingio15 = -1; // aux: io15
  char incomingio15Name[7];
  int incomingio16 = -1; // aux: io16
  char incomingio16Name[7];
  int incomingio17 = -1; // aux: io17
  char incomingio17Name[7];
  int incomingio18 = -1; // aux: io18
  char incomingio18Name[7];
  int incomingio19 = -1; // aux: io19
  char incomingio19Name[7];
} struct_message_in0;

typedef struct struct_message_in1 {
  int messageID = 1;
  int incomingio20 = -1; // aux: io20
  char incomingio20Name[7];
  int incomingio21 = -1; // front: basic/pro io1
  char incomingio21Name[7];
  int incomingio22 = -1; // front: basic/pro io2
  char incomingio22Name[7];
  int incomingio23 = -1; // front: basic/pro io3
  char incomingio23Name[7];
  int incomingio24 = -1; // front: basic/pro io4
  char incomingio24Name[7];
  int incomingio25 = -1; // front: pro io5
  char incomingio25Name[7];
  int incomingio26 = -1; // front: pro io6
  char incomingio26Name[7];
  int incomingio27 = -1; // front: pro io7
  char incomingio28Name[7];
  int incomingio28 = -1; // front: pro io8
  char incomingio29Name[7];
  int incomingio29 = -1; // front: pro io9
  char incomingio30Name[7];
  int incomingio30 = -1; // front: pro io10
  float incomingFrontMainBatt1V = -1;
  float incomingFrontAuxBatt1V = -1;
  float incomingRearMainBatt1V = -1;
  float incomingRearAuxBatt1V = -1;
  float incomingFrontMainBatt1I = -1;
  float incomingFrontAuxBatt1I = -1;
  float incomingRearMainBatt1I = -1;
  float incomingRearAuxBatt1I = -1; 
  String rearDeviceState = "WAIT!";
} struct_message_in1;

struct_message_priority priorityMessageStruct;

// Create a struct_message called localReadings to hold sensor readings
struct_message_in0 localReadings0Struct;
struct_message_in1 localReadings1Struct;

// Create a struct_message to hold incoming sensor readings
struct_message_in0 remoteReadings0Struct;
struct_message_in1 remoteReadings1Struct;

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
    esp_err_t result0 = esp_now_send(broadcastAddress, (uint8_t *) &localReadings0Struct, sizeof(localReadings0Struct));
    if (result0 == ESP_OK) {
      Serial.println("Sent message 0 with success");
    }
    else {
      Serial.println("Error sending the data");
    }
    delay(200);
    esp_err_t result1 = esp_now_send(broadcastAddress, (uint8_t *) &localReadings1Struct, sizeof(localReadings1Struct));
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
      memcpy(&remoteReadings0Struct, incomingData, sizeof(remoteReadings0Struct));
      Serial.print("0: Bytes received: "); Serial.println(len);
      if (remoteReadings0Struct.incomingio1 != -1)
      {
        localReadings0Struct.incomingio1 = remoteReadings0Struct.incomingio1; // rear: basic/pro io1
      }

      localReadings0Struct.incomingio1Name[0] = remoteReadings0Struct.incomingio1Name[0];
      localReadings0Struct.incomingio2 = remoteReadings0Struct.incomingio2; // rear: basic/pro io2
      localReadings0Struct.incomingio2Name[0] = remoteReadings0Struct.incomingio2Name[0];
      localReadings0Struct.incomingio3 = remoteReadings0Struct.incomingio3; // rear: basic/pro io3
      localReadings0Struct.incomingio3Name[0] = remoteReadings0Struct.incomingio3Name[0];
      localReadings0Struct.incomingio4 = remoteReadings0Struct.incomingio4; // rear: basic/pro io4
      localReadings0Struct.incomingio4Name[0] = remoteReadings0Struct.incomingio4Name[0];
      localReadings0Struct.incomingio5 = remoteReadings0Struct.incomingio5; // rear: pro io5
      localReadings0Struct.incomingio5Name[0] = remoteReadings0Struct.incomingio5Name[0];
      localReadings0Struct.incomingio6 = remoteReadings0Struct.incomingio6; // rear: pro io6
      localReadings0Struct.incomingio6Name[0] = remoteReadings0Struct.incomingio6Name[0];
      localReadings0Struct.incomingio7 = remoteReadings0Struct.incomingio7; // rear: pro io7
      localReadings0Struct.incomingio7Name[0] = remoteReadings0Struct.incomingio7Name[0];
      localReadings0Struct.incomingio8 = remoteReadings0Struct.incomingio8; // rear: pro io8
      localReadings0Struct.incomingio8Name[0] = remoteReadings0Struct.incomingio8Name[0];
      localReadings0Struct.incomingio9 = remoteReadings0Struct.incomingio9; // rear: pro io9
      localReadings0Struct.incomingio9Name[0] = remoteReadings0Struct.incomingio9Name[0];
      localReadings0Struct.incomingio10 = remoteReadings0Struct.incomingio10; // rear: pro io10
      localReadings0Struct.incomingio10Name[0] = remoteReadings0Struct.incomingio10Name[0];
      localReadings0Struct.incomingio11 = remoteReadings0Struct.incomingio11; // aux: io11
      localReadings0Struct.incomingio11Name[0] = remoteReadings0Struct.incomingio11Name[0];
      localReadings0Struct.incomingio12 = remoteReadings0Struct.incomingio12; // aux: io12
      localReadings0Struct.incomingio12Name[0] = remoteReadings0Struct.incomingio12Name[0];
      localReadings0Struct.incomingio13 = remoteReadings0Struct.incomingio13; // aux: io13
      localReadings0Struct.incomingio13Name[0] = remoteReadings0Struct.incomingio13Name[0];
      localReadings0Struct.incomingio14 = remoteReadings0Struct.incomingio14; // aux: io14
      localReadings0Struct.incomingio14Name[0] = remoteReadings0Struct.incomingio14Name[0];
      localReadings0Struct.incomingio15 = remoteReadings0Struct.incomingio15; // aux: io15
      localReadings0Struct.incomingio15Name[0] = remoteReadings0Struct.incomingio15Name[0];
      localReadings0Struct.incomingio16 = remoteReadings0Struct.incomingio16; // aux: io16
      localReadings0Struct.incomingio16Name[0] = remoteReadings0Struct.incomingio16Name[0];
      localReadings0Struct.incomingio17 = remoteReadings0Struct.incomingio17; // aux: io17
      localReadings0Struct.incomingio17Name[0] = remoteReadings0Struct.incomingio17Name[0];
      localReadings0Struct.incomingio18 = remoteReadings0Struct.incomingio18; // aux: io18
      localReadings0Struct.incomingio18Name[0] = remoteReadings0Struct.incomingio18Name[0];
      localReadings0Struct.incomingio19 = remoteReadings0Struct.incomingio19; // aux: io19
      localReadings0Struct.incomingio19Name[0] = remoteReadings0Struct.incomingio19Name[0];
      break;

    case 1 :
      memcpy(&remoteReadings1Struct, incomingData, sizeof(remoteReadings1Struct));
      Serial.print("1: Bytes received: "); Serial.println(len);
      localReadings1Struct.incomingio20 = remoteReadings1Struct.incomingio20; // aux: io20
      localReadings1Struct.incomingio21 = remoteReadings1Struct.incomingio21; // front: basic/pro io1
      localReadings1Struct.incomingio22 = remoteReadings1Struct.incomingio22; // front: basic/pro io2
      localReadings1Struct.incomingio23 = remoteReadings1Struct.incomingio23; // front: basic/pro io3
      localReadings1Struct.incomingio24 = remoteReadings1Struct.incomingio24; // front: basic/pro io4
      localReadings1Struct.incomingio25 = remoteReadings1Struct.incomingio25; // front: pro io5
      localReadings1Struct.incomingio26 = remoteReadings1Struct.incomingio26; // front: pro io6
      localReadings1Struct.incomingio27 = remoteReadings1Struct.incomingio27; // front: pro io7
      localReadings1Struct.incomingio28 = remoteReadings1Struct.incomingio28; // front: pro io8
      localReadings1Struct.incomingio29 = remoteReadings1Struct.incomingio29; // front: pro io9
      localReadings1Struct.incomingio30 = remoteReadings1Struct.incomingio30; // front: pro io10
      localReadings1Struct.incomingFrontMainBatt1V = remoteReadings1Struct.incomingFrontMainBatt1V;
      localReadings1Struct.incomingFrontAuxBatt1V = remoteReadings1Struct.incomingFrontAuxBatt1V;
      localReadings1Struct.incomingRearMainBatt1V = remoteReadings1Struct.incomingRearMainBatt1V;
      localReadings1Struct.incomingRearAuxBatt1V = remoteReadings1Struct.incomingRearAuxBatt1V;
      localReadings1Struct.incomingFrontMainBatt1I = remoteReadings1Struct.incomingFrontMainBatt1I;
      localReadings1Struct.incomingFrontAuxBatt1I = remoteReadings1Struct.incomingFrontAuxBatt1I;
      localReadings1Struct.incomingRearMainBatt1I = remoteReadings1Struct.incomingRearMainBatt1I;
      localReadings1Struct.incomingRearAuxBatt1I = remoteReadings1Struct.incomingRearAuxBatt1I;
      localReadings1Struct.rearDeviceState = remoteReadings1Struct.rearDeviceState;
      lv_label_set_text(ui_auxState, localReadings1Struct.rearDeviceState.c_str());

      auxVoltage = localReadings1Struct.incomingrearAuxBatt1V;
      dtostrf(auxVoltage, 6, 2, vinResult);
      char tmp[2] = "V";
      strcat(vinResult, tmp);
      break;

      case 255 : // message ID 255: means that a device has rebooted and needs data outside the sync window
      sendMessage();
      break;
  }
}

static const uint16_t screenWidth = 320;
static const uint16_t screenHeight = 170;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];

Arduino_DataBus *bus = new Arduino_ESP32LCD8(47 /* DC */, 46 /* CS */, 38 /* WR */, 39 /* RD */, 9 /* D0 */, 10 /* D1 */, 11 /* D2 */, 12 /* D3 */,
                                             13 /* D4 */, 14 /* D5 */, 15 /* D6 */, 16 /* D7 */);

Arduino_GFX *gfx = new Arduino_ST7789(bus, 48 /* RST */, 0 /* rotation */, true /* IPS */, 170 /* width */, 320 /* height */, 35 /* col offset 1 */,
                                      0 /* row offset 1 */, 35 /* col offset 2 */, 0 /* row offset 2 */);

TouchLib touch(Wire, PIN_IIC_SDA, PIN_IIC_SCL, CTS820_SLAVE_ADDRESS, PIN_TOUCH_RES);

int left_num = 0;
int right_num = 0;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
    gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

    lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
    uint16_t touchX = 0, touchY = 0;

    if (touch.read())
    {
        data->state = LV_INDEV_STATE_PR;

        uint8_t n = touch.getPointNum();
        for (uint8_t i = 0; i < n; i++)
        {
            TP_Point t = touch.getPoint(i);
            int temp_x = t.y;
            int temp_y = map(t.x, 0, 150, 170, 0);

            data->point.x = temp_x;
            data->point.y = temp_y;
            break;
        }
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }
}

void savePreferences()
{
  Serial.println("saving to flash");
  preferences.begin("basic", false);
  preferences.end();
}

void set_screen_brightness(lv_event_t *e)
{
  lv_obj_t *slider = lv_event_get_target(e);

  int brightness = lv_slider_get_value(slider);
  if (brightness < 10)
    brightness = 10;
  //tft.setBrightness(brightness * 2.55);
}

void hp1ToggleFunction(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    localReadings0Struct.incomingio1 = !localReadings0Struct.incomingio1; // ToDo: check this logic
    if (localReadings0Struct.incomingio1 == 1)
    {
      lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io1, LV_STATE_CHECKED);
    }
    else
    {
      lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io1, LV_STATE_CHECKED);
    }
  }
}

void hp2ToggleFunction(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    localReadings0Struct.incomingio2 = !localReadings0Struct.incomingio2;
    if (localReadings0Struct.incomingio2 == 1)
    {
      lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io2, LV_STATE_CHECKED);
    }
    else
    {
      lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io2, LV_STATE_CHECKED);
    }
  }
}

void lp1ToggleFunction(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    localReadings0Struct.incomingio3 = !localReadings0Struct.incomingio3;
    if (localReadings0Struct.incomingio3 == 1)
    {
      lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io3, LV_STATE_CHECKED);
    }
    else
    {
      lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io3, LV_STATE_CHECKED);
    }
  }
}

void lp2ToggleFunction(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    localReadings0Struct.incomingio4 = !localReadings0Struct.incomingio4;
    if (localReadings0Struct.incomingio4 == 1)
    {
      lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io4, LV_STATE_CHECKED);
    }
    else
    {
      lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io4, LV_STATE_CHECKED);
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

void checkData()
{
  lv_label_set_text(ui_auxBattVoltageLabel, vinResult);
  if (auxVoltage < 11)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "FLAT!");
    lv_arc_set_value(ui_auxBattVoltageArc, 5);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage <= 12.0)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "9%");
    lv_arc_set_value(ui_auxBattVoltageArc, 9);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage <= 12.5)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "14%");
    lv_arc_set_value(ui_auxBattVoltageArc, 14);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xFF0000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage <= 12.8)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "17%");
    lv_arc_set_value(ui_auxBattVoltageArc, 17);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xF06319), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xF06319), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xF06319), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xF06319), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage <= 12.9)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "20%");
    lv_arc_set_value(ui_auxBattVoltageArc, 20);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xE8B23B), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xE8B23B), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xE8B23B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xE8B23B), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage <= 13.0)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "30%");
    lv_arc_set_value(ui_auxBattVoltageArc, 30);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xE8B23B), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xE8B23B), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xE8B23B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xE8B23B), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage <= 13.1)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "40%");
    lv_arc_set_value(ui_auxBattVoltageArc, 40);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0xE3ED00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0xE3ED00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0xE3ED00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0xE3ED00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage <= 13.15)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "70%");
    lv_arc_set_value(ui_auxBattVoltageArc, 70);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage <= 13.3)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "90%");
    lv_arc_set_value(ui_auxBattVoltageArc, 90);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage <= 13.4)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "99%");
    lv_arc_set_value(ui_auxBattVoltageArc, 99);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage <= 13.55)
  {
    lv_label_set_text(ui_auxBattPercentageLabel, "100%");
    lv_arc_set_value(ui_auxBattVoltageArc, 100);
    lv_obj_set_style_arc_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_auxBattVoltageArc, lv_color_hex(0x00FF00), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattPercentageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_auxBattVoltageLabel, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else if (auxVoltage <= 16.00)
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
  
  
  if (auxVoltage < 11.00)
  {
    lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else 
  {
    if (localReadings0Struct.incomingio1  == 1)
    {
      lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io1, LV_STATE_CHECKED);
    }
    else
    {
      lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io1, LV_STATE_CHECKED);
    }

    if (localReadings0Struct.incomingio2 == 1)
    {
      lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io2, LV_STATE_CHECKED);
    }
    else
    {
      lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io2, LV_STATE_CHECKED);
    }

    if (localReadings0Struct.incomingio3 == 1)
    {
      lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io3, LV_STATE_CHECKED);
    }
    else
    {
      lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io3, LV_STATE_CHECKED);
    }

    if (localReadings0Struct.incomingio4 == 1)
    {
      lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_add_state(ui_io4, LV_STATE_CHECKED);
    }
    else
    {
      lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_obj_clear_state(ui_io4, LV_STATE_CHECKED);
    }
  }
}

void loadPreferences()
{
  if (preferences.begin("basic", false))
  {
    Serial.println("Pref load");
  }

  preferences.end();
}

void setup()
{
  Serial.begin(115200); /* prepare for possible serial debug */

  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);

  pinMode(PIN_LCD_RST, OUTPUT);
  digitalWrite(PIN_LCD_RST, HIGH);

  pinMode(PIN_TOUCH_RES, OUTPUT);
  digitalWrite(PIN_TOUCH_RES, LOW);
  delay(500);
  digitalWrite(PIN_TOUCH_RES, HIGH);

  ledcSetup(0, 2000, 8);
  ledcAttachPin(PIN_LCD_BL, 0);
  ledcWrite(0, 255);

  Wire.begin(PIN_IIC_SDA, PIN_IIC_SCL);
  touch.init();

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  gfx->begin();
  gfx->setRotation(1);

  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);

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
  
  //loadPreferences();
  newtime = millis();

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
  checkData();
  if (loopCounter % 910 == 0) // ~60 secs
  {
    Serial.println("Sending sync message!");
    //sendMessage();
    loopCounter = 0;
  }
  loopCounter++;
  delay(5);
}

//------------------------------------------------------------------------