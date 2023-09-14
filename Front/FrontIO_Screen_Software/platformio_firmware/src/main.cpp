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


String frontDeviceState = "Stable";

int rawValue = 0;
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
        lv_label_set_text(ui_hp1Label, localRear0Struct.rearIO1Name.c_str());
      } 

      if (remoteRear0Struct.rearIO2 != -1)
      {
        localRear0Struct.rearIO2 = remoteRear0Struct.rearIO2; // rear: basic/pro IO1
      } 

      if (remoteRear0Struct.rearIO2Name != "-1")
      {
        localRear0Struct.rearIO2Name = remoteRear0Struct.rearIO2Name;
        lv_label_set_text(ui_hp2Label, localRear0Struct.rearIO2Name.c_str());
      } 
       
      if (remoteRear0Struct.rearIO3 != -1)
      {
        localRear0Struct.rearIO3 = remoteRear0Struct.rearIO3; // rear: basic/pro IO1
      } 

      if (remoteRear0Struct.rearIO3Name != "-1")
      {
        localRear0Struct.rearIO3Name = remoteRear0Struct.rearIO3Name;
        lv_label_set_text(ui_lp1Label, localRear0Struct.rearIO3Name.c_str());
      } 

      if (remoteRear0Struct.rearIO4 != -1)
      {
        localRear0Struct.rearIO4 = remoteRear0Struct.rearIO4; // rear: basic/pro IO1
      }

      if (remoteRear0Struct.rearIO4Name != "-1")
      {
        localRear0Struct.rearIO4Name = remoteRear0Struct.rearIO4Name;
        lv_label_set_text(ui_lp2Label, localRear0Struct.rearIO4Name.c_str());
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
    localRear0Struct.rearIO1 = !localRear0Struct.rearIO1; // ToDo: check this logic
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
    localRear0Struct.rearIO3 = !localRear0Struct.rearIO3;
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
  }
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
  String ui_auxBattVoltageLabelText = String(localVoltage0Struct.rearAuxBatt1V);
  ui_auxBattVoltageLabelText.concat("V");
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
  else if (localVoltage0Struct.rearAuxBatt1V <= 12.5)
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
    lv_obj_set_style_text_color(ui_hp1Label, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_hp2Label, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lp1Label, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lp2Label, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  else 
  {
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

  //sendMessage(); // request nodes to send sync messages ASAP as priorityMessage = 1

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
    //sendMessage();
    loopCounter = 0;
  }
  loopCounter++;
  delay(5);
}

//------------------------------------------------------------------------