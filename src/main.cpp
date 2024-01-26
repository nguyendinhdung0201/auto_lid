
#include <Wire.h>
#include "DFRobot_INA219.h"
#include <SimpleKalmanFilter.h>
#include <EEPROM.h>
#include <CAN.h>
#include <stdint.h>
#include <time.h>
SimpleKalmanFilter bo_loc(5, 5, 0.001);


#define L_PWM 17
#define R_PWM 16
#define LR_EN 18
#define BUTTON_CLOSE 19      // Button CLOSE
#define STEP 9569 // 4900  // 9580
#define SCL 33
#define SDA 32  
#define BUTTON_1 34       // Button OPEN
#define BUTTON_2 35       // Button CLOSE
#define U_STEP 2
#define control_key 26
#define BUTTON_KEY 27


#define TX_GPIO_NUM   21  // Connects to CTX
#define RX_GPIO_NUM   22  // Connects to CRX

/* -------------  ESP32 C3
  #define L_PWM 2
  #define R_PWM 1
  #define LR_EN 6
  #define Ch_A 7
  #define Ch_B 8
  #define STEP 9569 // 4900  // 9580
  #define SCL 10
  #define SDA 9
  #define BUTTON_1 18
  #define BUTTON_2  19
  #define U_STEP 1
  -------------- */

// Current sensor
DFRobot_INA219_IIC     ina219(&Wire, INA219_I2C_ADDRESS1);
// Revise the following two paramters according to actual reading of the INA219 and the multimeter
// for linearly calibration
float ina219Reading_mA = 1000;
float extMeterReading_mA = 1000;
float previous_yn = 200.0;  
float abs_previous;


long pos = 0;
volatile long counter = 0;
unsigned long time_ms;
unsigned long last_time=0;
int16_t u=0;
int16_t u_defauct=200;
int16_t u_calc = 0;

uint8_t open_speed=0, close_speed=0;
uint8_t new_open_speed=0, new_close_speed=0; 
bool check_stuck=false;
bool flag = false;
bool stuck=false;
bool ket1=false;
float xn1 = 0;
float yn1 = 0;
float pre = 0.0;
float kalman;
int dem=0;
int current,voltage;

uint16_t new_threshold;
uint16_t threshold_current;
uint16_t threshold_current_defauct=650;
int check_int,check_tt;


/*--------------------------------------------Config list status feedback Frame 1
    ID: 0x111A
             ID Package 0x11 (status feedback Frame1)
             ID Device  0x1A (LID)
    byte[0]  uint8_t status_data
                  uint8_t opened=0
                  uint8_t running=1
                  uint8_t closed=2
                  uint8_t stuck=101
                  uint8_t Initializing=3
    byte[1]  uint16_t current_threshold_upper
    byte[2]  uint16_t current_threshold_lower
    byte[3]  byte button_status
                  bit[2] Lock Switch
                  bit[3] Lid Close Button
    byte[4]  uint16_t current_upper
    byte[5]  uint16_t current_lower
    byte[6]  uint16_t voltage_upper
    byte[7]  uint16_t voltage_lower
--------------------------------------------*/



/*--------------------------------------------Config list status feedback Frame 2
    ID: 0x121A
            ID Package 0x12 (status feedback Frame 2)
            ID Device  0x1A (LID)
    byte[0] byte error
            bit[0] error CURRENT
            bit[1] error EEPROM
            bit[2] error Lock Switch
            bit[3] error Lock Control
--------------------------------------------*/


//////////////////////// Frame 1
uint32_t Start_Feedback1 = 0x111A; 
uint8_t status_data;
uint8_t button_status=0;
////////////////////////////////


//////////////////////// Frame 2
uint32_t Start_Feedback2 = 0x121A;
uint8_t error=0;
////////////////////////////////


///////////////////////// set Bit to High level i position
void setBitIndex_High(uint8_t* data_Byte, int index)
{
  if(index >=0 && index <8)
  {
    *data_Byte |= (1 << index);
  }
}


//////////////////////// set Bit to Low level i position
void setBitIndex_Low(uint8_t* data_Byte, int index)
{
  if(index >=0 && index <8)
  {
    *data_Byte &= ~(1 << index);
  }
}


/////////////////////// get Bit level i position
int getBitIndex(uint8_t get_error, int index)
{
  if(index >=0 && index <8)
  {
    return (get_error >> index) & 0x01;
  }
  return -1;
}

static inline int8_t sign(int val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

int soft_start(int u_set, float u)
{
  float d_u = u_set - u;

  int k = d_u / U_STEP;

  if (k == 0) return u_set;
  else
  {
    pre = sign(d_u) * U_STEP + u;
    return (int)(pre);
  }
}


int clip(int value, int min_, int max_)
{
  if ( value < min_ ) return min_;
  if ( value > max_ ) return max_;
  return value;
}

void set_motor(int pwm)
{
  if (pwm > 0)
  {
    digitalWrite(R_PWM, 1);
    digitalWrite(L_PWM, 0);
    ledcWrite(0, clip(pwm, 80, 255));
  }
  else if (pwm < 0)
  {
    digitalWrite(R_PWM, 0);
    digitalWrite(L_PWM, 1);
    ledcWrite(0, clip(abs(pwm), 80, 255));
  }
  else
  {
    digitalWrite(R_PWM, 0);
    digitalWrite(L_PWM, 0);
    ledcWrite(0, pwm);
  }
}
float calc_angle(long pos)
{
  return 2 * PI * pos / STEP;
}

/////////////// Lid status feedback Frame 1
void send_status_frame1(uint32_t Start_Feedback1_tem, uint8_t status_data_tem, uint16_t threshold_current_tem, uint8_t button_status_tem, uint16_t current_tem, uint16_t voltage_tem)
{
  CAN.beginExtendedPacket(Start_Feedback1_tem);
  CAN.write(status_data_tem);
  byte thresholdByte[2];
  thresholdByte[0]=(byte)(threshold_current_tem >>8 ) & 0xFF;  //// threshold_upper
  thresholdByte[1]=(byte)(threshold_current_tem & 0xFF);       //// threshold_lower
  for(int i=0;i<sizeof(thresholdByte);i++)
  {
    CAN.write(thresholdByte[i]);
  }

  CAN.write(button_status_tem);

  byte currentByte[2];
  currentByte[0]=(byte)(current_tem >>8 ) & 0xFF;      //// current_upper
  currentByte[1]=(byte)(current_tem & 0xFF);           //// current_lower
  for(int i=0;i<sizeof(currentByte);i++)
  {
    CAN.write(currentByte[i]);
  }

  byte voltageByte[2];
  voltageByte[0]=(byte)(voltage_tem >> 8 ) & 0xFF;     //// voltage_upper
  voltageByte[1]=(byte)(voltage_tem & 0xFF);           //// voltage_lower
  for(int i=0;i<sizeof(voltageByte);i++)
  {
    CAN.write(voltageByte[i]);
  }
  CAN.endPacket();
  delay(10);
}


/////////////// Lid status feedback Frame 2
void send_status_frame2(uint32_t Start_Feedback2_tem, uint8_t error_tem)
{
  CAN.beginExtendedPacket(Start_Feedback2);
  CAN.write(error_tem);
  CAN.endPacket();
  delay(10);
}




//////////////////////////////////////////////////////////////////////////////////////SET_UP

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);
  delay (1000);
  static unsigned long startTime = millis();
  EEPROM.begin(512);
  while(EEPROM.begin(512) == false) 
  {
  setBitIndex_High(&error,1);
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;
    if(elapsedTime >= 2000)
    {
      break;
    }
  }
  if(EEPROM.begin(512)==true)
  {
      if((EEPROM.read(0)^EEPROM.read(1)^EEPROM.read(2)^EEPROM.read(3))==(EEPROM.read(4)))
      {
        close_speed=EEPROM.read(0);
        open_speed=EEPROM.read(1);
        threshold_current=(EEPROM.read(2)<<8) | EEPROM.read(3);
      }
      if((EEPROM.read(0)^EEPROM.read(1)^EEPROM.read(2)^EEPROM.read(3))!=(EEPROM.read(4)))
      {
        EEPROM.write(0,u_defauct);
        EEPROM.write(1,u_defauct);
        EEPROM.write(2,(threshold_current_defauct >> 8) & 0xFF);
        EEPROM.write(3,(threshold_current_defauct & 0xFF));
        EEPROM.commit();
        close_speed=EEPROM.read(0);
        open_speed=EEPROM.read(1);
        threshold_current=(EEPROM.read(2)<<8) | EEPROM.read(3);
      }
  }
  else if(EEPROM.begin(512)==false)
  {
    close_speed=0;
    open_speed=0;
    threshold_current=0;
  }
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(LR_EN, OUTPUT);
  pinMode(control_key, OUTPUT);
  pinMode(BUTTON_KEY, INPUT_PULLUP);
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  pinMode(BUTTON_CLOSE, INPUT_PULLUP);
  pinMode(2,OUTPUT);
  ledcSetup(0, 1000, 8);
  ledcAttachPin(LR_EN, 0);
  delay(10);
  digitalWrite(L_PWM, 0);
  digitalWrite(R_PWM, 0);
  time_ms = millis();
  //------------ INA219----------------
  Wire.setPins(SDA, SCL);
  static unsigned long startTime1 = millis();
  ina219.begin();
  while(ina219.begin() == false) {
    setBitIndex_High(&error,0);
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime1;
    if(elapsedTime >=3000)
    {
      break;
    }
  }
  //Linear calibration
  ina219.linearCalibrate(/*The measured current before calibration*/ina219Reading_mA, /*The current measured by other current testers*/extMeterReading_mA);
  //------------------------------------

    
////////////////////////////////////////////////////// BEGIN CAN

   CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  // start the CAN bus at 500 kbps
  if (!CAN.begin (500E3)) {
    Serial.println ("Starting CAN failed!");
    while (1);
  }
  else {
    Serial.println ("CAN Initialized");
  }

////////////////////////////////////////////////////// RESET MOTOR
  if(EEPROM.begin(512)==true && ina219.begin()==true)
  {
    u=open_speed;
  }
  else
  {
    u=0;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void check_button()
{
  int button1 = digitalRead(BUTTON_1);
  int button2 = digitalRead(BUTTON_2);
  int button_key = digitalRead(BUTTON_KEY);
  int button_close = digitalRead(BUTTON_CLOSE);
  if(button1==1)
  {
    setBitIndex_High(&button_status,0);
    setBitIndex_Low(&button_status,1);
  }
  else if (button2==1)
  {
    setBitIndex_High(&button_status,1);
    setBitIndex_Low(&button_status,0);
  }
  else if (button_key==1)
  {
    setBitIndex_High(&button_status,2);
  }
  else if (button_key==0)
  {
    setBitIndex_Low(&button_status,2);
  }
  else if (button_close==1)
  {
    setBitIndex_High(&button_status,3);
  }
  else if (button_close==0)
  {
    setBitIndex_Low(&button_status,3);
  }
}

void loop() {
///////////////////////////////////////////////////////////////////////////////////////check CAN
int size_data = CAN.parsePacket();
if(size_data)
{
  if(CAN.available())
  {
    int pk_ID=(CAN.packetId() >> 8) & 0xFF;
    int dv_ID=CAN.packetId() & 0xFF;
    if(dv_ID==0x1A)
    {
      if(pk_ID==0x01 && u==0)
        {
          byte data_input[8]; // Giả sử độ dài tối đa của gói tin là 8 byte
          CAN.readBytes(data_input, size_data);
          int check_run=data_input[0];
          if(check_run==0)
            {
                u=close_speed*-1;
                stuck=false;
            }
          else if(check_run==1)
            {
                u=open_speed;
                stuck=false;
            }
        }
      else if(pk_ID==0x02)
             {
                byte data_input[8]; // Giả sử độ dài tối đa của gói tin là 8 byte
                CAN.readBytes(data_input, size_data);
                new_close_speed = data_input[0];
                new_open_speed = data_input[1];
                new_threshold=(data_input[2] << 8) | data_input[3];
                if((threshold_current != new_threshold) || (close_speed != new_close_speed) || (open_speed != new_open_speed))
                  {
                    threshold_current=new_threshold;
                    close_speed=new_close_speed;
                    open_speed=new_open_speed;
                    if(EEPROM.begin(512)==true)
                    {
                      setBitIndex_Low(&error,1);
                      EEPROM.write(0,data_input[0]);       //data close speed
                      EEPROM.write(1,data_input[1]);       //data open speed
                      EEPROM.write(2,data_input[2]);       //data threshold_lower
                      EEPROM.write(3,data_input[3]);       //data threshold_upper
                      EEPROM.write(4,data_input[4]);       //data checksum
                      EEPROM.commit(); 
                    }
                    else if(EEPROM.begin(512)==false)
                    {
                      setBitIndex_High(&error,1);
                    }
                  }
              
             }

    }
  } 
}
///////////////////////////////////////////////////////////////////////////////////////check BUTTON_CLOSE
    int button1 = digitalRead(BUTTON_1);
    int button2 = digitalRead(BUTTON_2);
    int button_key = digitalRead(BUTTON_KEY);
    if(digitalRead(BUTTON_CLOSE)==1 && u==0 && button1==0)
    { 
      if(millis()-last_time >= 50)
      {
        u=close_speed*-1;
        last_time=millis();
      }
    }
//////////////////////////////////////////////////////////////////////////////////////check u
if(u>0)
  {
    if(button2==0)
    {
      digitalWrite(control_key,HIGH);
      if(button_key==1)
        {
          setBitIndex_Low(&error,3);
        }
      if(button_key==0)
        {
          u=0;
          status_data=101;
          setBitIndex_High(&error,3);
        }
    }
    else if(button2==1)
    {
      stuck=false;
      u = 0;
      u_calc = 0;
      pre = 0;
      flag = true;
      status_data=0;
    }

  }
if(u<0 && button1==1)
{
    stuck=false;
    u = 0;
    u_calc = 0;
    pre = 0;
    flag = true;
    status_data=2;
}

 /////////////////////////////////////////////////////////////////////////////////////////update u
  if(u<0 && u!=close_speed*-1)
  {
    u=close_speed*-1;
  }
  if(u>0 && u!=open_speed)
  {
    u=open_speed;
  }
/////////////////////////////////////////////////////////////////motor_control
  u_calc = soft_start(u, pre);
  if (flag)
  {
    u_calc = 0;
    pre = 0;
    flag = false;
  }
  set_motor(u_calc);

//////////////////////////////////////////////////////////////////running
  if(button1==0&&button2==0&&stuck==false)
  {
   status_data=1;
  }


  //////////////////////////////////////////////////////////////check_current
  if (millis() - time_ms  > 10)
  {
      // float  xn = ina219.getCurrent_mA();
      if(ina219.getCurrent_mA()==0.0)
        {
          setBitIndex_High(&error,0);
          ina219.begin();
        }
      else
        {
          setBitIndex_Low(&error,0);
        }
      float  yn = bo_loc.updateEstimate(ina219.getCurrent_mA());
      float  vol = ina219.getBusVoltage_V();
      if(yn<0)
      {
        yn=yn*-1.0;
      }
      current=(int)(yn);
      voltage=(int)(vol*10);
      if(yn>=threshold_current)
      {
        set_motor(0);
        u=0;
        stuck=true;
        status_data=101;
      } 
      time_ms = millis();
  }
/////////////////////////////////////////////////////////////////// close_KEY
  if(u==0)
  {
    if(button_key==1)
    {
     setBitIndex_High(&error,2);
     digitalWrite(control_key,LOW);
    }
    if(button_key==0)
    {
     setBitIndex_Low(&error,2);
    }
  }

  check_button();
/////////////////////////////////////////////////////////////////// send status
  send_status_frame1(Start_Feedback1,status_data,threshold_current,button_status,current,voltage);   
  send_status_frame2(Start_Feedback2,error);
  delay(1);
}
