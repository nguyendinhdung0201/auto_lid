
// #include <Wire.h>
// #include "DFRobot_INA219.h"
// #include <SimpleKalmanFilter.h>
// #include <EEPROM.h>
// #include <CAN.h>
// #include <stdint.h>
// #include <time.h>
// SimpleKalmanFilter bo_loc(5, 5, 0.001);


// #define L_PWM 17
// #define R_PWM 16
// #define LR_EN 18
// #define BUTTON_RST 19      // Button RESET
// #define STEP 9569 // 4900  // 9580
// #define SCL 33
// #define SDA 32
// #define BUTTON_1 34       // Button OPEN
// #define BUTTON_2 35       // Button CLOSE
// #define U_STEP 0.55
// #define control_key 26
// #define button_key 27


// #define TX_GPIO_NUM   21  // Connects to CTX
// #define RX_GPIO_NUM   22  // Connects to CRX

// /* -------------  ESP32 C3
//   #define L_PWM 2
//   #define R_PWM 1
//   #define LR_EN 6
//   #define Ch_A 7
//   #define Ch_B 8
//   #define STEP 9569 // 4900  // 9580
//   #define SCL 10
//   #define SDA 9
//   #define BUTTON_1 18
//   #define BUTTON_2  19
//   #define U_STEP 1
//   -------------- */

// // Current sensor
// DFRobot_INA219_IIC     ina219(&Wire, INA219_I2C_ADDRESS1);
// // Revise the following two paramters according to actual reading of the INA219 and the multimeter
// // for linearly calibration
// float ina219Reading_mA = 1000;
// float extMeterReading_mA = 1000;
// float previous_yn = 200.0;  
// float abs_previous;


// long pos = 0;
// volatile long counter = 0;
// unsigned long time_ms;
// int16_t u;
// int u_calc = 0;
// int check;
// bool flag = false;
// bool ket=false;
// bool ket1=false;
// float xn1 = 0;
// float yn1 = 0;
// float pre = 0.0;
// float kalman;
// int dem=0;


// uint16_t new_threshold;

// int tram, chuc, donvi;

// int check_int,check_tt;
// uint16_t threshold_current;

// /*--------------------------------------------Config list status feedback Frame 1
//     ID: 0x111A
//              ID Package 0x11 (status feedback Frame1)
//              ID Device  0x1A (LID)
//     byte[0]  uint8_t status_data
//                   uint8_t opened=0
//                   uint8_t running=1
//                   uint8_t closed=2
//                   uint8_t stuck=101
//                   uint8_t Initializing=3
//                   uint8_t 
//     byte[1]  uint16_t current_threshold_upper
//     byte[2]  uint16_t current_threshold_lower
//     byte[3]  byte button_status
//                   bit[2] Lock Switch
//                   bit[3] Lid Close Button
//     byte[4]  uint16_t current_upper
//     byte[5]  uint16_t current_lower
//     byte[6]  uint16_t voltage_upper
//     byte[7]  uint16_t voltage_lower
// --------------------------------------------*/



// /*--------------------------------------------Config list status feedback Frame 2
//     ID: 0x121A
//             ID Package 0x12 (status feedback Frame 2)
//             ID Device  0x1A (LID)
//     byte[0] byte error
//             bit[0] error CURRENT
//             bit[1] error EEPROM
//             bit[2] error Lock Switch
// --------------------------------------------*/


// //////////////////////// Frame 1
// uint32_t Start_Feedback1 = 0x111A; 
// uint8_t status_data;
// uint8_t button_status=0;
// ////////////////////////////////


// //////////////////////// Frame 2
// uint32_t Start_Feedback2 = 0x121A;
// uint8_t error=0;
// ////////////////////////////////


// ///////////////////////// set Bit to High level i position
// void setBitIndex_High(uint8_t* data_Byte, int index)
// {
//   if(index >=0 && index <8)
//   {
//     *data_Byte |= (1 << index);
//   }
// }


// //////////////////////// set Bit to Low level i position
// void setBitIndex_Low(uint8_t* data_Byte, int index)
// {
//   if(index >=0 && index <8)
//   {
//     *data_Byte &= ~(1 << index);
//   }
// }


// /////////////////////// get Bit level i position
// int getBitIndex(uint8_t get_error, int index)
// {
//   if(index >=0 && index <8)
//   {
//     return (get_error >> index) & 0x01;
//   }
//   return -1;
// }




// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(115200);
//   while (!Serial);
//   delay (1000);
//   static unsigned long startTime = millis();
//   while(EEPROM.begin(512) == false) {
//   setBitIndex_High(&error,1);
//   unsigned long currentTime = millis();
//   unsigned long elapsedTime = currentTime - startTime;
//   if(elapsedTime >=2000)
//   {
//     break;
//   }
// }

//   if(EEPROM.read(0)!=255)
//   {
//     byte byte_1=EEPROM.read(0);
//     byte byte_2=EEPROM.read(1);
//     byte byte_3=EEPROM.read(2);
//     threshold_current=byte_1*100+byte_2*10+byte_3; 
//   }


  

  
  
//   pinMode(L_PWM, OUTPUT);
//   pinMode(R_PWM, OUTPUT);
//   pinMode(LR_EN, OUTPUT);
//   pinMode(control_key, OUTPUT);
//   pinMode(button_key, INPUT_PULLUP);
//   pinMode(BUTTON_1, INPUT_PULLUP);
//   pinMode(BUTTON_2, INPUT_PULLUP);
//   pinMode(BUTTON_RST, INPUT_PULLUP);
//   pinMode(2,OUTPUT);
//   ledcSetup(0, 1000, 8);
//   ledcAttachPin(LR_EN, 0);
//   delay(10);
//   digitalWrite(L_PWM, 0);
//   digitalWrite(R_PWM, 0);
//   time_ms = millis();
//   //------------ INA219----------------
//   Wire.setPins(SDA, SCL);
//   static unsigned long startTime1 = millis();
  
//   while(ina219.begin() == false) {
//     setBitIndex_High(&error,0);
//     unsigned long currentTime = millis();
//     unsigned long elapsedTime = currentTime - startTime1;
//     if(elapsedTime >=3000)
//     {
//       break;
//     }
//   }
//   //Linear calibration
//   ina219.linearCalibrate(/*The measured current before calibration*/ina219Reading_mA, /*The current measured by other current testers*/extMeterReading_mA);
//   //------------------------------------

    
// ////////////////////////////////////////////////////// BEGIN CAN

//    CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

//   // start the CAN bus at 500 kbps
//   if (!CAN.begin (500E3)) {
//     Serial.println ("Starting CAN failed!");
//     while (1);
//   }
//   else {
//     Serial.println ("CAN Initialized");
//   }

// ////////////////////////////////////////////////////// RESET MOTOR
//  int u_reset=100;
// while (digitalRead(BUTTON_1)==1)
// {
//    static unsigned long startTime2 = millis();
//    while(digitalRead(button_key)==1)
//    {
//     digitalWrite(control_key,HIGH);
//     unsigned long currentTime = millis();
//     unsigned long elapsedTime = currentTime - startTime2;
//     if(elapsedTime >=2000)
//     {
//      setBitIndex_High(&error,2);
//      send_status_frame2(Start_Feedback2,error);
//     }
//    }
//    setBitIndex_High(&button_status,2);
//    setBitIndex_Low(&error,2);
//    float xn_reset = ina219.getCurrent_mA();
//    float yn_reset = bo_loc.updateEstimate(xn_reset);
//    float vol=ina219.getBusVoltage_V();
//    int current=(int)(yn_reset*10);
//    int voltage=(int)(vol*10);
//    if(yn_reset > threshold_current)
//    {
//       u_reset=0;
//    }
//    set_motor(u_reset);
//    delay(1);
//    status_data=3;
//    send_status_frame1(Start_Feedback1,status_data,threshold_current,button_status,current,voltage);  
   
// }
  
// }



// static inline int8_t sign(int val) {
//   if (val < 0) return -1;
//   if (val == 0) return 0;
//   return 1;
// }

// int soft_start(int u_set, float u)
// {
//   float d_u = u_set - u;

//   int k = d_u / U_STEP;

//   if (k == 0) return u_set;
//   else
//   {
//     pre = sign(d_u) * U_STEP + u;
//     return (int)(pre);
//   }
// }


// int clip(int value, int min_, int max_)
// {
//   if ( value < min_ ) return min_;
//   if ( value > max_ ) return max_;
//   return value;
// }

// void set_motor(int pwm)
// {
//   if (pwm > 0)
//   {
//     digitalWrite(R_PWM, 1);
//     digitalWrite(L_PWM, 0);
//     ledcWrite(0, clip(pwm, 80, 255));
//   }
//   else if (pwm < 0)
//   {
//     digitalWrite(R_PWM, 0);
//     digitalWrite(L_PWM, 1);
//     ledcWrite(0, clip(abs(pwm), 80, 255));
//   }
//   else
//   {
//     digitalWrite(R_PWM, 0);
//     digitalWrite(L_PWM, 0);
//     ledcWrite(0, pwm);
//   }
// }
// float calc_angle(long pos)
// {
//   return 2 * PI * pos / STEP;
// }

// /////////////// Lid status feedback Frame 1
// void send_status_frame1(uint32_t Start_Feedback1_tem, uint8_t status_data_tem, uint16_t threshold_current_tem, uint8_t button_status_tem, uint16_t current_tem, uint16_t voltage_tem)
// {
//   CAN.beginExtendedPacket(Start_Feedback1_tem);
//   CAN.write(status_data_tem);
//   byte thresholdByte[2];
//   thresholdByte[0]=(byte)(threshold_current_tem & 0xFF);       //// threshold_lower
//   thresholdByte[1]=(byte)(threshold_current_tem >>8 ) & 0xFF;  //// threshold_upper
//   for(int i=0;i<sizeof(thresholdByte);i++)
//   {
//     CAN.write(thresholdByte[i]);
//   }

//   CAN.write(button_status_tem);

//   byte currentByte[2];
//   currentByte[0]=(byte)(current_tem & 0xFF);           //// current_lower
//   currentByte[1]=(byte)(current_tem >>8 ) & 0xFF;      //// current_upper
//   for(int i=0;i<sizeof(currentByte);i++)
//   {
//     CAN.write(currentByte[i]);
//   }

//   byte voltageByte[2];
//   voltageByte[0]=(byte)(voltage_tem & 0xFF);           //// voltage_lower
//   voltageByte[1]=(byte)(voltage_tem >> 8 ) & 0xFF;     //// voltage_upper
//   for(int i=0;i<sizeof(voltageByte);i++)
//   {
//     CAN.write(voltageByte[i]);
//   }
//   CAN.endPacket();
//   delay(10);
// }


// /////////////// Lid status feedback Frame 2
// void send_status_frame2(uint32_t Start_Feedback2_tem, uint8_t error_tem)
// {
//   CAN.beginExtendedPacket(Start_Feedback2);
//   CAN.write(error_tem);
//   CAN.endPacket();
//   delay(10);
// }









// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// void loop() {
// int size_data = CAN.parsePacket();
// float xn,yn,vol;
// int current,voltage;
// if(size_data)
// {
//   if(CAN.available())
//   {
//     int pk_ID=(CAN.packetId() >> 8) & 0xFF;
//     int dv_ID=CAN.packetId() & 0xFF;
//     if(dv_ID==0x1A)
//     {
//       if(pk_ID==0x01)
//         {
//           byte data_input[8]; // Giả sử độ dài tối đa của gói tin là 8 byte
//           CAN.readBytes(data_input, size_data);
//           u=(data_input[0] << 8) | data_input[1];
//           if(u>0)
//             {
//             check=1;
//             Serial.println(u);
//             digitalWrite(control_key,HIGH);
//             delay(100);
//               if(digitalRead(button_key)==0)
//               {
//                 setBitIndex_High(&button_status,2);  
//                 setBitIndex_Low(&error,2);
//               }
//               else
//                 setBitIndex_High(&error,2);
//             }
//            else if(u<0)
//             {
//             check=2;
//             Serial.println(u);
//             digitalWrite(control_key,LOW);
//             delay(100);
//               if(digitalRead(button_key)==1)
//               {
//                 setBitIndex_Low(&button_status,2);  
//                 setBitIndex_Low(&error,2);
//               }
//               else
//                 setBitIndex_High(&error,2);
//             }
//         }
//       else if(pk_ID==0x02)
//              {
//               int pk_ID=(CAN.packetId() >> 8) & 0xFF;
//               int dv_ID=CAN.packetId() & 0xFF;
//               if(pk_ID==0x02 && dv_ID==0x1A)
//               {
//                 byte data_input[8]; // Giả sử độ dài tối đa của gói tin là 8 byte
//                 CAN.readBytes(data_input, size_data);
//                 new_threshold=(data_input[0] << 8) | data_input[1];
//                 if(threshold_current != new_threshold)
//                   {
//                     threshold_current=new_threshold;
//                     tram=new_threshold/100;
//                     chuc=new_threshold/10%10;
//                     donvi=new_threshold%10;
//                     EEPROM.write(0,tram);
//                     EEPROM.write(1,chuc);
//                     EEPROM.write(2,donvi);
//                     EEPROM.commit(); 
//                   }
//               }
//              }

//     }
//   } 
// }

//   int button1 = digitalRead(BUTTON_1);
//   int button2 = digitalRead(BUTTON_2);

//   if (u < 0 && button2 == 1)
//   {
//     ket=false;
//     u = 0;
//     u_calc = 0;
//     pre = 0;
//     flag = true;
//   }


//   if (u > 0 && button1 == 1)
//   {
//     ket=false;
//     u = 0;
//     u_calc = 0;
//     pre = 0;
//     flag = true;
//   }
  
//   if(button1==1)
//   {
//    ket=false;
//    status_data=0;  
//   }
  
//   if(button2==1)
//   {
//    ket=false;
//    status_data=2;
//   }

//   if(button1==0&&button2==0&&ket==false)
//   {
//    status_data=1;
//   }

//     ///////////// current not error
//   if(ina219.begin() == true) 
//   {
//       setBitIndex_Low(&error,0);
//   }

//   ///////////// current error
//   else if(ina219.begin() == false)
//   {
//       setBitIndex_High(&error,0);
//   }
//   ////////// EEPROM not error

//   if(EEPROM.begin(512) == true)
//   {
//       setBitIndex_Low(&error,1);
//   }

//   //////////// EEPROM error
//   else if (EEPROM.begin(512) == false) 
//   {
//       setBitIndex_High(&error,1);
//   }
   

  
//   // put your main code here, to run repeatedly:
//   if (time_ms - millis() > 100)
//   {
//       if(digitalRead(BUTTON_RST)==0)
//       {
//         setBitIndex_High(&button_status,3);
//         while(!digitalRead(BUTTON_2))
//         {
//           set_motor(-100);
//           delay(1);
//         }
//       }
//       setBitIndex_Low(&button_status,3);
//       send_status_frame2(Start_Feedback2,error);
//         xn = ina219.getCurrent_mA();
//         yn = bo_loc.updateEstimate(xn);
//         vol=ina219.getBusVoltage_V();
//         current=(int)(yn*10);
//         voltage=(int)(vol*10);
//       if(yn>=threshold_current)
//       {
//         u=0;
//         u_calc = 0;
//         pre = 0;
//         ket=true;
//         check=3;
//         status_data=101;
//       }
//       else if(yn<=25&&ket==true)
//       {
//         status_data=101;
//         if(check==1 or check==2)
//         {
//           ket=false; 
//         }
//       } 
//       send_status_frame1(Start_Feedback1,status_data,threshold_current,button_status,current,voltage);    
//       time_ms = millis();
//   }
//   //  u_calc = soft_start(u,u_calc,1,5000,-100,100);
//   u_calc = soft_start(u, pre);
//   if (flag)
//   {
//     u_calc = 0;
//     pre = 0;
//     flag = false;
//   }
//   set_motor(u_calc);
//   delay(1);
// }