#include "mavlink.h";
#define THRO 11
#define THRO_MIN 45
#define THRO_MAX 139

#define AILE 10
#define AILE_MIN 46
#define AILE_MAX 145
 
#define ELEV 9
#define ELEV_MIN 46
#define ELEV_MAX 140

#define RUDD 6
#define RUDD_MIN 45
#define RUDD_MAX 140

#define GEAR 5
#define GEAR_MIN 45
#define GEAR_MAX 140

#define AUX1 3
#define AUX1_MIN 45
#define AUX1_MAX 140

#define MAV_SYSTEM_ID 123
#define MAV_COMPONENT_ID 200

const int system_type = MAV_QUADROTOR;
const int autopilot_type = MAV_AUTOPILOT_GENERIC;

int system_mode = MAV_MODE_UNINIT;
int system_nav_mode = MAV_NAV_GROUNDED;
int system_status = MAV_STATE_ACTIVE;

#include <Servo.h>

Servo sTHRO;
Servo sAILE;
Servo sELEV;
Servo sRUDD;
Servo sGEAR;
Servo sAUX1;

int pos = 0;

mavlink_message_t msg; 
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
mavlink_status_t status;

long system_dropped_packets = 0;

void setup()
{
  Serial.begin(57600);
  sTHRO.attach(THRO);
  sAILE.attach(AILE);
  sELEV.attach(ELEV);
  sRUDD.attach(RUDD);
  sGEAR.attach(GEAR);
  sAUX1.attach(AUX1);
}


void sendSerialHeartbeat() {
  mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, system_type, autopilot_type);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void sendSerialSysStatus() {
  mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, system_mode, system_nav_mode, system_status, 0, (int)0, 0, system_dropped_packets);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void readSerialMavLink() {
  while(Serial.available() > 0) { 
    uint8_t c = Serial.read();
    //try to get a new message 
      if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
        // Handle message
        switch(msg.msgid) {
          /*
          case MAVLINK_MSG_ID_SET_MODE: {
            system_mode = mavlink_msg_set_mode_get_mode(&msg);
            sendSerialSysStatus();
          }
          break;
          
          case MAVLINK_MSG_ID_ACTION: {
            uint8_t result = 0;
            
            //if (mavlink_msg_action_get_target(&msg) != MAV_SYSTEM_ID || mavlink_msg_action_get_target_component(&msg) != MAV_COMPONENT_ID) return;
              uint8_t action = mavlink_msg_action_get_action(&msg);
              switch(action) {
                MAV_ACTION_MOTORS_START: {
                  armed = ON;
                  result = 1;
                  system_status = MAV_STATE_ACTIVE;
                  sendSerialSysStatus();
                }
                break;
                MAV_ACTION_MOTORS_STOP: {
                  armed = OFF;
                  result = 1;
                  system_status = MAV_STATE_STANDBY;
                  sendSerialSysStatus();
                }
                break;
                MAV_ACTION_CALIBRATE_GYRO: {
                  if (system_status == MAV_STATE_STANDBY)
                  {
                    gyro.calibrate();
                    result = 1;
                  }
                  else
                  {
                  result = 0;
                  }
                }                
                break;                    
              }
             
             mavlink_msg_action_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, action, result);
             uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
             Serial.write(buf, len);
          }
          break;*/
          case MAVLINK_MSG_ID_MANUAL_CONTROL: {
            //if (mavlink_msg_manual_control_get_target(&msg) != MAV_SYSTEM_ID || system_mode != MAV_MODE_MANUAL) return;
            if (mavlink_msg_manual_control_get_roll_manual(&msg))
            {
              sAILE.write((int)((((mavlink_msg_manual_control_get_roll(&msg)+0.2)*2.5)*(AILE_MAX-AILE_MIN)+AILE_MIN)));
            }
            if (mavlink_msg_manual_control_get_pitch_manual(&msg))
            {
              sELEV.write((int)((((mavlink_msg_manual_control_get_pitch(&msg)+0.2)*2.5)*(ELEV_MAX-ELEV_MIN)+ELEV_MIN)));
            }
            if (mavlink_msg_manual_control_get_thrust_manual(&msg))
            {
              sRUDD.write((int)((RUDD_MAX-((mavlink_msg_manual_control_get_thrust(&msg))*(RUDD_MAX-RUDD_MIN))+RUDD_MIN)));
            }
            if (mavlink_msg_manual_control_get_yaw_manual(&msg))
            {
               sTHRO.write((int)(((mavlink_msg_manual_control_get_yaw(&msg)+0.5)*(THRO_MAX-THRO_MIN)+THRO_MIN)));
            }
            
          }
          break;
          /*
          case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            int8_t roll_p[15] = "Roll_P";
            int8_t roll_i[15] = "Roll_I";
            int8_t roll_d[15] = "Roll_D";
            sendSerialPID(ROLL, roll_p, roll_i, roll_d, 1, 24);
            
            int8_t pitch_p[15] = "Pitch_P";
            int8_t pitch_i[15] = "Pitch_I";
            int8_t pitch_d[15] = "Pitch_D";
            sendSerialPID(PITCH, pitch_p, pitch_i, pitch_d, 4, 24);
            
            int8_t yaw_p[15] = "Yaw_P";
            int8_t yaw_i[15] = "Yaw_I";
            int8_t yaw_d[15] = "Yaw_D";
            sendSerialPID(YAW, yaw_p, yaw_i, yaw_d, 7, 24);
            
            int8_t heading_p[15] = "Heading_P";
            int8_t heading_i[15] = "Heading_I";
            int8_t heading_d[15] = "Heading_D";
            sendSerialPID(HEADING, heading_p, heading_i, heading_d, 10, 24);
            
            int8_t levelroll_p[15] = "Level Roll_P";
            int8_t levelroll_i[15] = "Level Roll_I";
            int8_t levelroll_d[15] = "Level Roll_D";
            sendSerialPID(LEVELROLL, levelroll_p, levelroll_i, levelroll_d, 13, 24);
                        
            int8_t levelpitch_p[15] = "Level Pitch_P";
            int8_t levelpitch_i[15] = "Level Pitch_I";
            int8_t levelpitch_d[15] = "Level Pitch_D";
            sendSerialPID(LEVELPITCH, levelpitch_p, levelpitch_i, levelpitch_d, 16, 24);
            
            int8_t levelgyroroll_p[15] = "Lvl gyro rol_P";
            int8_t levelgyroroll_i[15] = "Lvl gyro rol_I";
            int8_t levelgyroroll_d[15] = "Lvl gyro rol_D";
            sendSerialPID(LEVELGYROROLL, levelgyroroll_p, levelgyroroll_i, levelgyroroll_d, 19, 24);
            
            int8_t levelgyropitch_p[15] = "Lvl gyro pit_P";
            int8_t levelgyropitch_i[15] = "Lvl gyro pit_I";
            int8_t levelgyropitch_d[15] = "Lvl gyro pit_D";
            sendSerialPID(LEVELGYROPITCH, levelgyropitch_p, levelgyropitch_i, levelgyropitch_d, 22, 24);
          }
          break;*/
          default:
            //Do nothing
          break;
        }
      } 
      // And get the next one
    } 
    system_dropped_packets += status.packet_rx_drop_count;
}
int i = 0;
void loop()
{
  readSerialMavLink();
  
  // keep in stable mode!
  sGEAR.write(140);
  
  if (i > 10000)
  {
    sendSerialHeartbeat(); 
  }
  i++; 
} 
