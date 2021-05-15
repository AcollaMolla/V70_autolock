#include <SPI.h>
#include <mcp2515.h>

/*
 * This application monitors the CAN bus for vehicle speed and doors locked status. If the vehicle speed is below
 * a certail threshold and the doors status are unlocked, the doors will be automatically locked.
 * When the vehicle speed has been above the same threshold for a certain amount of time, the doors will unlock.
 */

struct can_frame canMsg;
struct can_frame lockUnlock; 
struct can_frame get_vehicle_speed;
struct can_frame doors_status;

////////////////////////////////Constants////////////////////////////////////////////////////
//The amount of ms that the application will sleep between each CAN bus request. 
int TICK_PERIOD = 1000;
//The number of TICK_PERIODs that the vehicle must be above THRESHOLD_SPEED for unlocking doors.
int ABOVE_THRESHOLD_LIMIT = 100;
//The vehicle speed threshold in km/h
int THRESHOLD_SPEED = 40;
////////////////////////////////////////////////////////////////////////////////////////////

//If a response to the request is detected this will be true
bool vehicle_speed_found = false, verbose = true, doors_locked = false;

int counter = 0, speed = 0;
int seconds_above_limit = 0;
int requests_sent = 0, replies_received = 0;
  
MCP2515 mcp2515(10);


void setup() {
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  //mcp2515.setFilterMask(MCP2515::MASK0, true, 0x1FFFFFFF);
  //mcp2515.setFilter(MCP2515::RXF0, true, 0x81c01022);
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  requests_sent = 0;
  replies_received = 0;
  delay(TICK_PERIOD);
  vehicle_speed_found = false;
  counter = 0;
  
  //Ask CEM for speed and door status
  GetDoorsStatus();
  GetVehicleSpeed();

  //Monitor the bus for any responses linked to the previous request. Keep listening for 500 messages (which is around 2 minutes).
  //if a message containing the vehicle speed is detected the loop is breaked.
  do{
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if(canMsg.can_id == 0x80c00003 && canMsg.data[3] == 0x4a){
      replies_received++;
      speed = canMsg.data[7]/4;
      if(verbose){
        Serial.print("Speed: ");
        Serial.println(speed);
      }
      
      if(speed < THRESHOLD_SPEED && !doors_locked){
        LockDoors();
        doors_locked = true;
        seconds_above_limit = 0;
      }
      else if(speed > THRESHOLD_SPEED && doors_locked && seconds_above_limit < ABOVE_THRESHOLD_LIMIT){
        seconds_above_limit++;
      }

      else if(speed > THRESHOLD_SPEED && doors_locked && seconds_above_limit >= ABOVE_THRESHOLD_LIMIT){
        seconds_above_limit = 0;
        LockDoors();
        doors_locked = false;
      }
      
      vehicle_speed_found = true;
    }

    else if(canMsg.can_id == 0x80c00003 && canMsg.data[3] == 0x6a){
      replies_received++;
      /*Serial.print(canMsg.can_id, HEX);
      Serial.print(",");
      Serial.print(canMsg.can_dlc, HEX);
      Serial.print(",");
      for(int i=0;i<canMsg.can_dlc;i++){
        Serial.print(canMsg.data[i], HEX);
      }
      Serial.println();*/
      if(verbose)Serial.print("Door status: ");
      if(canMsg.data[5] > 0x7a){
        Serial.println("Unlocked");
        doors_locked = false; 
      }
      else if(canMsg.data[5] <= 0x1){
        Serial.println("Locked");
        doors_locked = true;
      }
      else{
        Serial.print("Unknown (0x");
        Serial.print(canMsg.data[5], HEX);
        Serial.println(")");
      }
    }
  }
  counter++;
 }while((replies_received < requests_sent) && counter < 500);
}

void GetVehicleSpeed(){
  requests_sent++;
  get_vehicle_speed.can_id = 0x0800ffffe; //Try this without leading 0 as well
  get_vehicle_speed.can_dlc = 8;
  get_vehicle_speed.data[0] = 0xcd;
  get_vehicle_speed.data[1] = 0x40;
  get_vehicle_speed.data[2] = 0xa6;
  get_vehicle_speed.data[3] = 0x4a;
  get_vehicle_speed.data[4] = 0x1;
  get_vehicle_speed.data[5] = 0x1;
  get_vehicle_speed.data[6] = 0x00;
  get_vehicle_speed.data[7] = 0x00;
  mcp2515.sendMessage(&get_vehicle_speed);
}

void GetDoorsStatus(){
  requests_sent++;
  doors_status.can_id = 0x0800ffffe;
  doors_status.can_dlc = 8;
  doors_status.data[0] = 0xcd;
  doors_status.data[1] = 0x40;
  doors_status.data[2] = 0xa6;
  doors_status.data[3] = 0x6a;
  doors_status.data[4] = 0x6;
  doors_status.data[5] = 0x1;
  doors_status.data[6] = 0x00;
  doors_status.data[7] = 0x00;
  mcp2515.sendMessage(&doors_status);
}

void GetKeyPos(){
  doors_status.can_id = 0x800ffffe;
  doors_status.can_dlc = 8;
  doors_status.data[0] = 0xcd;
  doors_status.data[1] = 0x40;
  doors_status.data[2] = 0xa6;
  doors_status.data[3] = 0x1a;
  doors_status.data[4] = 0x4;
  doors_status.data[5] = 0x1;
  doors_status.data[6] = 0x00;
  doors_status.data[7] = 0x00;
  mcp2515.sendMessage(&doors_status);
}

void LockDoors(){
    lockUnlock.can_id = 0x81c01022;
    lockUnlock.can_dlc = 8;
    lockUnlock.data[0] = 0x00;
    lockUnlock.data[1] = 0x00;
    lockUnlock.data[2] = 0x00;
    lockUnlock.data[3] = 0xff;
    lockUnlock.data[4] = 0x00;
    lockUnlock.data[5] = 0x00;
    lockUnlock.data[6] = 0x00;
    lockUnlock.data[7] = 0x00;
    if(verbose && !doors_locked){
      Serial.println("Locking doors");
      doors_locked = true;
    }
    if(verbose && doors_locked){
      Serial.println("Unlocking doors");
      doors_locked = false;
    }
    mcp2515.sendMessage(&lockUnlock);
}
