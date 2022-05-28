#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include <LTC2499.h>
#include <WireIMXRT.h>
enum ACUSTATE {BOOTUP, RUNNING, FAULT};
int acuState=0;
#define runningFoReal
#ifdef runningFoReal
#define NUMBER_OF_LTCs 6
uint8_t gettingTempState=0; //0=set 1=wait 2=get
//Init ADCs
Ltc2499 theThings[16];
int8_t batteryTempvoltages[96];
uint8_t ltcAddressList[]={ADDR_00Z,ADDR_ZZZ,ADDR_0ZZ,
                           ADDR_ZZ0,ADDR_Z0Z,ADDR_Z00}; //first 6 configurable addresses in the mf datasheet
byte ADCChannels[]={CHAN_SINGLE_0P,CHAN_SINGLE_1P,CHAN_SINGLE_2P,CHAN_SINGLE_3P,
                    CHAN_SINGLE_4P,CHAN_SINGLE_5P,CHAN_SINGLE_6P,CHAN_SINGLE_7P,
                    CHAN_SINGLE_8P,CHAN_SINGLE_9P,CHAN_SINGLE_10P,CHAN_SINGLE_11P};
elapsedMillis conversionTime;//wait 80ms for conversion to be ready
#endif
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
int8_t batteryTemps[96];
//Can IDs
#define BMS_ID 0x7E3
#define ThermistorToBMS_ID 0x9839F380 
#define ThermistorAddressClaim_ID 0x98EEFF80 //probably unnecessary
#define BMS_Response_ID 0x7EB
//can bytes
int moduleNo=0;//byte0
int enabledTherm;//byte4
byte getLowestTemp[] = {0x03,0x22,0xF0,0x28,0x55,0x55,0x55,0x55}; //lowest temp request
byte getHighestTemp[] = {0x03,0x22,0xF0,0x29,0x55,0x55,0x55,0x55}; //lowest temp request
Metro sendTempRate=Metro(100);
Metro getTempRate=Metro(500);
Metro doThingsRate=Metro(100);
#define DEBUG
//printing received to serial
void canSniff(const CAN_message_t &msg);
void getTempData();
void sendTempData();
void ACUStateMachine();
int setChannels(int channelNo);
void getTemps(int channelNo);
void setup() {
  Wire.setClock(100000);
   Wire.begin();
  Serial.begin(115200); delay(400);
  Serial.println("Battery temp array: ");
  for(int i=0; i<=95;i++){
    Serial.print("Cell number: ");
    Serial.print(i);
    Serial.print(" Value: ");
    // batteryTemps[i]=random(-40,80);
    batteryTemps[i]=25; //init default temps as a safe value
    Serial.println(batteryTemps[i]);
  }
  pinMode(9, OUTPUT); digitalWrite(9, HIGH); //tranceiver enable pin, connect to MCP2562 standby pin
  // pinMode(LED_BUILTIN,OUTPUT);
  for(int i=0;i<4;i++){
    pinMode(i, OUTPUT); digitalWrite(i, LOW);
  }
  // #ifdef runningFoReal
  //  for(int i=0;i<NUMBER_OF_LTCs;i++){
  //   byte ltcStatus=theThings[i].begin(ltcAddressList[i]);
  //   if(ltcStatus){
  //     Serial.print(ltcAddressList[i]);
  //     Serial.printf(", Error with LTC # %d",i);
  //     Serial.println();
  //     digitalWrite(0,HIGH);
  //     // while(1){};
  //   }
  //   else{
  //     Serial.printf("initialized LTC #%d with address %x\n",i,ltcAddressList[i]);
  //   }
  // }
  // #endif
  
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  #ifdef DEBUG
    //Can0.onReceive(canSniff);
  #endif
  Can0.mailboxStatus();
  // Serial.println("Send something on serial to continue...");
  // while(!Serial.available());{ }
}
#ifdef runningFoReal
void getAllTheTemps(int channelNo){
  for(int i=0;i<NUMBER_OF_LTCs;i++){
    theThings[i].changeChannel(ADCChannels[channelNo]);
  }
  conversionTime=0;
  while(conversionTime<80){}
  for(int i=0;i<NUMBER_OF_LTCs;i++){
    float voltageX=theThings[i].readVoltage();
    batteryTempvoltages[(i*16)+channelNo]=voltageX;
    #ifdef DEBUG
      char buffer[50];
      sprintf(buffer,"LTC Number: %d  Channel: %d Voltage: %f",i,channelNo,voltageX);
      Serial.println(buffer);
    #endif
  }
  // Serial.println("=================");
}
#endif
void loop() {
  digitalWrite(LED_BUILTIN,LOW);
  Can0.events();
  if(doThingsRate.check()){
    ACUStateMachine();
  }
  // if(sendTempRate.check()==1){
  //   digitalWrite(LED_BUILTIN,HIGH);
  //   sendTempData();
  //   //while(1){}
  // }
  // if(getTempRate.check()){
  //   digitalWrite(LED_BUILTIN,HIGH);
  //   getTempData();
  // }
  // #ifdef runningFoReal
  //   getAllTheTemps(0);
  //   digitalWrite(LED_BUILTIN,HIGH);
  //   getAllTheTemps(1);
  //   digitalWrite(LED_BUILTIN,LOW);
  //   getAllTheTemps(2);
  //   digitalWrite(LED_BUILTIN,HIGH);
  //   getAllTheTemps(3);
  //   digitalWrite(LED_BUILTIN,LOW);
  //   getAllTheTemps(4);
  //   digitalWrite(LED_BUILTIN,HIGH);
  //   getAllTheTemps(5);
  //   digitalWrite(LED_BUILTIN,LOW);
  //   getAllTheTemps(6);
  //   digitalWrite(LED_BUILTIN,HIGH);
  //   getAllTheTemps(7);
  //   digitalWrite(LED_BUILTIN,LOW);
  //   getAllTheTemps(8);
  //   digitalWrite(LED_BUILTIN,HIGH);
  //   getAllTheTemps(9);
  //   digitalWrite(LED_BUILTIN,LOW);
  //   getAllTheTemps(10);
  //   digitalWrite(LED_BUILTIN,HIGH);
  //   getAllTheTemps(11);
  //   digitalWrite(LED_BUILTIN,LOW);
  //   getAllTheTemps(12);
  //   digitalWrite(LED_BUILTIN,HIGH);
  //   getAllTheTemps(13);
  //   digitalWrite(LED_BUILTIN,LOW);
  //   getAllTheTemps(14);
  //   digitalWrite(LED_BUILTIN,HIGH);
  //   getAllTheTemps(15);
  // #endif
}
void canSniff(const CAN_message_t &msg) {
  //if(msg.id==BMS_Response_ID){
  digitalWrite(LED_BUILTIN,HIGH);
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
  //}
}
//getting one of the max temps from BMS (high or low not sure lol)
void getTempData()
{
    CAN_message_t getTempMsg;
    getTempMsg.flags.extended=1;
    getTempMsg.len = 8;
    getTempMsg.id = BMS_ID; //OUR BMS
    memcpy(getTempMsg.buf, getLowestTemp, sizeof(getTempMsg.buf));
    Can0.write(getTempMsg);
    Serial.println("Requesting Lowest Temp Data...");
    memcpy(getTempMsg.buf, getHighestTemp, sizeof(getTempMsg.buf));
    Can0.write(getTempMsg);
    Serial.println("Requesting Highest Temp Data...");
}
//Sending highest/lowest temperature to the BMS
void sendTempData(){
  CAN_message_t sendTempMsg;
  sendTempMsg.flags.extended=1;//extended id
  sendTempMsg.len = 8;//per protocol
  sendTempMsg.id = ThermistorToBMS_ID; //Temp broadcast ID
  enabledTherm=95;//number of cells 0 based
  int lowTherm=0, lowestThermId=0, highTherm=0, highestThermId=0;
  for (int i=0;i<=95;i++){ //get lowest and highest
  #ifdef DEBUG
   Serial.print("Cell number: ");
    Serial.print(i);
    Serial.print(" Value: ");
    Serial.println(batteryTemps[i]);
  #endif
    if(batteryTemps[i]<lowTherm){
      lowTherm=batteryTemps[i];
      lowestThermId=i;
    }
    if(batteryTemps[i]>highTherm){
      highTherm=batteryTemps[i];
      highestThermId=i;
    }
    #ifdef DEBUG
    Serial.printf("Iter: %d Highest: %d Lowest: %d\n",i,highTherm,lowTherm);
    #endif
  }
  int avgTherm=(lowTherm+highTherm)/2;//yep
  int checksum=moduleNo+lowTherm+highTherm+avgTherm+enabledTherm+highestThermId+lowestThermId+57+8;//0x39 and 0x08 added to checksum per orion protocol
  byte tempdata[]={moduleNo,lowTherm,highTherm,avgTherm,enabledTherm,highestThermId,lowestThermId,checksum};
  memcpy(sendTempMsg.buf, tempdata, sizeof(sendTempMsg.buf));
  // Can0.write(sendTempMsg);
  // #ifdef DEBUG
  //  Serial.print("  LEN: "); Serial.print(sendTempMsg.len);
  //   Serial.print(" EXT: "); Serial.print(sendTempMsg.flags.extended);
  //   Serial.print(" TS: "); Serial.print(sendTempMsg.timestamp);
  //   Serial.print(" ID: "); Serial.print(sendTempMsg.id, HEX);
  //   Serial.print(" Buffer: ");
  //   for ( uint8_t i = 0; i < sendTempMsg.len; i++ ) {
  //     Serial.print(sendTempMsg.buf[i], HEX); Serial.print(" ");
  //   } 
  // Serial.println();
  // //Serial.println("Sending Temp Data...");
  // #endif
}
void ACUStateMachine(){
  switch(acuState){
    case 0:
      bool initOk=true;
      for(int i=0;i<NUMBER_OF_LTCs;i++){
        byte ltcStatus=theThings[i].begin(ltcAddressList[i]);
        if(ltcStatus){
          Serial.print(ltcAddressList[i]);
          Serial.printf(", Error with LTC # %d\n",i);
          initOk=false;
        }
        else{
          Serial.printf("initialized LTC #%d with address %x\n",i,ltcAddressList[i]);
        }
      }
      if(initOk==true){
        acuState=1;
      }else if(initOk==false){acuState=2;}    
    break;
    case 1:
      switch(gettingTempState){
        case 0:
          setChannels(0);
          conversionTime=0;
          gettingTempState=1;
          break;
        case 1:
          if(conversionTime>=80){
            gettingTempState=2;
          }
          break;
        case 2:
          getTemps(0);
          gettingTempState=0;
      }
    break;
    case 2:
    delay(200);
    acuState=0;
    break;
  }
}
int setChannels(int channelNo){
  for(int i=0;i<NUMBER_OF_LTCs;i++){
    theThings[i].changeChannel(ADCChannels[channelNo]);
    return channelNo;
  }
} 
void getTemps(int channelNo){
  for(int i=0;i<NUMBER_OF_LTCs;i++){
    float v=theThings[i].readVoltage();
    int cellNum=(i*16)+channelNo;
    batteryTempvoltages[cellNum]=v;
    #ifdef DEBUG
      char buffer[50];
      sprintf(buffer,"LTC Number: %d  Channel: %d Voltage: %d",i,channelNo,batteryTempvoltages[cellNum]);
      Serial.println(buffer);
    #endif
  }
}
void actuallyGetAllTemps(){
  switch(gettingTempState){
        case 0:
          setChannels(0);
          conversionTime=0;
          gettingTempState=1;
          break;
        case 1:
          if(conversionTime>=80){
            gettingTempState=2;
          }
          break;
        case 2:
          getTemps(0);
          gettingTempState=0;
      }
}