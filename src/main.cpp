#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include <LTC2499.h>
#include <WireIMXRT.h>
#define runningFoReal
#ifdef runningFoReal
//Init ADCs
Ltc2499 theThings[16];
int8_t batteryTempvoltages[96];
uint8_t ltcAddressList[6]={ADDR_ZZZ,0x15,0x16,
                           0x17,0x24,0x25}; //first 6 configurable addresses in the mf datasheet
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
//#define DEBUG
//printing received to serial
void canSniff(const CAN_message_t &msg);
void getTempData();
void sendTempData();
void setup() {
  Wire.setClock(100000);
   Wire.begin();
  Serial.begin(115200); delay(400);
  Serial.println("Battery temp array: ");
  for(int i=0; i<=95;i++){
    Serial.print("Cell number: ");
    Serial.print(i);
    Serial.print(" Value: ");
    batteryTemps[i]=random(-40,80);
    Serial.println(batteryTemps[i]);
  }
  #ifdef runningFoReal
   for(int i=0;i<6;i++){
    byte ltcStatus=theThings[i].begin(ltcAddressList[i]);
    if(ltcStatus){
      Serial.print(ltcAddressList[i]);
      Serial.printf(", Error with LTC # %d\n",i);
    }
    else{
      Serial.printf("initialized LTC #%d with address %x\n",i,ltcAddressList[i]);
    }
  }
  #endif
  pinMode(6, OUTPUT); digitalWrite(6, LOW); //tranceiver enable pin, connect to MCP2562 standby pin
  pinMode(LED_BUILTIN,OUTPUT);
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  #ifdef DEBUG
    //Can0.onReceive(canSniff);
  #endif
  Can0.mailboxStatus();
  Serial.println("Send something on serial to continue...");
  while(!Serial.available());{

  }
}
#ifdef runningFoReal
void getAllTheTemps(const int idfk,int channelNo){
  for(int i=0;i<6;i++){
    theThings[i].changeChannel(idfk);
  }
  conversionTime=0;
  while(conversionTime<80){}
  for(int i=0;i<6;i++){
    float voltageX=theThings[i].readVoltage();
    batteryTempvoltages[(i*16)+channelNo]=voltageX;
  }
}
#endif
void loop() {
  digitalWrite(LED_BUILTIN,LOW);
  Can0.events();
  //signed int avgTherm=(lowTherm+highTherm)/2;
  if(sendTempRate.check()==1){
    digitalWrite(LED_BUILTIN,HIGH);
    sendTempData();
    //while(1){}
  }
  if(getTempRate.check()){
    digitalWrite(LED_BUILTIN,HIGH);
    getTempData();
  }
  #ifdef runningFoReal
    getAllTheTemps(CHAN_SINGLE_0P,0);
    getAllTheTemps(CHAN_SINGLE_1P,1);
    getAllTheTemps(CHAN_SINGLE_2P,2);
    getAllTheTemps(CHAN_SINGLE_3P,3);
    getAllTheTemps(CHAN_SINGLE_4P,4);
    getAllTheTemps(CHAN_SINGLE_5P,5);
    getAllTheTemps(CHAN_SINGLE_6P,6);
    getAllTheTemps(CHAN_SINGLE_7P,7);
    getAllTheTemps(CHAN_SINGLE_8P,8);
    getAllTheTemps(CHAN_SINGLE_9P,9);
    getAllTheTemps(CHAN_SINGLE_10P,10);
    getAllTheTemps(CHAN_SINGLE_11P,11);
    getAllTheTemps(CHAN_SINGLE_12P,12);
    getAllTheTemps(CHAN_SINGLE_13P,13);
    getAllTheTemps(CHAN_SINGLE_14P,14);
    getAllTheTemps(CHAN_SINGLE_15P,15);
  #endif
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
  Can0.write(sendTempMsg);
  #ifdef DEBUG
   Serial.print("  LEN: "); Serial.print(sendTempMsg.len);
    Serial.print(" EXT: "); Serial.print(sendTempMsg.flags.extended);
    Serial.print(" TS: "); Serial.print(sendTempMsg.timestamp);
    Serial.print(" ID: "); Serial.print(sendTempMsg.id, HEX);
    Serial.print(" Buffer: ");
    for ( uint8_t i = 0; i < sendTempMsg.len; i++ ) {
      Serial.print(sendTempMsg.buf[i], HEX); Serial.print(" ");
    } 
  Serial.println();
  //Serial.println("Sending Temp Data...");
  #endif
}
