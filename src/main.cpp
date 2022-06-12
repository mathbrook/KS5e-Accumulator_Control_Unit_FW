#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include <LTC2499.h>
#include <WireIMXRT.h>
#include <FreqMeasureMulti.h>
enum ACUSTATE {BOOTUP, RUNNING, FAULT};
int acuState=0;
#define runningFoReal
#ifdef runningFoReal
#define NUMBER_OF_LTCs 5
#define NUMBER_OF_CELLS 60
uint8_t gettingTempState=0; //0=set 1=wait 2=get
//Init ADCs
Ltc2499 theThings[6];
float batteryTempvoltages[NUMBER_OF_CELLS];
uint8_t ltcAddressList[]={ADDR_Z00,ADDR_Z0Z,ADDR_0Z0, //one two three
                           ADDR_ZZ0,ADDR_0ZZ}; //first 6 configurable addresses in the mf datasheet
byte ADCChannels[]={CHAN_SINGLE_0P,CHAN_SINGLE_1P,CHAN_SINGLE_2P,CHAN_SINGLE_3P,
                    CHAN_SINGLE_4P,CHAN_SINGLE_5P,CHAN_SINGLE_6P,CHAN_SINGLE_7P,
                    CHAN_SINGLE_8P,CHAN_SINGLE_9P,CHAN_SINGLE_10P,CHAN_SINGLE_11P};
elapsedMillis conversionTime;//wait 80ms for conversion to be ready
#endif
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
int8_t batteryTemps[NUMBER_OF_CELLS];
FreqMeasureMulti imdPWM; float sum1=0;int count1=0;
FreqMeasureMulti imdPWM2; float sum2=0;int count2=0;
int currentChannel=0;
int globalHighTherm=25, globalLowTherm=25;
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
Metro IMDPwmPrintTimer=Metro(500);
Metro fanSpeedMsgTimer=Metro(1000);
#define DEBUG
//printing received to serial
void canSniff(const CAN_message_t &msg);
void getTempData();
void sendTempData();
void ACUStateMachine();
int setChannels(int channelNo);
void setChannelsSwitchCase(int channelNo);
void getTemps(int channelNo);
void getImdPwm();
void DebuggingPrintout();
void controlFanSpeed();
void setup() {
  Wire.setClock(100000);
   Wire.begin();
  Serial.begin(115200); delay(400);
  Serial.println("Battery temp array: ");
  for(int i=0; i<NUMBER_OF_CELLS;i++){
    Serial.print("Cell number: ");
    Serial.print(i);
    Serial.print(" Value: ");
    // batteryTemps[i]=random(-40,80);
    batteryTemps[i]=0; //init default temps as a safe value
    Serial.println(batteryTemps[i]);
  }
  pinMode(9, OUTPUT); digitalWrite(9, HIGH); //Relay pin
  // pinMode(LED_BUILTIN,OUTPUT);
  for(int i=0;i<4;i++){
    pinMode(i, OUTPUT); digitalWrite(i, LOW); //indicator LEDs
  }
  // #ifdef runningFoReal
   for(int i=0;i<NUMBER_OF_LTCs;i++){
    byte ltcStatus=theThings[i].begin(ltcAddressList[i],5000);
    if(ltcStatus){
      Serial.print(ltcAddressList[i]);
      Serial.printf(", Error with LTC # %d",i);
      Serial.println();
      digitalWrite(0,HIGH);
      // while(1){};
    }
    else{
      Serial.printf("initialized LTC #%d with address %x\n",i,ltcAddressList[i]);
    } delay(100);
  }
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
  imdPWM.begin(8,FREQMEASUREMULTI_MARK_ONLY);
}
#ifdef runningFoReal

void getAllTheTemps(int channelNo){
  for(int i=0;i<NUMBER_OF_LTCs;i++){
    theThings[i].changeChannel(CHAN_SINGLE_0P);
  }
  conversionTime=0;
  delay(100);
  for(int i=0;i<NUMBER_OF_LTCs;i++){
    float voltageX=theThings[i].readVoltage();
    Serial.println(voltageX);
    int readingNumber=(i*12)+channelNo;
    batteryTempvoltages[readingNumber]=voltageX;
    #ifdef DEBUG
      char buffer[50];
      sprintf(buffer,"LTC Number: %d  Channel: %d Voltage: %f Reading Number: %d",i,channelNo,voltageX, readingNumber);
      Serial.println(buffer);
    #endif
  }
  // Serial.println("=================");
}
#endif
void loop() {
  getImdPwm();
  digitalWrite(LED_BUILTIN,LOW);
  Can0.events();
  if(doThingsRate.check()){
      ACUStateMachine();
  }
  if(sendTempRate.check()==1){
    digitalWrite(2,HIGH);
    sendTempData();
    //while(1){}
    digitalWrite(2,LOW);
    //DebuggingPrintout();
  }
  if(globalHighTherm>=60){
    digitalWrite(9,LOW);
  }else if(globalHighTherm<=59){
    digitalWrite(9,HIGH);
    controlFanSpeed();
  }
}
void canSniff(const CAN_message_t &msg) {
  //if(msg.id==BMS_Response_ID){
  // digitalWrite(LED_BUILTIN,HIGH);
  // Serial.print("MB "); Serial.print(msg.mb);
  // Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  // Serial.print("  LEN: "); Serial.print(msg.len);
  // Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  // Serial.print(" TS: "); Serial.print(msg.timestamp);
  // Serial.print(" ID: "); Serial.print(msg.id, HEX);
  // Serial.print(" Buffer: ");
  // for ( uint8_t i = 0; i < msg.len; i++ ) {
  //   Serial.print(msg.buf[i], HEX); Serial.print(" ");
  // } Serial.println();
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
  enabledTherm=NUMBER_OF_CELLS-1;//number of cells 0 based
  int lowTherm=batteryTemps[0], lowestThermId, highTherm=batteryTemps[0], highestThermId;
  for (int i=0;i<NUMBER_OF_CELLS;i++){ //get lowest and highest
  #ifdef DEBUG
  //  Serial.print("Cell number: ");
  //   Serial.print(i);
  //   Serial.print(" Value: ");
  //   Serial.println(batteryTemps[i]);
  #endif
    if(batteryTemps[i]<lowTherm){
      lowTherm=batteryTemps[i];
      lowestThermId=i;
    }
    if(batteryTemps[i]>highTherm){
      highTherm=batteryTemps[i];
      highestThermId=i;
    }
    // #ifdef DEBUG
    // Serial.printf("Iter: %d Highest: %d Lowest: %d\n",i,highTherm,lowTherm);
    // #endif
  }
  if(lowTherm<-40){
    lowTherm=-40;
  }
  if(highTherm>80){
    highTherm=80;
  }
  Serial.printf("Highest: %d Lowest: %d\n",highTherm,lowTherm);
  int avgTherm=(lowTherm+highTherm)/2;//yep
  int checksum=moduleNo+lowTherm+highTherm+avgTherm+enabledTherm+highestThermId+lowestThermId+57+8;//0x39 and 0x08 added to checksum per orion protocol
  byte tempdata[]={moduleNo,lowTherm,highTherm,avgTherm,enabledTherm,highestThermId,lowestThermId,checksum};
  memcpy(sendTempMsg.buf, tempdata, sizeof(sendTempMsg.buf));
  Can0.write(sendTempMsg);
  //GLobal ints for tracking
  globalHighTherm=highTherm;
  globalLowTherm=lowTherm;
}

int setChannels(int channelNo){
  for(int i=0;i<NUMBER_OF_LTCs;i++){
    theThings[i].changeChannel(ADCChannels[channelNo]);
    return channelNo;
  }
} 
void setChannelsSwitchCase(int channelNo){
  switch(channelNo){
    case 0:{
      for(int i=0;i<NUMBER_OF_LTCs;i++){
      theThings[i].changeChannel(CHAN_SINGLE_0P);
      }
    }
    break;
      case 1:{
      for(int i=0;i<NUMBER_OF_LTCs;i++){
      theThings[i].changeChannel(CHAN_SINGLE_1P);
      break;
      }
      }
      break;
      case 2:{
      for(int i=0;i<NUMBER_OF_LTCs;i++){
      theThings[i].changeChannel(CHAN_SINGLE_2P);
      break;
      }
      }
      break;
      case 3:{
      for(int i=0;i<NUMBER_OF_LTCs;i++){
      theThings[i].changeChannel(CHAN_SINGLE_3P);
      }
      }
      break;
      case 4:{
      for(int i=0;i<NUMBER_OF_LTCs;i++){
      theThings[i].changeChannel(CHAN_SINGLE_4P);
      }
      }
      break;
      case 5:{
      for(int i=0;i<NUMBER_OF_LTCs;i++){
      theThings[i].changeChannel(CHAN_SINGLE_5P);
      }
      }
      break;
      case 6:{
      for(int i=0;i<NUMBER_OF_LTCs;i++){
      theThings[i].changeChannel(CHAN_SINGLE_6P);
      break;
      }
      }
      break;
      case 7:{
      for(int i=0;i<NUMBER_OF_LTCs;i++){
      theThings[i].changeChannel(CHAN_SINGLE_7P);
      }
      }
      break;
      case 8:{
      for(int i=0;i<NUMBER_OF_LTCs;i++){
      theThings[i].changeChannel(CHAN_SINGLE_8P);
      }
      }
      break;
      case 9:{
      for(int i=0;i<NUMBER_OF_LTCs;i++){
      theThings[i].changeChannel(CHAN_SINGLE_9P);
      }
      }
      break;
      case 10:{
      for(int i=0;i<NUMBER_OF_LTCs;i++){
      theThings[i].changeChannel(CHAN_SINGLE_10P);
      }
      }
      break;
      case 11:{
      for(int i=0;i<NUMBER_OF_LTCs;i++){
      theThings[i].changeChannel(CHAN_SINGLE_11P);
      }
      }
      break;
  }
} 
void getTemps(int channelNo){
  for(int i=0;i<NUMBER_OF_LTCs;i++){
    float v=theThings[i].readVoltage();
    int cellNum=(i*12)+channelNo;
    batteryTempvoltages[cellNum]=v;
    float temp=(v*-79.256)+168.4;
    batteryTemps[cellNum]=temp;
    #ifdef DEBUG
      char buffer[100];
      sprintf(buffer,"LTC Number: %d CellNum: %d Channel: %d Reading: %f TempC: %f ",i,cellNum,channelNo,v,temp);
      Serial.println(buffer);
    #endif
  }
}
void ACUStateMachine(){
  // Serial.println("State:");
  // Serial.println(acuState);
  switch(acuState){
    case 0:
    {
      setChannelsSwitchCase(currentChannel);
      conversionTime=0;
      acuState=1;
      Serial.print("Setting Channels: ");
      break;
    }
    case 1:
      if(conversionTime>=100){
        acuState=2;
        Serial.println("Going to get readings");
        }
      break;
    case 2:
      Serial.println("reading channels");
      getTemps(currentChannel);
      acuState=0;
      currentChannel++;
      if(currentChannel>11){
      currentChannel=0;
      }
      break;
    }
  }
  void DebuggingPrintout(){
    for(int i=0;i<NUMBER_OF_CELLS;i++){
      Serial.print("Cell Number: "); Serial.print(i+1); Serial.print(" Temp: "); Serial.print(batteryTemps[i]);
      Serial.println();
    }
  }
  void getImdPwm(){
    float imdasdf=imdPWM.read();
    if (imdPWM.available()) {
    sum1 = sum1 + imdPWM.read();
    count1++;
    }
    if(IMDPwmPrintTimer.check()){
        if (count1 > 0) {
          float imdPWMfrequency=(imdPWM.countToFrequency(sum1 /count1)/2);
        Serial.print("FREQUENCY: ");Serial.println(imdPWM.countToFrequency(sum1 /count1)/2);
        float period=1/imdPWMfrequency; 
        Serial.print("PW: ");Serial.println((imdPWM.countToNanoseconds(sum1/count1)/1000000.0)/(period*10));
      } else {
        Serial.print("(no pulses)");
      }
      
      sum1=0;count1=0;
    }
  }
  void controlFanSpeed(){
    if(fanSpeedMsgTimer.check()){
    uint8_t fanSpeed=64;
    if(globalHighTherm>=25){
        fanSpeed=map(globalHighTherm,25,40,128,255);
    }
    Serial.print("Fan Speed: ");
    Serial.println(fanSpeed);
    CAN_message_t ctrlMsg;
      ctrlMsg.len=8;
      ctrlMsg.id=0xC5;
      uint8_t fanSpeedMsg[]={fanSpeed,0,0,0,1,1,0,0};
      memcpy(ctrlMsg.buf, fanSpeedMsg, sizeof(ctrlMsg.buf));
      Can0.write(ctrlMsg);
    }
  }