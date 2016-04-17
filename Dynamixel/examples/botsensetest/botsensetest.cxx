#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaCommBotSense.h"
#include "Dynamixel/DynaChain.h"
#include "botsense/libBotSense.h"
#include "botsense/BotSense.h"

int main()
{
  BsClient_P bsClient;
  DynaCommBotSense* com;
  com = new DynaCommBotSense("/dev/ttyUSB0",1000000,"localhost",9195);
  //com = DynaComm::New("botsense://localhost:9195:/dev/ttyUSB0",1000000);
  DynaChain* chain;
  
  bsClient = com->GetProxyClient();
  printf("client: %s\n",bsClientAttrGetName(bsClient));
  chain = new DynaChain(*com);
  
  
  //int rc = com->Open();
  
  printf("chain size: %d\n",chain->GetNumberInChain());
  chain->AddNewServosByScan();
  printf("chain size: %d\n",chain->GetNumberInChain());
  
  com->Close();
}
