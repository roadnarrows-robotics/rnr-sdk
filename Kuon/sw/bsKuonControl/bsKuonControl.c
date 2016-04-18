#include "libgen.h"

#include "botsense/bsKuon.h"
#include "botsense/bsKuonMsgs.h"
#include "Kuon/RS160DControl.h"
#include "rnr/hid/360Controller.h"
#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/opts.h"
#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "version.h"

#define APP_EC_OK       0         ///< success exit code
#define APP_EC_USAGE    2         ///< usage error exit code
#define APP_EC_EXEC     4         ///< execution error exit code

//
// The command and command-line options.
//
static char    *Argv0;                          ///< this command basename
static char    *OptsProxyServer = "localhost";  ///< proxy server addr/port
static bool_t   OptsServer      = false;        ///< executing on Kuon
static bool_t   OptsEcho        = false;        ///< echo controller output

//
// Forward declarations.
//
static int OptsCvtArgServerAddr(const char *argv0, const char *sOptName,
                            char *optarg, void *pOptVal);

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  .synopsis =
    "bsKuonControl is a manual control application for the\n Kuon platforms.",

  .long_desc = 
    "The %P application is an application which allows client systems to"
    " manuever the Kuon platforms using a standard wired Xbox360 controller,"
    " a tethered wireless Xbox360 controller, a wireless Xbox360 Controller,"
    " or a wireless Xbox360 Speed Wheel. Users can control top power/speed"
    " using the D-Pad. Basic throttle and steering is handled with the left"
    " stick on all controllers, or by turning the wheel and using the right"
    " trigger on the Speed Wheel. Left trigger on all devices places the Kuon"
    " platform into a coast mode, and the center X halts the application."
};

/*!
 * \brief Command-line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // -p, --proxy=<addr[:port]>
  {
    .long_opt   = "proxy",
    .short_opt  = 'p',
    .has_arg    = required_argument,
    .has_default= true,
    .opt_addr   = &OptsProxyServer,
    .fn_cvt     = OptsCvtArgServerAddr,
    .fn_fmt     = OptsFmtStr,
    .arg_name   = "<addr[:port]>",
    .opt_desc   = "Proxy Server's network address. The format of the address "
                  "can be either a network hostname or a dotted IP address "
                  "number. If port is not specfied, then the default port "
                  "9195 is used."
  },

  // -s, --server
  {
    .long_opt   = "server",
    .short_opt  = 's',
    .has_arg    = no_argument,
    .has_default= true,
    .opt_addr   = &OptsServer,
    .fn_cvt     = OptsCvtArgBool,
    .fn_fmt     = OptsFmtBool,
    .arg_name   = NULL,
    .opt_desc   = "This instance is executing as the daemon server on the main "
                  "Kuon processor. The server instance has higher priority "
                  "over host side client instances."
  },

  // -e, --echo
  {
    .long_opt   = "echo",
    .short_opt  = 'e',
    .has_arg    = no_argument,
    .has_default= true,
    .opt_addr   = &OptsEcho,
    .fn_cvt     = OptsCvtArgBool,
    .fn_fmt     = OptsFmtBool,
    .arg_name   = NULL,
    .opt_desc   = "Do [not] echo output of controller button actions to the "
                  "terminal."
  },

  {NULL, }
};

//
// State
//
static char      *ProxyIPAddr   = "localhost";  ///< default bsProxy IP addr
static int        ProxyIPPort   = BSPROXY_LISTEN_PORT_DFT;


/*!
 * \brief Convert command-line server option IP address string to 
 * network name/number and port number.
 *
 * \par Option Argument Syntax:
 * proxy:      addr[:port]
 *
 * \param argv0         Command name.
 * \param sOptName      Option name.
 * \param optarg        Parsed option argument to convert (optional).
 * \param[out] pOptVal  Pointer to converted option value (not used).
 *
 * \exception OptsInvalid()
 *
 * \return If returns, then returns 0.
 */
static int OptsCvtArgServerAddr(const char *argv0, const char *sOptName,
                                 char *optarg, void *pOptVal)
{
  char  *sSepField;
  char  *sPort;

  ProxyIPAddr = NULL;
  ProxyIPPort = 0;

  ProxyIPAddr = new_strdup(optarg);

  sSepField = strchr(ProxyIPAddr, ':');

  if( sSepField )
  {
    *sSepField    = 0;
    sPort         = sSepField+1;
    ProxyIPPort = (int)atol(sPort);
    if( ProxyIPPort <= 0 )
    {
      OptsInvalid(Argv0, "'%s': Invalid '%s' argument port value.",
          optarg, sOptName);
    }
  }
  else
  {
    ProxyIPPort = BSPROXY_LISTEN_PORT_DFT;
  }

  if( *ProxyIPAddr == 0 )
  {
    OptsInvalid(Argv0, "'%s': Invalid '%s' argument address value.",
        optarg, sOptName);
  }

  return 0;
}

/*!
 * \brief Initialize application.
 *
 * Any command-line error immediately terminates application.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 */
static void MainInit(int argc, char *argv[])
{
  // Name of this process.
  Argv0 = basename(argv[0]);

  // 
  // Parse input options.
  //
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);
}

int GetRightValue(int Why, int Ex, float Percentage) {
  int Return;
  float Percent;
  if(Why == 0) {
    Return = (-1)*Ex;
  }
  else {
    Return = (Why) - (Ex);
  }
  if(Return >= 250) {
    Return = 249;
  }
  if(Return <= -250) {
    Return = -249;
  }
  Percent = Percentage/100;
  return (Return*Percent);
}

int GetLeftValue( int Why, int Ex, float Percentage) {
  int Return;
  float Percent;
  if(Why == 0) {
    Return = 1 * Ex;
  }
  else {
    Return = (Why) + (Ex);
  }
  if(Return >= 250){
    Return = 249;
  }
  if(Return <= -250) {
    Return = -249;
  }
  Percent = Percentage/100;
  return (Return*Percent);
}

void ControlState(struct Control360 *Controller, int PercentVal){
  fprintf(stderr,"%*s\r", 50,"");
  if(Controller->Start) {
      fprintf(stderr,"%*sStart\r", 35, "");
  }
  if(Controller->Back) {
    fprintf(stderr,"%*sBack\r", 30, "");
  }
  if(Controller->Pad_Left) {
    fprintf(stderr,"%*s<\r", 28, "");
  }
  if(Controller->Pad_Right) {
    fprintf(stderr,"%*s>\r", 28, "");
  }
  if(Controller->Pad_Up) {
    fprintf(stderr,"%*s^\r", 28, "");
  }
  if(Controller->Pad_Down) {
    fprintf(stderr,"%*sv\r", 28, "");
  }
  if(Controller->A_Button) {
    fprintf(stderr,"%*sA\r", 26, "");
  }
  if(Controller->B_Button) {
    fprintf(stderr,"%*sB\r", 24, "");
  }
  if(Controller->Y_Button) {
    fprintf(stderr,"%*sY\r", 22, "");
  }
  if(Controller->X_Button) {
    fprintf(stderr,"%*sX\r", 20, "");
  }
  fprintf(stderr,"                Y:%d\r", Controller->Left_Y_Val);
  fprintf(stderr,"          X:%d\r", Controller->Left_X_Val);
  fprintf(stderr,"Power:%d\%\n", PercentVal);
}

int main(int argc, char *argv[]) {
  float Heading, Pitch, Roll;
  float tmpH, tmpP, tmpR;
  struct Control360 hid360;
  int CnT = 0;
  bool_t coast = false;
  int error, vConn, wasAController, i, justInLoop;
  int PercentFull = 25;

  MainInit(argc, argv);

  BsClient_P pClient = bsClientNew("kuon");

  error = bsServerConnect(pClient, ProxyIPAddr, ProxyIPPort);
  if(error < 0 ){
    LOGERROR("Can not connect to %s:%d. %s.\n",ProxyIPAddr, ProxyIPPort,
              bsStrError(error));
    exit(1);
  }

  LOGDIAG1("Connected as %s to bsProxy @%s:%d\n",
      (OptsServer? "server": "client"), ProxyIPAddr, ProxyIPPort);


  while(1) {
    justInLoop = 0;
    error = Init360Controller(&hid360);
    if(error == -1) {
      usleep(500);
      Init360Controller(&hid360);
    }
    Update360ControllerValues();
    if(hid360.Controller_Connected) {
      vConn = bsKuonReqOpen(pClient, "0", "1", OptsServer, false);
      if( vConn < 0 )
      {
        LOGERROR("Failed to open proxied Kuon.");
        return APP_EC_EXEC;
      }
      if(OptsEcho){
        if(hid360.type == 1){
          fprintf(stderr, "Wired 360 controller attached.\n");
        }
        if(hid360.type == 2){
          fprintf(stderr, "Tethered wireless 360 controller attached.\n");
        }
        if(hid360.type == 3){
          if(hid360.WorC == 1) {
            fprintf(stderr, "Wireless 360 controller attached.\n");
          }
          if(hid360.WorC == 2) {
            fprintf(stderr, "Wireless 360 speed wheel attached.\n");
          }
        }
      }
    }
    while(hid360.Controller_Connected) {
      wasAController = 1; 
      justInLoop = 1;
      Update360ControllerValues();
      
      if(hid360.Center_X) {
        LOGDIAG1("\n****Exiting****\n");
        bsKuonReqStop(pClient, vConn);
        bsKuonReqClose(pClient, vConn);
        Kill360Controller(&hid360);
        exit(APP_EC_OK);
      }
      if(hid360.Left_Trig_Val > 80){
        bsKuonReqStop(pClient, vConn);
      }
      else if(hid360.A_Button || hid360.type==1 || hid360.type==2 || hid360.WorC== 1) {
        if(hid360.Pad_Up) {
          if(PercentFull >= 90){
            PercentFull = 95;
          }
          else {
            PercentFull += 5;
          }
        }
        if(hid360.Pad_Down) {
          if(PercentFull <= 10){
            PercentFull = 5;
          }
          else {
            PercentFull -= 5;
          }
        }
        if((hid360.Right_Trig_Val >= 70) && !coast) {
          bsKuonReqAlterBrake(pClient, vConn, 0, 0, 0, 0);
          bsKuonReqAlterSlew(pClient, vConn, 40, 40, 40, 40);
          coast = true;
        }
        if((hid360.Right_Trig_Val < 70) && coast) {
          bsKuonReqAlterBrake(pClient, vConn, 31, 31, 31, 31);
          bsKuonReqAlterSlew(pClient, vConn, 0, 0, 0, 0);
          coast = false;
        }
        if(CnT == 1) {
          error = bsKuonReqReadImuDecoupAngles(pClient,vConn, &Heading, 
                                                       &Pitch, &Roll);
          if(error == 0) {
            fprintf(stderr,"Yaw:%.2f, Pitch:%.2f, Roll:%.2f\n", 
                                 Heading, Pitch, Roll);
          }
          CnT = 0;
        }
        bsKuonReqSetMotorSpeeds(pClient, vConn,

                           -1*GetLeftValue(hid360.Left_Y_Val,
                               hid360.Left_X_Val, PercentFull),

                           GetRightValue(hid360.Left_Y_Val,
                               hid360.Left_X_Val, PercentFull));
        CnT++;

      }
      else {
        bsKuonReqStop(pClient, vConn);
      }
      if(OptsEcho){
        ControlState(&hid360,PercentFull);
      }

      usleep(5000);
    }
    if(justInLoop){
      if(OptsEcho){
        fprintf(stderr,"\nController disconnected, please re-connect.\n");
        fprintf(stderr,"Halting robot for safety.\n");
      }
      bsKuonReqStop(pClient, vConn);
      bsKuonReqClose(pClient, vConn);
    }
    if(hid360.type == 3){
      wasAController = 1;
      hid360.WorC = 0;
    }
    hid360.type = 1;
    if(wasAController){
      LOGDIAG1("Controller disconnected.\n");
      Kill360Controller(&hid360);
      wasAController = 0;
    }
    LOGDIAG1("No controller connected.\n");
    sleep(1);
  }
  return APP_EC_OK;
}

