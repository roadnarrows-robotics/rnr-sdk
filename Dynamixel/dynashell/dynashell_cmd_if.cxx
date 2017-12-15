////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Program:   dynashell
//
// File:      dynashell_if.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-07-04 11:47:16 -0600 (Sat, 04 Jul 2015) $
 * $Rev: 4023 $
 *
 * \brief The Dynamixel Shell Interface Derived Commands.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////


#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>

#include <cstring>
#include <iostream>
#include <fstream>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaCommSerial.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaBgThread.h"

#include "dynashell.h"
#include "dynashell_cmd.h"
#include "dynashell_util.h"

using namespace std;


// -----------------------------------------------------------------------------
// DynaShellCmdCreate Class
// -----------------------------------------------------------------------------

/*!
 * \brief Create command.
 */
class DynaShellCmdCreate : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdCreate() : DynaShellCmd(2, 2)
  {
    m_sCmdName      = "create";
    m_sCmdHelpBrief = "Create a new dynamixel chain interface.";
    m_sCmdHelpArgs  = "<uri> <baudrate>";
    m_sCmdHelpDesc  = "Create a new, empty dynamixel chain interface. "
                      "A connection is opened to the (proxied) dynamixel bus "
                      "as specifed by the <uri> and at the specified "
                      "<baudrate>. The created dynamixel chain is empty. "
                      "Use scan to populate the chain.\n"
                      "  <uri>      "
                      "[botsense://[<hostname>][:<port>]]/<device>\n"
                      "  <baudrate> Baud rate. One of:\n"
                      "                9600 19200 57600 115200 200000 250000 "
                      "400000 500000 1000000\n"
                      "                2250000 2500000 3000000";
  }

  /*!
   * \brief Create dynamixel interface.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    char *sUri;
    int   nBaudRate;

    TRY( ChkArgCnt(shell, argc) );
  
    sUri = argv[0];

    TRY( ToInt(shell, argv[1], &nBaudRate) );

    if( shell.m_pDynaChain != NULL )
    {
      shell.Error("Dynamixel interface already created. Try destroying first.");
      return;
    }

    shell.m_pDynaComm = CommNew(shell, sUri, nBaudRate);

    TRY( shell.m_pDynaComm != NULL );

    shell.m_pDynaChain = new DynaChain(*shell.m_pDynaComm);

    // create background thread and register chain - leave in ready state
    if( shell.m_pDynaBgThread == NULL )
    {
      shell.m_pDynaBgThread = new DynaBgThread();
      shell.m_pDynaBgThread->RegisterChainAgent(shell.m_pDynaChain);
    }

    // register new chain with background thread - leave in ready state
    else
    {
      switch( shell.m_pDynaBgThread->GetCurrentState() )
      {
        case DynaBgThread::BgThreadStateRunning:
        case DynaBgThread::BgThreadStatePaused:
          shell.m_pDynaBgThread->Stop();
          break;
        default:
          break;
      }

      shell.m_pDynaBgThread->UnregisterAgent();
      shell.m_pDynaBgThread->RegisterChainAgent(shell.m_pDynaChain);
    }

    shell.Ok();
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes <device> <baudrate>
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST, then initialize any statics.
   * \param sContext  Generator context (i.e. canonical command path).
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext)
  {
    int   nBaudRate;
    char  buf[32];

    // <device>
    if( ReadLine::wc(sContext) == ReadLine::wc(m_sPubName) )
    {
      return ReadLine::FileCompletionGenerator(sText, nState);
    }

    //
    // New command to complete - initialize.
    //
    if( nState == ReadLine::FIRST )
    {
      m_nTabIter = 0;
    }

    // <baudrate>
    while( (nBaudRate = DynaComm::BaudRateAt(m_nTabIter)) != 0 )
    {
      m_nTabIter++;

      snprintf(buf, sizeof(buf), "%d", nBaudRate);
      buf[sizeof(buf)-1] = 0;

      if( !strncmp(buf, sText, uTextLen) )
      {
        return ReadLine::dupstr(buf);
      }
    }

    // no more matches
    return NULL;
  }

protected:
  int m_nTabIter;     ///< tab completion: baud rate iterator

  /*!
   * \brief Create a new open dynamixel bus object.
   *
   * \param shell     Dynamixel shell.
   * \param sUri      Dynamixel bus [proxied] device Uniform Resource Id.
   * \param nBaudRate Device and servo baudrate.
   *
   * \return Returns new DynaComm object on success, NULL on failure.
   */
  DynaComm *CommNew(DynaShell &shell, const char *sUri, int nBaudRate)
  {
    DynaComm *pDynaComm = DynaComm::New(sUri, nBaudRate);

    if( pDynaComm == NULL )
    {
      shell.Error("Failed to create %s@%d.", sUri, nBaudRate);
    }

    else if( !pDynaComm->IsOpen() )
    {
      shell.Error("Failed to open %s@%d.", sUri, nBaudRate);
      delete pDynaComm;
      pDynaComm = NULL;
    }

    return pDynaComm;
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdDestroy Class
// -----------------------------------------------------------------------------

/*!
 * \brief Destroy command.
 */
class DynaShellCmdDestroy : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdDestroy() : DynaShellCmd(0, 0)
  {
    m_sCmdName      = "destroy";
    m_sCmdHelpBrief = "Destroy the dynamixel interface.";
    m_sCmdHelpArgs  = "";
    m_sCmdHelpDesc  = "Destroy the existing dynamixel interface. Any existing "
                      "dynamixel bus communication is closed.";
  }

  /*!
   * \brief Destroy dynamixel interface.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    TRY( ChkArgCnt(shell, argc) );

    if( shell.m_pDynaChain == NULL )
    {
      shell.Error("Dynamixel interface not created.");
    }
    else
    {
      switch( shell.m_pDynaBgThread->GetCurrentState() )
      {
        case DynaBgThread::BgThreadStateRunning:
        case DynaBgThread::BgThreadStatePaused:
          shell.m_pDynaBgThread->Stop();
          break;
        default:
          break;
      }

      shell.m_pDynaBgThread->UnregisterAgent();

      DELOBJ(shell.m_pDynaChain);
      DELOBJ(shell.m_pDynaComm);

      shell.Ok();
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdScan Class
// -----------------------------------------------------------------------------

/*!
 * \brief Scan command.
 */
class DynaShellCmdScan : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdScan() : DynaShellCmd(0, 0)
  {
    m_sCmdName      = "scan";
    m_sCmdHelpBrief = "Scan for servos.";
    m_sCmdHelpArgs  = "";
    m_sCmdHelpDesc  = "Scan the dynamixel bus for all servos and add to the "
                      "chain. The chain is cleared of existing servos prior to "
                      "the scan.";
  }

  /*!
   * \brief Scan for all connected servos and add to chain.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    DynaBgThread::BgThreadState eOldState;
    int                         nNumServos;

    TRY( ChkArgCnt(shell, argc) );
    TRY( ChkComm(shell) );
    TRY( ChkChain(shell) );

    switch( (eOldState = shell.m_pDynaBgThread->GetCurrentState()) )
    {
      case DynaBgThread::BgThreadStateRunning:
      case DynaBgThread::BgThreadStatePaused:
        shell.m_pDynaBgThread->Stop();
        break;
      default:
        break;
    }

    shell.m_pDynaBgThread->UnregisterAgent();

    shell.m_pDynaChain->AddNewServosByScan();

    shell.m_pDynaBgThread->RegisterChainAgent(shell.m_pDynaChain);

    nNumServos = shell.m_pDynaChain->GetNumberInChain();

    if( (eOldState == DynaBgThread::BgThreadStateRunning) && (nNumServos > 0) )
    {
      shell.m_pDynaBgThread->Run();
    }

    shell.Response("Added %d servos to chain.\n", nNumServos);
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdPing Class
// -----------------------------------------------------------------------------

/*!
 * \brief Ping command.
 */
class DynaShellCmdPing : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdPing() : DynaShellCmd(1, 1)
  {
    m_sCmdName      = "ping";
    m_sCmdHelpBrief = "Ping a servo.";
    m_sCmdHelpArgs  = "<servo_id>";
    m_sCmdHelpDesc  = "Ping a servo. The servo does not have to be present "
                      "in the dynamixel chain.\n"
                      "  <servo_id>  Servo id [0-253].";
  }

  /*!
   * \brief Ping a servo.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    int     nServoId;
    bool    bPong;

    TRY( ChkArgCnt(shell, argc) );
    TRY( ToInt(shell, argv[0], &nServoId) );
    TRY( ChkComm(shell) );

    bPong = shell.m_pDynaComm->Ping(nServoId);

    shell.Response("Servo %d: %s.\n", nServoId,
        (bPong? "present": "no response"));
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdList Class
// -----------------------------------------------------------------------------

/*!
 * \brief List command.
 */
class DynaShellCmdList : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdList() : DynaShellCmd(0, 0)
  {
    m_sCmdName      = "list";
    m_sCmdHelpBrief = "List servo information.";
    m_sCmdHelpArgs  = "";
    m_sCmdHelpDesc  = "List information about all servos in the dynamixel "
                      "chain. Per each servo, the servo id, model number "
                      "firmware version, mode, and linkage, if any, are "
                      "printed.";
  }

  /*!
   * \brief List servo chain info.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    int               nServoId;
    const DynaServo  *pServo;
    int               iter;

    TRY( ChkArgCnt(shell, argc) );
    TRY( ChkComm(shell) );
    TRY( ChkChain(shell) );

    for(nServoId = shell.m_pDynaChain->IterStart(&iter);
        nServoId != DYNA_ID_NONE;
        nServoId = shell.m_pDynaChain->IterNext(&iter))
    {
      pServo = shell.m_pDynaChain->GetServo(nServoId);
      shell.Response("Servo %3d: %7s, model 0x%04x, fwver 0x%02x",
          nServoId, pServo->GetModelName(),
          pServo->GetModelNumber(), pServo->GetFirmwareVersion());
      if( pServo->GetServoMode() == DYNA_MODE_SERVO )
      {
        shell.Response(", servo mode");
      }
      else
      {
        shell.Response(", continuous mode");
      }
      if( shell.m_pDynaChain->IsLinkedMaster(nServoId) )
      {
        shell.Response(", linked master of %d",
            shell.m_pDynaChain->GetLinkedMateId(nServoId));
      }
      else if( shell.m_pDynaChain->IsLinkedSlave(nServoId) )
      {
        shell.Response(", linked slave of %d",
            shell.m_pDynaChain->GetLinkedMateId(nServoId));
      }
      shell.Response("\n");
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdLink Class
// -----------------------------------------------------------------------------

/*!
 * \brief Link <master_id> <slaf_id> <dir> command.
 */
class DynaShellCmdLink : public DynaShellCmd
{
public:
  /*
   * \brief Default constructor.
   */
  DynaShellCmdLink() : DynaShellCmd(3, 3)
  {
    m_sCmdName      = "link";
    m_sCmdHelpBrief = "Link two servos in a master-slave configuration.";
    m_sCmdHelpArgs  = "<master_id> <slave_id> <dir>";
    m_sCmdHelpDesc  = "Link two servos in a master-slave configuration. "
                      "The servos may operate in the same forward rotation "
                      "direction or in opposite reverse directions. Once "
                      "linked, all commands that modify servo state "
                      "(e.g. move and write) are only supported on the master "
                      "servo to preserve synchronization.\n"
                      "  <master_id>  Master servo id.\n"
                      "  <slave_id>   Slave servo id.\n"
                      "  <dir>        One of: forward reverse.";
  }

  /*!
   * \brief Link two servo together.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    int     nServoIdMaster;
    int     nServoIdSlave;
    bool    bIsReversed;
    int     rc;

    TRY( ChkArgCnt(shell, argc) );

    TRY( ToInt(shell, argv[0], &nServoIdMaster) );
    TRY( ToInt(shell, argv[1], &nServoIdSlave) );

    if( !strncmp("forward", argv[2], strlen(argv[2])) )
    {
      bIsReversed = false;
    }
    else if( !strncmp("reverse", argv[2], strlen(argv[2])) )
    {
      bIsReversed = true;
    }
    else
    {
      shell.Error(-DYNA_ECODE_BAD_VAL, "Unknown linkage <dir> '%s'", argv[2]);
      return;
    }

    TRY( ChkComm(shell) );
    TRY( ChkChain(shell) );
    TRY( ChkChainHasServo(shell, nServoIdMaster) );
    TRY( ChkChainHasServo(shell, nServoIdSlave) );

    rc = shell.m_pDynaChain->LinkServos(nServoIdMaster,
                                        nServoIdSlave,
                                        bIsReversed);

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Cannot link servos %d and %d.",
                              nServoIdMaster, nServoIdSlave);
    }
    else
    {
      shell.Ok();
    }
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes <master_id> <slave_id> <dir>
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST, then initialize any statics.
   * \param sContext  Generator context (i.e. canonical command path).
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext)
  {
    // direction keyword table
    static const char *keywordsDir[] = {"forward", "reverse"}; 

    int         nArgNum;
    int         nServoId;
    const char *s;
    char        buf[16];
  
    if( (shell.m_pDynaChain == NULL) ||
        (shell.m_pDynaChain->GetNumberInChain() == 0) )
    {
      return NULL;
    }
  
    // argument number of already (expanded) arguments
    nArgNum = ReadLine::wc(sContext) - ReadLine::wc(m_sPubName);

    //
    // New command argument to complete - initialize.
    //
    if( nState == ReadLine::FIRST )
    {
      m_nTabServoId = shell.m_pDynaChain->IterStart(&m_nTabIter);
      m_nTabIndex   = 0;
    }
  
    // <master_id> or <slave_id>
    if( nArgNum < 2 )
    {
      while( m_nTabServoId != DYNA_ID_NONE )
      {
        nServoId = m_nTabServoId;

        m_nTabServoId = shell.m_pDynaChain->IterNext(&m_nTabIter);

        if( !shell.m_pDynaChain->IsUnlinked(m_nTabServoId) )
        {
          continue;
        }

        snprintf(buf, sizeof(buf), "%d", nServoId);
        buf[sizeof(buf)-1] = 0;
  
        if( !strncmp(buf, sText, uTextLen) )
        {
          return ReadLine::dupstr(buf);
        }
      }
    }

    // <dir>
    else if( nArgNum == 2 )
    {
      while( m_nTabIndex < arraysize(keywordsDir) )
      {
        s = keywordsDir[m_nTabIndex++];

        if( !strncmp(s, sText, uTextLen) )
        {
          return ReadLine::dupstr(s);
        }
      }
    }

    // no more matches
    return NULL;
  }

protected:
  int  m_nTabIter;        ///< tab completion: servo id iterator
  int  m_nTabServoId;     ///< tab completion: current servo id
  int  m_nTabIndex;       ///< tab completion: keyword index
};



// -----------------------------------------------------------------------------
// DynaShellCmdUnlink Class
// -----------------------------------------------------------------------------

/*!
 * \brief List command.
 */
class DynaShellCmdUnlink : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdUnlink() : DynaShellCmd(1, 1)
  {
    m_sCmdName      = "unlink";
    m_sCmdHelpBrief = "Unlink two servos.";
    m_sCmdHelpArgs  = "<master_id>";
    m_sCmdHelpDesc  = "Unlink two servos previously linked in a master-slave "
                      "configuration.\n"
                      "  <master_id>  Master servo id.";
  }

  /*!
   * \brief Unlink two servo.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    int     nServoId;
    int     rc;

    TRY( ChkArgCnt(shell, argc) );

    TRY( ToInt(shell, argv[0], &nServoId) );

    TRY( ChkComm(shell) );
    TRY( ChkChain(shell) );
    TRY( ChkChainHasServo(shell, nServoId) );

    rc = shell.m_pDynaChain->UnlinkServos(nServoId);

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Cannot unlink servo %d.", nServoId);
    }
    else
    {
      shell.Ok();
    }
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes <master_id>
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST, then initialize any statics.
   * \param sContext  Generator context (i.e. canonical command path).
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext)
  {
    int         nServoId;
    const char *s;
    char        buf[16];
  
    if( (shell.m_pDynaChain == NULL) ||
        (shell.m_pDynaChain->GetNumberInChain() == 0) )
    {
      return NULL;
    }
  
    // argument number of already (expanded) arguments
    else if( (ReadLine::wc(sContext) - ReadLine::wc(m_sPubName)) > 0 )
    {
      return NULL;
    }

    //
    // New command argument to complete - initialize.
    //
    if( nState == ReadLine::FIRST )
    {
      m_nTabServoId = shell.m_pDynaChain->IterStartMaster(&m_nTabIter);
    }
  
    // <master_id> 
    while( m_nTabServoId != DYNA_ID_NONE )
    {
      nServoId = m_nTabServoId;

      m_nTabServoId = shell.m_pDynaChain->IterNextMaster(&m_nTabIter);

      if( !shell.m_pDynaChain->IsLinkedMaster(m_nTabServoId) )
      {
        continue;
      }

      snprintf(buf, sizeof(buf), "%d", nServoId);
      buf[sizeof(buf)-1] = 0;
  
      if( !strncmp(buf, sText, uTextLen) )
      {
        return ReadLine::dupstr(buf);
      }
    }

    // no more matches
    return NULL;
  }

protected:
  int  m_nTabIter;        ///< tab completion: servo id iterator
  int  m_nTabServoId;     ///< tab completion: current servo id
};


// -----------------------------------------------------------------------------
// DynaShellCmdMegaScan Class
// -----------------------------------------------------------------------------

/*!
 * \brief MegaScan command.
 */
class DynaShellCmdMegaScan : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdMegaScan() : DynaShellCmd(1, 1)
  {
    m_sCmdName      = "megascan";
    m_sCmdHelpBrief = "Exhaustive scan for all connected servos.";
    m_sCmdHelpArgs  = "<uri>";
    m_sCmdHelpDesc  = "Scan the interface at all supported baud rates for "
                      "connected servos. The scan does not effect the "
                      "current dynamixel chain.\n"
                      "  <uri>  [botsense://[<hostname>][:<port>]]/<device>."
                      "\n\n"
                      "Note: The servos may respond to a different baudrate "
                      "other the what is saved\n"
                      "      in the EEPROM.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdMegaScan() { }

  /*!
   * \brief Scan interface at all supported baudrates for connected
   * servos.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    static int baudTbl[] =
    {
      1000000, 500000, 400000, 250000, 200000, 115200, 57600, 19200, 9600
    };

    char       *sUri;
    DynaComm   *pComm;
    int         nBaudRate;
    int         i;
    int         nServoId;
    uint_t      cnt;
    uint_t      uModelNum;

    TRY( ChkArgCnt(shell, argc) );
  
    sUri = argv[0];

    shell.Response("Mega Scan on Interface %s.\n", sUri);

    for(i=0; i<arraysize(baudTbl); ++i)
    {
      shell.Response("%8d baudrate\n", baudTbl[i]);

      pComm = CommNew(shell, sUri, baudTbl[i]);

      if( pComm == NULL )
      {
        continue;
      }

      for(nServoId=DYNA_ID_MIN, cnt=0; nServoId<DYNA_ID_NUMOF; ++nServoId)
      {
        if( pComm->Ping(nServoId) )
        {
          if( DynaServo::ReadModelNumber(*pComm, nServoId, &uModelNum) < 0 )
          {
            uModelNum = DYNA_MODEL_NUM_GENERIC;
          }
          shell.Response("%4d:0x%04x", nServoId, uModelNum);
          fflush(stdout);
          ++cnt;
          if( (cnt % 5) == 0 )
          {
            shell.Response("\n");
          }
        }
      }

      if( cnt > 0 )
      {
        if( (cnt % 5) != 0 )
        {
          shell.Response("\n");
        }
        shell.Response(" %d servos found @%d baudrate\n", cnt, baudTbl[i]);
      }

      delete pComm;
    }
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes <device>
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST, then initialize any statics.
   * \param sContext  Generator context (i.e. canonical command path).
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext)
  {
    // <device>
    return ReadLine::FileCompletionGenerator(sText, nState);
  }

protected:

  /*!
   * \brief Create a new open dynamixel bus object.
   *
   * \param shell     Dynamixel shell.
   * \param sUri      Dynamixel bus [proxied] device Uniform Resource Id.
   * \param nBaudRate Device and servo baudrate.
   *
   * \return Returns new DynaComm object on success, NULL on failure.
   */
  DynaComm *CommNew(DynaShell &shell, const char *sUri, int nBaudRate)
  {
    DynaComm *pDynaComm = DynaComm::New(sUri, nBaudRate);

    if( pDynaComm == NULL )
    {
      shell.Error("Failed to create %s@%d.", sUri, nBaudRate);
    }

    else if( !pDynaComm->IsOpen() )
    {
      shell.Error("Failed to open %s@%d.", sUri, nBaudRate);
      delete pDynaComm;
      pDynaComm = NULL;
    }

    return pDynaComm;
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdBgStart Class
// -----------------------------------------------------------------------------

/*!
 * \brief Start the background thread.
 */
class DynaShellCmdBgtStart : public DynaShellCmd
{
public:
  /*
   * \brief Default constructor.
   */
  DynaShellCmdBgtStart() : DynaShellCmd(0, 1)
  {
    m_sCmdName      = "start";
    m_sCmdHelpBrief = "Start the dynamixel background thread.";
    m_sCmdHelpArgs  = "[<hz>]";
    m_sCmdHelpDesc  = "Start the background thread with the given or default "
                      "control and monitor execute hertz.\n"
                      "  <hz>   Execute hertz.\n"
                      "             Default: 50.0\n";

    SetTabTDefaults();
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdBgtStart() { }

  /*!
   * \brief Execute background thread start.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    int     i;
    double  val;
    double  fHz;

    TRY( ChkArgCnt(shell, argc) );

    TRY( ChkComm(shell) );
    TRY( ChkChainNotEmpty(shell) );

    switch( shell.m_pDynaBgThread->GetCurrentState() )
    {
      case DynaBgThread::BgThreadStateReady:
      case DynaBgThread::BgThreadStatePaused:
        break;
      case DynaBgThread::BgThreadStateRunning:
      default:
        shell.Error("Background thread already running. "
                    "Try stopping first.");
        return;
    }

    if( argc > 0 )
    {
      TRY( ToDouble(shell, argv[0], &val) );
      fHz = val;
    }
    else
    {
      fHz = shell.m_pDynaBgThread->getHz();
    }

    shell.m_pDynaBgThread->setHz(fHz);

    shell.m_pDynaBgThread->Run();

    shell.Response("Background thread started.\n");
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes t_posctl_dft|notask t_dynamics_dft|notask t_health_dft|notask
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST, then initialize any statics.
   * \param sContext  Generator context (i.e. canonical command path).
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext)
  {
    int   nArgNum;
    char  buf[16];
  
    if( (shell.m_pDynaChain == NULL) ||
        (shell.m_pDynaChain->GetNumberInChain() == 0) )
    {
      return NULL;
    }
  
    // argument number of already (expanded) arguments
    nArgNum = ReadLine::wc(sContext) - ReadLine::wc(m_sPubName);

    if( nArgNum > m_nArgCntMax )
    {
      return NULL;
    }

    //
    // New command to complete - initialize.
    //
    if( nState == ReadLine::FIRST )
    {
      m_nTabIndex = 0;
    }
  
    // can only expand default keyword
    while( m_nTabIndex < 2 )
    {
      ++m_nTabIndex;

      if( m_nTabIndex == 1 )
      {
        if( !strncmp(m_strTDefault.c_str(), sText, uTextLen) )
        {
          return ReadLine::dupstr(m_strTDefault);
        }
      }
    }
  
    // no more matches
    return NULL;
  }

protected:
  int         m_nTabIndex;                  ///< tab completion: index
  string      m_strTDefault;                ///< default time period value

  /*!
   * \brief Set tab completion time period default strings.
   */
  void SetTabTDefaults()
  {
    char  buf[32];

    snprintf(buf, sizeof(buf), "%lf", DynaBgThread::HZ_EXEC_DFT);
    buf[sizeof(buf)-1] = 0;
    m_strTDefault = buf;
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdBgtStop Class
// -----------------------------------------------------------------------------

/*!
 * \brief Start the background thread.
 */
class DynaShellCmdBgtStop : public DynaShellCmd
{
public:
  /*
   * \brief Default constructor.
   */
  DynaShellCmdBgtStop() : DynaShellCmd(0, 0)
  {
    m_sCmdName      = "stop";
    m_sCmdHelpBrief = "Stop the dynamixel background thread.";
    m_sCmdHelpArgs  = "";
    m_sCmdHelpDesc  = "The dynamixel background thread stopped. The thread is "
                      "not destroyed, simply blocked, and can be resumed.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdBgtStop() { }

  /*!
   * \brief Execute background thread stop.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    TRY( ChkArgCnt(shell, argc) );

    TRY( ChkComm(shell) );
    TRY( ChkChainNotEmpty(shell) );

    switch( shell.m_pDynaBgThread->GetCurrentState() )
    {
      case DynaBgThread::BgThreadStateRunning:
        shell.m_pDynaBgThread->Stop();
        break;
      case DynaBgThread::BgThreadStateReady:
      case DynaBgThread::BgThreadStatePaused:
      default:
        shell.Error("Background thread not running.");
        return;
    }

    shell.Response("Background thread stopped.\n");
  }

protected:
};


// -----------------------------------------------------------------------------
// DynaShellCmdBgtState Class
// -----------------------------------------------------------------------------

/*!
 * \brief Start the background thread.
 */
class DynaShellCmdBgtGetState : public DynaShellCmd
{
public:
  /*
   * \brief Default constructor.
   */
  DynaShellCmdBgtGetState() : DynaShellCmd(0, 0)
  {
    m_sCmdName      = "get";
    m_sCmdHelpBrief = "Get the state of the dynamixel background thread.";
    m_sCmdHelpArgs  = "";
    m_sCmdHelpDesc  = "The dynamixel background thread state includes its "
                      "operational state plus any task time periods.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdBgtGetState() { }

  /*!
   * \brief Execute background thread get state.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    const char *sState;

    TRY( ChkArgCnt(shell, argc) );

    TRY( ChkComm(shell) );
    TRY( ChkChainNotEmpty(shell) );

    switch( shell.m_pDynaBgThread->GetCurrentState() )
    {
      case DynaBgThread::BgThreadStateZombie:
        sState = "Zombie";
        break;
      case DynaBgThread::BgThreadStateReady:
        sState = "Stopped";
        break;
      case DynaBgThread::BgThreadStateRunning:
        sState = "Running";
        break;
      case DynaBgThread::BgThreadStatePaused:
        sState = "Paused";
        break;
      case DynaBgThread::BgThreadStateExit:
        sState = "Exiting";
        break;
      default:
        sState = "Unknown";
        break;
    }

    shell.Response("Background Thread: %s, Hz %lf.\n", 
                  sState, shell.m_pDynaBgThread->getHz());
  }

protected:
};


// -----------------------------------------------------------------------------
// DynaShellCmdSetHalfDuplexCtl Class
// -----------------------------------------------------------------------------

/*!
 * \brief Set halfduplex control command.
 */
class DynaShellCmdSetHalfDuplexCtl : public DynaShellCmd
{
public:
  /*
   * \brief Default constructor.
   */
  DynaShellCmdSetHalfDuplexCtl() : DynaShellCmd(1, 2)
  {
    m_sCmdName      = "halfduplex";
    m_sCmdHelpBrief = "Set/unset sofware half-duplex control.";
    m_sCmdHelpArgs  = "none\n"
                      "modem {rts | cts}\n"
                      "gpio <number>";
    m_sCmdHelpDesc  = "Dynamixel 3-wire serial interface is a half-duplex "
                      "channel. Software can control the channel direction by "
                      "toggling between transmit and receive using a signal.\n"
                      " Signaling types:\n"
                      "  none   Disable software half-duplex control.\n"
                      "  modem  Use built-in modem signal. Signals:\n"
                      "           rts - request to send.\n"
                      "           cts - clear to send.\n"
                      "  <gpio> Use GPIO signal (future).\n"
                      "           <number> - GPIO pin number. If negative,\n"
                      "                      then transmit is active low.";
  }

  /*!
   * \brief Set software half-duplex control signal.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    int     nSignal;
    int     rc;

    TRY( ChkArgCnt(shell, argc) );
    TRY( ChkComm(shell) );

    if( !strncmp("none", argv[0], strlen(argv[0])) )
    {
      nSignal = DynaCommSerial::CtlSignalNone;
    }
    else if( !strncmp("modem", argv[0], strlen(argv[0])) )
    {
      TRY( ChkArgCntEQ(shell, argc, 2) );
      if( !strncmp("cts", argv[1], strlen(argv[1])) )
      {
        nSignal = DynaCommSerial::CtlSignalModemCTS;
      }
      else if( !strncmp("rts", argv[1], strlen(argv[1])) )
      {
        nSignal = DynaCommSerial::CtlSignalModemRTS;
      }
      else
      {
        shell.Error(-DYNA_ECODE_BAD_VAL, "Unknown modem signal '%s'", argv[1]);
        return;
      }
    }
    else if( !strncmp("gpio", argv[0], strlen(argv[0])) )
    {
      TRY( ChkArgCntEQ(shell, argc, 2) );
      TRY( ToInt(shell, argv[1], &nSignal) );
    }
    else
    {
      shell.Error(-DYNA_ECODE_BAD_VAL, "Unknown signal type '%s'", argv[0]);
      return;
    }

    rc = shell.m_pDynaComm->SetHalfDuplexCtl(nSignal);

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Failed to set half-duplex %s signal control.", argv[0]);
    }
    else
    {
      shell.Ok();
    }
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes none | modem sig | gpio number
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST, then initialize any statics.
   * \param sContext  Generator context (i.e. canonical command path).
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext)
  {
    static const char *keywordsSigType[]  = {"gpio", "modem", "none"}; 
    static const char *keywordsSigModem[] = {"cts", "rts"}; 
    static char        cSigType = '\0';

    int         nArgNum;
    int         nServoId;
    const char *s;
    char        buf[16];
  
    // argument number of already (expanded) arguments
    nArgNum = ReadLine::wc(sContext) - ReadLine::wc(m_sPubName);

    //
    // New command argument to complete - initialize.
    //
    if( nState == ReadLine::FIRST )
    {
      m_nTabIndex   = 0;
    }
  
    // gpio | modem | none
    if( nArgNum == 0 )
    {
      while( m_nTabIndex < arraysize(keywordsSigType) )
      {
        s = keywordsSigType[m_nTabIndex++];

        if( !strncmp(s, sText, uTextLen) )
        {
          cSigType = *s;
          return ReadLine::dupstr(s);
        }
      }
    }

    else if( nArgNum == 1 )
    {
      // rts | cts
      if( cSigType == 'm' )
      {
        while( m_nTabIndex < arraysize(keywordsSigModem) )
        {
          s = keywordsSigModem[m_nTabIndex++];

          if( !strncmp(s, sText, uTextLen) )
          {
            return ReadLine::dupstr(s);
          }
        }
      }
      // NUM
      else if( cSigType == 'g' )
      {
      }
      // 
      else if( cSigType == 'n' )
      {
      }
    }

    // no more matches
    return NULL;
  }

protected:
  int  m_nTabIndex;       ///< tab completion: keyword index
};


// -----------------------------------------------------------------------------
// Public Interface.
// -----------------------------------------------------------------------------

/*!
 * \brief Publish shell dynamixel interface commands to shell.
 *
 * The dynamixel interface command set contains the top-level dynamixel
 * library semantic fundamentals.
 *
 * \brief shell   Dynamixel shell.
 */
void PublishShellInterfaceCommands(DynaShell &shell)
{
  shell.PublishCommand(NULL, new DynaShellCmdCreate());
  shell.PublishCommand(NULL, new DynaShellCmdDestroy());
  shell.PublishCommand(NULL, new DynaShellCmdScan());
  shell.PublishCommand(NULL, new DynaShellCmdPing());
  shell.PublishCommand(NULL, new DynaShellCmdList());
  shell.PublishCommand(NULL, new DynaShellCmdLink());
  shell.PublishCommand(NULL, new DynaShellCmdUnlink());
  shell.PublishCommand(NULL, new DynaShellCmdMegaScan());
  shell.PublishCommand("bg", new DynaShellCmdBgtStart());
  shell.PublishCommand("bg", new DynaShellCmdBgtStop());
  shell.PublishCommand("bg", new DynaShellCmdBgtGetState());
  shell.PublishCommand("set", new DynaShellCmdSetHalfDuplexCtl());
}
