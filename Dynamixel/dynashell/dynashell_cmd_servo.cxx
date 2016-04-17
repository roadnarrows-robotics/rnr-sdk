////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Program:   dynashell
//
// File:      dynashell_servo.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief The Dynamixel Shell Servo Derived Commands.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2015  RoadNarrows LLC.
 * \n All Rights Reserved
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

#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaChain.h"

#include "dynashell.h"
#include "dynashell_cmd.h"
#include "dynashell_util.h"

using namespace std;

/*!
 * \brief Convert rotation direction to string.
 *
 * \param nSpeed  Speed.
 *
 * \return Short string.
 */
inline const char *rotdirstr(int nSpeed)
{
  if( nSpeed < 0 )
  {
    return "(cw)";
  }
  else if( nSpeed == 0 )
  {
    return "";
  }
  else
  {
    return "(ccw)";
  }
}


// -----------------------------------------------------------------------------
// DynaShellCmdMoveTo Class
// -----------------------------------------------------------------------------

/*!
 * \brief Determine if servos are moving.
 */
class DynaShellCmdMoveTo : public DynaShellCmdChainOut
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdMoveTo() : DynaShellCmdChainOut(true)
  {
    m_sCmdName      = "moveto";
    m_sCmdHelpBrief = "Move servos to goal positions.";
    m_sCmdHelpArgs  = "<servo_id> <pos> [<servo_id> <pos> ...]\n"
                      "chain <pos>";
    m_sCmdHelpDesc  = "Move the specified servos to the goal positions. "
                      "If more than one servo is specifed then a synchronous "
                      "move will be automatically performed. "
                      "If the keyword 'chain' is specified, then all servos in "
                      "the chain are synchronously moved.\n"
                      "  <servo_id>  Servo id [0-253].\n"
                      "  <pos>       Odometer goal position.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdMoveTo() { }

protected:
  /*!
   * \brief Move servos to goal positions.
   *
   * \param shell   Dynamixel shell.
   * \param tup     Array of (servo id, goal position) 2-tuples.
   * \param uCount  Number of tuples.
   */
  virtual void doExec(DynaShell &shell, ExecTup_T tup[], uint_t uCount)
  {
    uint_t    i;
    int       rc;

    if( uCount == 1 )
    {
      rc = shell.m_pDynaChain->MoveTo(tup[0].m_nServoId, tup[0].m_nVal);

      if( rc != DYNA_OK )
      {
        shell.Error(rc, "Servo %d: move to %d failed.",
                                tup[0].m_nServoId, tup[0].m_nVal);
      }
    }
    else // > 1
    {
      DynaPosTuple_T  tupPos[uCount];

      for(i=0; i<uCount; ++i)
      {
        tupPos[i].m_nServoId = tup[i].m_nServoId;
        tupPos[i].m_nPos     = tup[i].m_nVal;
      }

      rc = shell.m_pDynaChain->SyncMoveTo(tupPos, uCount);

      if( rc != DYNA_OK )
      {
        shell.Error(rc, "Synchronous move of %u servos failed.", uCount);
      }
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdMoveAtSpeedTo Class
// -----------------------------------------------------------------------------

/*!
 * \brief Determine if servos are moving.
 */
class DynaShellCmdMoveAtSpeedTo : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdMoveAtSpeedTo() : DynaShellCmd(3, 3*DYNA_ID_NUMOF)
  {
    m_sCmdName      = "moveatspeedto";
    m_sCmdHelpBrief = "Move servos at speeds to goal positions.";
    m_sCmdHelpArgs  = "<servo_id> <speed> <pos> [<servo_id> <speed> "
                      "<pos> ...]\n"
                      "chain <speed> <pos>";
    m_sCmdHelpDesc  = "Move the specified servos to the goal positions at the "
                      "given speeds. "
                      "If more than one servo is specifed then a synchronous "
                      "move will be automatically performed. "
                      "If the keyword 'chain' is specified, then all servos in "
                      "the chain are synchronously moved.\n"
                      "  <servo_id>  Servo id [0-253].\n"
                      "  <speed>     Servo goal speed.\n"
                      "  <pos>       Odometer goal position.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdMoveAtSpeedTo() { }

  /*!
   * \brief Execute 'write-like' command on servos.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    uint_t                uNumPairs;
    DynaSpeedPosTuple_T   tup[DYNA_ID_NUMOF];
    int                   nServoId;
    int                   nSpeed;
    int                   nPos;
    int                   iter;
    int                   cnt;
  
    TRY( ChkArgCnt(shell, argc) );
  
    if( (argc % 3) != 0 )
    {
      shell.Error("Unmatched servo_id,speed,pos triplets.");
      return;
    }
  
    uNumPairs = (uint_t)(argc / 3);
  
    if( uNumPairs > DYNA_ID_NUMOF )
    {
      shell.Error("%u: Too many servo_id,speed,pos triplets.", uNumPairs);
      return;
    }
  
    TRY( ChkComm(shell) );
    TRY( ChkChainNotEmpty(shell) );
  
    // full chain of servos
    if( !strcmp(argv[0], "chain") )
    {
      TRY( ChkArgCntEQ(shell, argc, 3) );
      TRY( ToInt(shell, argv[1], &nSpeed) );
      TRY( ToInt(shell, argv[2], &nPos) );
  
      for(nServoId = shell.m_pDynaChain->IterStartMaster(&iter), cnt=0;
          nServoId != DYNA_ID_NONE;
          nServoId = shell.m_pDynaChain->IterNextMaster(&iter), ++cnt)
      {
        tup[cnt].m_nServoId = nServoId;
        tup[cnt].m_nSpeed   = nSpeed;
        tup[cnt].m_nPos     = nPos;
      }

      uNumPairs = (uint_t)cnt;
    }
  
    // list of servos
    else
    {
      for(iter=0, cnt=0; iter<argc; iter+=3, ++cnt)
      {
        TRY( ToInt(shell, argv[iter], &nServoId) );
        TRY( ToInt(shell, argv[iter+1], &nSpeed) );
        TRY( ToInt(shell, argv[iter+2], &nPos) );
        TRY( ChkChainHasServo(shell, nServoId) );
        TRY( ChkChainIsMasterServo(shell, nServoId) );
  
        tup[cnt].m_nServoId = nServoId;
        tup[cnt].m_nSpeed   = nSpeed;
        tup[cnt].m_nPos     = nPos;
      } 
    }
  
    doExec(shell, tup, uNumPairs);
  }

protected:
  /*!
   * \brief Move servos to goal positions.
   *
   * \param shell   Dynamixel shell.
   * \param tup     Array of (servo id, goal position) 2-tuples.
   * \param uCount  Number of tuples.
   */
  virtual void doExec(DynaShell            &shell,
                      DynaSpeedPosTuple_T   tup[],
                      uint_t                uCount)
  {
    int             rc;

    if( uCount == 1 )
    {
      rc = shell.m_pDynaChain->MoveAtSpeedTo(tup[0].m_nServoId,
                                             tup[0].m_nSpeed,
                                             tup[0].m_nPos);

      if( rc != DYNA_OK )
      {
        shell.Error(rc, "Servo %d: move at speed %d to %d failed.",
                          tup[0].m_nServoId, tup[0].m_nSpeed, tup[0].m_nPos);
      }
    }
    else // > 1
    {
      rc = shell.m_pDynaChain->SyncMoveAtSpeedTo(tup, uCount);

      if( rc != DYNA_OK )
      {
        shell.Error(rc, "Synchronous move at speed of %u servos failed.",
            uCount);
      }
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdEStop Class
// -----------------------------------------------------------------------------

/*!
 * \brief Emergency stop command.
 */
class DynaShellCmdEStop : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdEStop() : DynaShellCmd(0, 0)
  {
    m_sCmdName      = "estop";
    m_sCmdHelpBrief = "Emergency stop all servos.";
    m_sCmdHelpArgs  = "";
    m_sCmdHelpDesc  = "";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdEStop() { }

  /*!
   * \brief Emergence stop all servos.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    int     rc;

    TRY( ChkComm(shell) );
    TRY( ChkChain(shell) );

    rc = shell.m_pDynaChain->EStop();

    if( rc == DYNA_OK )
    {
      shell.Ok();
    }
    else
    {
      shell.Error(rc, "Emergency stop failed.");
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdStop Class
// -----------------------------------------------------------------------------

/*!
 * \brief Stop servo(s) command.
 */
class DynaShellCmdStop : public DynaShellCmdChainIn
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdStop() : DynaShellCmdChainIn(true)
  {
    m_sCmdName      = "stop";
    m_sCmdHelpBrief = "Stop servo movement.";
    m_sCmdHelpArgs  = "<servo_id [<servo_id ...]\nchain";
    m_sCmdHelpDesc  = "Stop the movement of the specified servos. " 
                      "If the keyword 'chain' is specified, then all servos "
                      "in the chain are stopped.\n"
                      "  <servo_id>  Servo id [0-253].";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdStop() { }

protected:
  virtual void doExec(DynaShell &shell, DynaServo *pServo)
  {
    pServo->Stop();
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdFreeze Class
// -----------------------------------------------------------------------------

/*!
 * \brief Freeze command.
 */
class DynaShellCmdFreeze : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdFreeze() : DynaShellCmd(0, 0)
  {
    m_sCmdName      = "freeze";
    m_sCmdHelpBrief = "Freeze all servos.";
    m_sCmdHelpArgs  = "";
    m_sCmdHelpDesc  = "Freeze all servos at the current positions. Torque is "
                      "applied to keep the positions.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdFreeze() { }

  /*!
   * \brief Freeze servos.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    int     rc;

    TRY( ChkArgCnt(shell, argc) );
    TRY( ChkComm(shell) );
    TRY( ChkChainNotEmpty(shell) );

    rc = shell.m_pDynaChain->Freeze();

    if( rc == DYNA_OK )
    {
      shell.Ok();
    }
    else
    {
      shell.Error(rc, "Freeze failed.");
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdRelease Class
// -----------------------------------------------------------------------------

/*!
 * \brief Release command.
 */
class DynaShellCmdRelease : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdRelease() : DynaShellCmd(0, 0)
  {
    m_sCmdName      = "release";
    m_sCmdHelpBrief = "Release all servos.";
    m_sCmdHelpArgs  = "";
    m_sCmdHelpDesc  = "Release all servos. No torque is applied.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdRelease() { }

  /*!
   * \brief Release servos.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    int     rc;

    TRY( ChkArgCnt(shell, argc) );
    TRY( ChkComm(shell) );
    TRY( ChkChainNotEmpty(shell) );

    rc = shell.m_pDynaChain->Release();

    if( rc == DYNA_OK )
    {
      shell.Ok();
    }
    else
    {
      shell.Error(rc, "Release failed.");
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdGetOdometer Class
// -----------------------------------------------------------------------------

/*!
 * \brief Read servos goal positions.
 */
class DynaShellCmdGetOdometer : public DynaShellCmdChainIn
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdGetOdometer()
  {
    m_sCmdName      = "odometer";
    m_sCmdHelpBrief = "Get servo odometer parameters.";
    m_sCmdHelpArgs  = "<servo_id> [servo_id...]\nchain";
    m_sCmdHelpDesc  = "Get the servo odometer zero point, modulo, and reverse "
                      "parameters.  If the keyword 'chain' is specified, then "
                      "the odometer settings for all of the servos in the "
                      "chain are read.\n"
                      "  <servo_id>  Servo id [0-253].";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdGetOdometer() { }

protected:

  /*!
   * \brief Determine torque enable state for servo.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo.
   */
  virtual void doExec(DynaShell &shell, DynaServo *pServo)
  {
    int   nOdometer;
    int   nOdZeroPt;
    bool  bOdReversed;

    nOdometer   = pServo->GetOdometer();
    nOdZeroPt   = pServo->GetOdometerZeroPt();
    bOdReversed = pServo->IsOdometerReversed();

    shell.Response("Servo %3d: odometer %5d, zeropt %4d, reverse: %s\n",
                   pServo->GetServoId(), nOdometer, nOdZeroPt,
                  (bOdReversed? "true": "false"));
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdSetOdometer Class
// -----------------------------------------------------------------------------

/*!
 * \brief Determine if servos are moving.
 */
class DynaShellCmdSetOdometer : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdSetOdometer() : DynaShellCmd(3, 3)
  {
    m_sCmdName      = "odometer";
    m_sCmdHelpBrief = "Set odometer parameters.";
    m_sCmdHelpArgs  = "<servo_id> <zeropt> <modulo> <reverse>";
    m_sCmdHelpDesc  = "Set servo virtual odometer parameters.\n"
                      "  <servo_id>  Servo id [0-253].\n"
                      "  <zeropt>    Zero point in encoder units.\n"
                      "  <reverse>   Odometer is [not] reversed. One of: "
                      "t true 1 f false 0";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdSetOdometer() { }

  /*!
   * \brief Execute 'write-like' command on servos.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    int         nServoId;
    int         nOdZeroPt;
    bool        bOdReversed;
    DynaServo  *pServo;
  
    TRY( ChkArgCnt(shell, argc) );
  
    TRY( ChkComm(shell) );
    TRY( ChkChainNotEmpty(shell) );
  
    TRY( ToInt(shell,  argv[0], &nServoId) );
    TRY( ToInt(shell,  argv[1], &nOdZeroPt) );
    TRY( ToBool(shell, argv[2], &bOdReversed) );
    TRY( ChkChainHasServo(shell, nServoId) );
    TRY( ChkChainIsMasterServo(shell, nServoId) );
  
    pServo = shell.m_pDynaChain->GetServo(nServoId);

    if( pServo == NULL )
    {
      shell.Error("Servo %d: Servo not found.\n");
    }
    else
    {
      pServo->ResetOdometer(nOdZeroPt, bOdReversed);
      shell.Ok();
    }
  }

protected:
};


// -----------------------------------------------------------------------------
// DynaShellCmdCfgWriteServoMode Class
// -----------------------------------------------------------------------------

/*!
 * \brief Write EEPROM configuration to put servos into the given  modes.
 *
 * \par Syntax:
 * \verbatim
 * arg_to_uint ::= mode
 * mode ::= 'c' | 'cont' | 'continuous' | 's' | 'servo'
 * \endverbatim
 */
class DynaShellCmdCfgWriteServoMode : public DynaShellCmdChainOut
{
public:
  /*!
   * \brief Default constructor.
   */
  DynaShellCmdCfgWriteServoMode() : DynaShellCmdChainOut(true)
  {
    m_sCmdName      = "mode";
    m_sCmdHelpBrief = "Configure servo operational modes.";
    m_sCmdHelpArgs  = "<servo_id> <mode> [<servo_id> <mode> ...]\n"
                      "chain <mode>";
    m_sCmdHelpDesc  = "Configure the operational modes of the specified "
                      "servos. The servo configuration is written to the servo "
                      "EEPROM. If the keyword 'chain' is specified, then all "
                      "servos in the chain are configured.\n"
                      "  <servo_id>  Servo id [0-253].\n"
                      "  <mode>      Servo mode. One of: continuous servo.";
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdCfgWriteServoMode() { }

protected:
  /*!
   * \brief Execute operation.
   *
   * \param shell   Dynamixel shell.
   * \param tup     Array of (servo id, mode) 2-tuples.
   * \param uCount  Number of tuples.
   */
  virtual void doExec(DynaShell &shell, ExecTup_T tup[], uint_t uCount)
  {
    DynaServo            *pServo;
    DynaServoSpec_T       spec;
    uint_t                i;
    int                   rc;

    for(i=0, rc=DYNA_OK; i<uCount && rc==DYNA_OK; ++i)
    {
      pServo = shell.m_pDynaChain->GetServo(tup[i].m_nServoId);

      switch( tup[i].m_nVal )
      {
        case DYNA_MODE_CONTINUOUS:
          rc = pServo->CfgWriteServoModeContinuous();
          break;

        case DYNA_MODE_SERVO:
        default:
          spec = pServo->GetSpecification();
          rc = pServo->CfgWriteServoMode(spec.m_uRawPosMin, spec.m_uRawPosMax);
          break;
      }
    }

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Servo %d: Failed to set mode to %d.",
          pServo->GetServoId(), tup[i].m_nVal);
    }
  }

  /*!
   * \brief Convert command argument into servo mode.
   *
   * \param shell       Dynamixel shell.
   * \param sArg        String argument to convert.
   * \param [out] pVal  Converted argument.
   *
   * \return Returns true on success, false otherwise.
   */
  virtual bool ArgToInt(DynaShell &shell, const char *sArg, int *pVal)
  {
    size_t  n = strlen(sArg);

    if( !strncmp("continuous", sArg, n) )
    {
      *pVal = DYNA_MODE_CONTINUOUS;
      return true;
    }
    else if( !strncmp("servo", sArg, n) )
    {
      *pVal = DYNA_MODE_SERVO;
      return true;
    }
    else
    {
      shell.Error("Argument value \"%s\": Not a servo mode.", sArg);
      return false;
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdReadTorqueEnable Class
// -----------------------------------------------------------------------------

/*!
 * \brief Read servos goal positions.
 */
class DynaShellCmdReadTorqueEnable : public DynaShellCmdChainIn
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdReadTorqueEnable()
  {
    m_sCmdName      = "torqueenable";
    m_sCmdHelpBrief = "Read servo torque enable states.";
    m_sCmdHelpArgs  = "<servo_id> [servo_id...]\nchain";
    m_sCmdHelpDesc  = "Read the servo torque enable states for the specified "
                      "servos.  If the keyword 'chain' is specified, then the "
                      "torque enable states for all of the servos in the "
                      "chain are read.\n"
                      "  <servo_id>  Servo id [0-253].";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdReadTorqueEnable() { }

protected:

  /*!
   * \brief Determine torque enable state for servo.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo.
   */
  virtual void doExec(DynaShell &shell, DynaServo *pServo)
  {
    bool        bTorqueEnable;
    int         rc;

    rc = pServo->ReadTorqueEnable(&bTorqueEnable);

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Servo %d: Read torque enable failed.",
          pServo->GetServoId());
      return;
    }

    shell.Response("Servo %3d: %s\n", pServo->GetServoId(),
            (bTorqueEnable? "enabled": "disabled"));
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdWriteTorqueEnable Class
// -----------------------------------------------------------------------------

/*!
 * \brief Determine if servos are moving.
 */
class DynaShellCmdWriteTorqueEnable : public DynaShellCmdChainOut
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdWriteTorqueEnable() : DynaShellCmdChainOut(true)
  {
    m_sCmdName      = "torqueenable";
    m_sCmdHelpBrief = "Enable/disable servo torques.";
    m_sCmdHelpArgs  = "<servo_id> <switch> [<servo_id> <switch> ...]\n"
                      "chain <switch>";
    m_sCmdHelpDesc  = "Enable or disable servo torques for the specified "
                      "servos. If the keyword 'chain' is specified, then all "
                      "servos in the chain are set.\n";
                      "  <servo_id>  Servo id [0-253].\n"
                      "  <switch>:   One of: enable e on true 1"
                      "disable d off false 0";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdWriteTorqueEnable() { }

protected:
  /*!
   * \brief Enable/disable servos torques.
   *
   * \param shell   Dynamixel shell.
   * \param tup     Array of (servo id, goal position) 2-tuples.
   * \param uCount  Number of tuples.
   */
  virtual void doExec(DynaShell &shell, ExecTup_T tup[], uint_t uCount)
  {
    uint_t      i;
    DynaServo  *pServo;
    bool        state;
    int         rc;

    for(i=0, rc=DYNA_OK; i<uCount && i==DYNA_OK; ++i)
    {
      pServo = shell.m_pDynaChain->GetServo(tup[i].m_nServoId);
      state = tup[i].m_nVal? true: false;

      rc = pServo->WriteTorqueEnable(state);

      if( rc != DYNA_OK )
      {
        shell.Error(rc,
            "Servo %d: Failed to set torque enable state.",
            pServo->GetServoId());
      }
    }
  }

  /*!
   * \brief Convert command argument into enable state.
   *
   * \param shell       Dynamixel shell.
   * \param sArg        String argument to convert.
   * \param [out] pVal  Converted argument.
   *
   * \return Returns true on success, false otherwise.
   */
  virtual bool ArgToInt(DynaShell &shell, const char *sArg, int *pVal)
  {
    if( !strcmp(sArg, "e")    || !strcmp(sArg, "enable")  ||
        !strcmp(sArg, "on")   || !strcmp(sArg, "true")    ||
        !strcmp(sArg, "1") )
    {
      *pVal = 1;
      return true;
    }
    else if( !strcmp(sArg, "d")    || !strcmp(sArg, "disable")  ||
             !strcmp(sArg, "off")  || !strcmp(sArg, "false")    ||
             !strcmp(sArg, "0") )
    {
      *pVal = 0;
      return true;
    }
    else
    {
      shell.Error("Argument value \"%s\": Invalid state.", sArg);
      return false;
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdReadGoalPos Class
// -----------------------------------------------------------------------------

/*!
 * \brief Read servos goal positions.
 */
class DynaShellCmdReadGoalPos : public DynaShellCmdChainIn
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdReadGoalPos()
  {
    m_sCmdName      = "goalpos";
    m_sCmdHelpBrief = "Read servo goal positions.";
    m_sCmdHelpArgs  = "<servo_id> [servo_id...]\nchain";
    m_sCmdHelpDesc  = "Read servo goal positions for all of the specified "
                      "servos. If the keyword 'chain' is specified, then the "
                      "goal positions for all of the servos in the chain are "
                      "read.\n"
                      "  <servo_id>  Servo id [0-253].";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdReadGoalPos() { }

protected:

  /*!
   * \brief Read goal position for servo.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo.
   */
  virtual void doExec(DynaShell &shell, DynaServo *pServo)
  {
    int     nGoalPos;
    int     rc;

    rc = pServo->ReadGoalPos(&nGoalPos);

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Servo %d: Read goal position failed.",
          pServo->GetServoId());
      return;
    }

    switch( pServo->GetServoMode() )
    {
      case DYNA_MODE_CONTINUOUS:
        shell.Response("Servo %3d: %d (continuous mode)\n",
            pServo->GetServoId(), nGoalPos);
        break;
      case DYNA_MODE_SERVO:
      default:
        shell.Response("Servo %3d: %d\n", pServo->GetServoId(), nGoalPos);
        break;
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdReadGoalSpeed Class
// -----------------------------------------------------------------------------

/*!
 * \brief Read servos goal positions.
 */
class DynaShellCmdReadGoalSpeed : public DynaShellCmdChainIn
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdReadGoalSpeed()
  {
    m_sCmdName      = "goalspeed";
    m_sCmdHelpBrief = "Read servo goal speeds.";
    m_sCmdHelpArgs  = "<servo_id> [servo_id...]\nchain";
    m_sCmdHelpDesc  = "Read the goal rotational speeds for the specified "
                      "servos. Speeds <0 are clockwise rotations, >0 are "
                      "conterclockwise. Directions are N/A for servos in "
                      "servo mode. If the keyword 'chain' is specified, then "
                      "the goal speeds for all of the servos in the chain are "
                      "read.\n"
                      "  <servo_id>  Servo id [0-253].";
  }
  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdReadGoalSpeed() { }


protected:

  /*!
   * \brief Read goal position for servo.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo.
   */
  virtual void doExec(DynaShell &shell, DynaServo *pServo)
  {
    int   nGoalSpeed;
    int   nSpeed;
    int   rc;

    rc = pServo->ReadGoalSpeed(&nGoalSpeed);

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Servo %d: Read goal speed failed.",
          pServo->GetServoId());
      return;
    }

    if( pServo->GetServoMode() == DYNA_MODE_CONTINUOUS )
    {
      shell.Response("Servo %3d: %d %s\n",
                  pServo->GetServoId(), nGoalSpeed, rotdirstr(nGoalSpeed));
    }
    else
    {
      shell.Response("Servo %3d: %d\n", pServo->GetServoId(), nGoalSpeed);
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdWriteGoalSpeed Class
// -----------------------------------------------------------------------------

/*!
 * \brief Determine if servos are moving.
 */
class DynaShellCmdWriteGoalSpeed : public DynaShellCmdChainOut
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdWriteGoalSpeed() : DynaShellCmdChainOut(true)
  {
    m_sCmdName      = "goalspeed";
    m_sCmdHelpBrief = "Write the servo goal speeds.";
    m_sCmdHelpArgs  = "<servo_id> <speed> [<servo_id> <speed> ...]\n"
                      "chain <speed>";
    m_sCmdHelpDesc  = "Write the goal speeds for the specified servos. "
                      "Speeds <0 are clockwise rotations, >0 are "
                      "conterclockwise. Directions are N/A for servos in "
                      "servo mode. If more than one servo is specifed then a "
                      "synchronous write will be automatically performed. "
                      "If the keyword 'chain' is specified, all servos in the "
                      "chain are synchronously written.\n"
                      "  <servo_id>  Servo id [0-253].\n"
                      "  <speed>     Servo goal speed.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdWriteGoalSpeed() { }

protected:
  /*!
   * \brief Write goal speeds.
   *
   * \param shell   Dynamixel shell.
   * \param tup     Array of (servo id, goal speed) 2-tuples.
   * \param uCount  Number of tuples.
   */
  virtual void doExec(DynaShell &shell, ExecTup_T tup[], uint_t uCount)
  {
    DynaServo        *pServo;
    int               nGoalSpeed;
    DynaSpeedTuple_T  tupSpeed[DYNA_ID_NUMOF];
    int               i;
    int               rc;

    if( uCount == 1 )
    {
      pServo = shell.m_pDynaChain->GetServo(tup[0].m_nServoId);

      nGoalSpeed = tup[0].m_nVal;

      rc = pServo->WriteGoalSpeed(nGoalSpeed);

      if( rc != DYNA_OK )
      {
        shell.Error(rc,
            "Servo %d: Write goal speed %d failed.",
            pServo->GetServoId(), nGoalSpeed);
      }
    }
    else // > 1
    {
      for(i=0; i<(int)uCount; ++i)
      {
        pServo = shell.m_pDynaChain->GetServo(tup[i].m_nServoId);

        tupSpeed[i].m_nServoId = tup[i].m_nServoId;
        tupSpeed[i].m_nSpeed   = tup[i].m_nVal;
      }

      rc = shell.m_pDynaChain->SyncWriteGoalSpeed(tupSpeed, uCount);

      if( rc != DYNA_OK )
      {
        shell.Error(rc,
            "Synchronous write of goal speeds for %u servos failed.",
            uCount);
      }
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdReadMaxTorqueLimit Class
// -----------------------------------------------------------------------------

/*!
 * \brief Read servos on power-up or run-time maximum torque limits.
 */
class DynaShellCmdReadMaxTorqueLimit : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdReadMaxTorqueLimit() : DynaShellCmd(2, 1+DYNA_ID_NUMOF)
  {
    m_sCmdName      = "maxtorque";
    m_sCmdHelpBrief = "Read maximum torque limits.";
    m_sCmdHelpArgs  = "<which> <servo_id> [<servo_id> ...]\n<which> chain";
    m_sCmdHelpDesc  = "Read either the on power-up maximum torque limits "
                      "or the run-time limits for the specified "
                      "servos. The power-up limits are read from servo EEPROM "
                      "configuration. The run-time limits are read from servo "
                      "state RAM."
                      "If the keyword 'chain' is specified, then the "
                      "torque limits for all of the servos in the chain are "
                      "read.\n"
                      "  <which>     Which maximum torque limits. One of: "
                      "powerup runtime.\n"   
                      "  <servo_id>  Servo id [0-253].";
  }
  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdReadMaxTorqueLimit() { }

  /*!
   * \brief Execute command on servos.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  virtual void Exec(DynaShell &shell, int argc, char *argv[])
  {
    uint_t  uAddr;
    int     nServoId;
    int     iter;

    TRY( ChkArgCnt(shell, argc) );
    TRY( ChkComm(shell) );
    TRY( ChkChainNotEmpty(shell) );

    if( !strcmp(argv[0], "powerup") )
    {
      uAddr = DYNA_ADDR_LIM_TORQUE_MAX_ON_LSB;
    }
    else if( !strcmp(argv[0], "runtime") )
    {
      uAddr = DYNA_ADDR_LIM_TORQUE_MAX_LSB;
    }
    else
    {
      shell.Error("%s: Unknown <which> value.", argv[0]);
      return;
    }

    if( !strcmp(argv[1], "chain") )
    {
      TRY( ChkArgCntEQ(shell, argc, 2) );

      for(nServoId = shell.m_pDynaChain->IterStart(&iter);
          nServoId != DYNA_ID_NONE;
          nServoId = shell.m_pDynaChain->IterNext(&iter))
      {
        doExec(shell, nServoId, uAddr);
      }
    }

    else
    {
      for(iter=1; iter<argc; ++iter)
      {
        TRY( ToInt(shell, argv[iter], &nServoId) );
        TRY( ChkChainHasServo(shell, nServoId) );

        doExec(shell, nServoId, uAddr);
      }
    }
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes NULL <which> <servo_id> | chain [servo_id [servo_id ...]]
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
    int   i;
  
    if( (shell.m_pDynaChain == NULL) ||
        (shell.m_pDynaChain->GetNumberInChain() == 0) )
    {
      return NULL;
    }
  
    // argument number of already (expanded) arguments
    nArgNum = ReadLine::wc(sContext) - ReadLine::wc(m_sPubName);

    // <which> 
    if( nArgNum == 0 )
    {
      if( nState == ReadLine::FIRST )
      {
        m_nTabIter = 0;
      }

      while( m_nTabIter < 2 )
      {
        i = m_nTabIter++;

        switch( i )
        {
          case 0:
            if( !strncmp("powerup", sText, uTextLen) )
            {
              return ReadLine::dupstr("powerup");
            }
            break;
          case 1:
            if( !strncmp("runtime", sText, uTextLen) )
            {
              return ReadLine::dupstr("runtime");
            }
            break;
        }
      }
    }

    //
    // Tab complete servo list 
    //
    else
    {
      if( nState == ReadLine::FIRST )
      {
        m_nTabServoId = shell.m_pDynaChain->IterStart(&m_nTabIter);
  
        if( nArgNum == 1 )
        {
          m_bTabIncChain = true;
        }
        else
        {
          m_bTabIncChain = false;
        }
      }
  
      while( m_nTabServoId != DYNA_ID_NONE )
      {
        snprintf(buf, sizeof(buf), "%d", m_nTabServoId);
        buf[sizeof(buf)-1] = 0;
  
        m_nTabServoId = shell.m_pDynaChain->IterNext(&m_nTabIter);
  
        if( !strncmp(buf, sText, uTextLen) )
        {
          return ReadLine::dupstr(buf);
        }
      }
  
      if( m_bTabIncChain )
      {
        m_bTabIncChain = false;
        if( !strncmp("chain", sText, uTextLen) )
        {
          return ReadLine::dupstr("chain");
        }
      }
    }
  
    // no more matches
    return NULL;
  }

protected:
  int  m_nTabIter;        ///< tab completion: iterator
  int  m_nTabServoId;     ///< tab completion: current servo id
  bool m_bTabIncChain;    ///< tab completion: [do not] include chain keyword

  /*!
   * \brief Read goal position for servo.
   *
   * \param shell     Dynamixel shell.
   * \param nServoId  Servo id.
   * \param uAddr     Servo read address.
   */
  virtual void doExec(DynaShell &shell, int nServoId, uint_t uAddr)
  {
    uint_t      uMaxTorqueLimit;
    int         rc;

    rc = shell.m_pDynaComm->Read16(nServoId, uAddr, &uMaxTorqueLimit);

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Servo %d: Read maximum torque limit failed.", nServoId);
      return;
    }

    shell.Response("Servo %3d: %d\n", nServoId, uMaxTorqueLimit);
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdWriteMaxTorqueLimit Class
// -----------------------------------------------------------------------------

/*!
 * \brief Write servos on power-up or run-time maximum torque limits.
 */
class DynaShellCmdWriteMaxTorqueLimit : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdWriteMaxTorqueLimit() : DynaShellCmd(3, 2+DYNA_ID_NUMOF*2)
  {
    m_sCmdName      = "maxtorque";
    m_sCmdHelpBrief = "Write maximum torque limits.";
    m_sCmdHelpArgs  = "<which> <servo_id> <limit> [<servo_id> <limit> ...]\n"
                      "<which> chain <limit>";
    m_sCmdHelpDesc  = "Write either the on power-up maximum torque limits "
                      "or the run-time limits for the specified "
                      "servos. The power-up limits are written to servo EEPROM "
                      "configuration. The run-time limits are written to servo "
                      "state RAM."
                      "If the keyword 'chain' is specified, then the "
                      "torque limits for all of the servos in the chain are "
                      "written.\n"
                      "  <which>     Which maximum torque limits. One of: "
                      "powerup runtime.\n"   
                      "  <servo_id>  Servo id [0-253].\n"
                      "  <limit>     Servo torque limit.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdWriteMaxTorqueLimit() { }

  /*!
   * \brief Execute command on servos.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    uint_t      uAddr;
    uint_t      uNumPairs;
    ExecTup_T   tup[DYNA_ID_NUMOF];
    int         nServoId;
    int         nVal;
    int         iter;
    int         cnt;
  
    TRY( ChkArgCnt(shell, argc) );
  
    if( ((argc-1) % 2) != 0 )
    {
      shell.Error("Unmatched servo_id,value pairs.");
      return;
    }
  
    if( !strcmp(argv[0], "powerup") )
    {
      uAddr = DYNA_ADDR_LIM_TORQUE_MAX_ON_LSB;
    }
    else if( !strcmp(argv[0], "runtime") )
    {
      uAddr = DYNA_ADDR_LIM_TORQUE_MAX_LSB;
    }
    else
    {
      shell.Error("%s: Unknown <which> value.", argv[0]);
      return;
    }

    uNumPairs = (uint_t)((argc-1) / 2);
  
    if( uNumPairs > DYNA_ID_NUMOF )
    {
      shell.Error("%u: Too many servo_id,value pairs.", uNumPairs);
      return;
    }
  
    TRY( ChkComm(shell) );
    TRY( ChkChainNotEmpty(shell) );
  
    // full chain of servos
    if( !strcmp(argv[1], "chain") )
    {
      TRY( ChkArgCntEQ(shell, argc, 3) );
      TRY( ToInt(shell, argv[2], &nVal) );
  
      for(nServoId = shell.m_pDynaChain->IterStart(&iter), cnt=0;
          nServoId != DYNA_ID_NONE;
          nServoId = shell.m_pDynaChain->IterNext(&iter), ++cnt)
      {
        tup[cnt].m_nServoId = nServoId;
        tup[cnt].m_nVal     = nVal;
      }
  
      uNumPairs = (uint_t)cnt;
    }
  
    // list of servos
    else
    {
      for(iter=1, cnt=0; iter<argc-1; iter+=2, ++cnt)
      {
        TRY( ToInt(shell, argv[iter], &nServoId) );
        TRY( ToInt(shell, argv[iter+1], &nVal) );
        TRY( ChkChainHasServo(shell, nServoId) );
  
        tup[cnt].m_nServoId = nServoId;
        tup[cnt].m_nVal     = nVal;
      } 
    }
  
    doExec(shell, uAddr, tup, uNumPairs);
  }


  /*!
   * \brief Command tab completion generator.
   *
   * Completes NULL <which> <servo_id> | chain [servo_id [servo_id ...]]
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
    int   i;
  
    if( (shell.m_pDynaChain == NULL) ||
        (shell.m_pDynaChain->GetNumberInChain() == 0) )
    {
      return NULL;
    }
  
    // argument number of already (expanded) arguments
    nArgNum = ReadLine::wc(sContext) - ReadLine::wc(m_sPubName);

    // <which> 
    if( nArgNum == 0 )
    {
      if( nState == ReadLine::FIRST )
      {
        m_nTabIter = 0;
      }

      while( m_nTabIter < 2 )
      {
        i = m_nTabIter++;

        switch( i )
        {
          case 0:
            if( !strncmp("powerup", sText, uTextLen) )
            {
              return ReadLine::dupstr("powerup");
            }
            break;
          case 1:
            if( !strncmp("runtime", sText, uTextLen) )
            {
              return ReadLine::dupstr("runtime");
            }
            break;
        }
      }
    }

    // every even argument is the servo value, which cannot be expanded
    else if( !(nArgNum & 0x01) )
    {
      return NULL;
    }

    //
    // Tab complete servo list 
    //
    else
    {
      if( nState == ReadLine::FIRST )
      {
        m_nTabServoId = shell.m_pDynaChain->IterStart(&m_nTabIter);
  
        if( nArgNum == 1 )
        {
          m_bTabIncChain = true;
        }
        else
        {
          m_bTabIncChain = false;
        }
      }
  
      while( m_nTabServoId != DYNA_ID_NONE )
      {
        snprintf(buf, sizeof(buf), "%d", m_nTabServoId);
        buf[sizeof(buf)-1] = 0;
  
        m_nTabServoId = shell.m_pDynaChain->IterNext(&m_nTabIter);
  
        if( !strncmp(buf, sText, uTextLen) )
        {
          return ReadLine::dupstr(buf);
        }
      }
  
      if( m_bTabIncChain )
      {
        m_bTabIncChain = false;
        if( !strncmp("chain", sText, uTextLen) )
        {
          return ReadLine::dupstr("chain");
        }
      }
    }
  
    // no more matches
    return NULL;
  }

protected:
  int  m_nTabIter;        ///< tab completion: iterator
  int  m_nTabServoId;     ///< tab completion: current servo id
  bool m_bTabIncChain;    ///< tab completion: [do not] include chain keyword

  /*!
   * \brief Move servos to goal positions.
   *
   * \param shell   Dynamixel shell.
   * \param uAddr   Torque limit address.
   * \param tup     Array of (servo id, goal position) 2-tuples.
   * \param uCount  Number of tuples.
   */
  virtual void doExec(DynaShell  &shell,
                      uint_t      uAddr,
                      ExecTup_T   tup[],
                      uint_t      uCount)
  {
    int         nServoId;
    uint_t      uVal;
    uint_t      i;
    int         rc;

    for(i=0, rc=DYNA_OK; i<uCount && rc==DYNA_OK; ++i)
    {
      nServoId = tup[i].m_nServoId;
      uVal     = (uint_t)tup[i].m_nVal;

      rc = shell.m_pDynaComm->Write16(nServoId, uAddr, uVal);

      if( rc != DYNA_OK )
      {
        shell.Error(rc,
            "Servo %d: Write maximum torque limit at 0x%02x with %u failed.",
            nServoId, uAddr, uVal);
      }
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdGetSoftTorqueTh Class
// -----------------------------------------------------------------------------

/*!
 * \brief Read servos goal positions.
 */
class DynaShellCmdGetSoftTorqueTh : public DynaShellCmdChainIn
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdGetSoftTorqueTh()
  {
    m_sCmdName      = "torqueth";
    m_sCmdHelpBrief = "Get servo soft torque thresholds.";
    m_sCmdHelpArgs  = "<servo_id> [servo_id...]\nchain";
    m_sCmdHelpDesc  = "Get servo soft torque hysteresis thresholds. When a "
                      "servo exceeds the over torque limit "
                      "threshold, it is placed in the over torque limit "
                      "condition which limits servo actuation. "
                      "When a servo drops below the clear threshold, the "
                      "servo torque over limit condition is cleared. "
                      "If the keyword 'chain' is specified, then "
                      "the thresholds for all of the servos in the "
                      "chain are retrieved.\n"
                      "  <servo_id>  Servo id [0-253].";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdGetSoftTorqueTh() { }

protected:

  /*!
   * \brief Determine torque enable state for servo.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo.
   */
  virtual void doExec(DynaShell &shell, DynaServo *pServo)
  {
    uint_t  uOverTorqueTh;
    uint_t  uClearTorqueTh;

    pServo->GetSoftTorqueThresholds(uOverTorqueTh, uClearTorqueTh);

    shell.Response("Servo %3d: over_th %5d, "
                   "clear_th %5d\n",
                   pServo->GetServoId(), uOverTorqueTh, uClearTorqueTh);
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdSetSoftTorqueTh Class
// -----------------------------------------------------------------------------

/*!
 * \brief Determine if servos are moving.
 */
class DynaShellCmdSetSoftTorqueTh : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdSetSoftTorqueTh() : DynaShellCmd(3, 3)
  {
    m_sCmdName      = "torqueth";
    m_sCmdHelpBrief = "Set servo soft torque thresholds.";
    m_sCmdHelpArgs  = "<servo_id> <over_th> <clear_th>";
    m_sCmdHelpDesc  = "Set servo soft torque hysteresis thresholds.\n"
                      "  <servo_id>  Servo id [0-253].\n"
                      "  <over_th>   Threshold to set the torque over "
                      "limit condition.\n"
                      "  <clear_th>  Theshold to clear the torque over "
                      "limit condition. The clear\n"
                      "              threshold must be lower "
                      "than the over threshold.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdSetSoftTorqueTh() { }

  /*!
   * \brief Execute command on servos.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    int         nServoId;
    uint_t      uOverTorqueTh;
    uint_t      uClearTorqueTh;
    bool        bOdReversed;
    DynaServo  *pServo;
  
    TRY( ChkArgCnt(shell, argc) );
  
    TRY( ChkComm(shell) );
    TRY( ChkChainNotEmpty(shell) );
  
    TRY( ToInt(shell,  argv[0], &nServoId) );
    TRY( ToUInt(shell, argv[1], &uOverTorqueTh) );
    TRY( ToUInt(shell, argv[2], &uClearTorqueTh) );
    TRY( ChkChainHasServo(shell, nServoId) );
    TRY( ChkChainIsMasterServo(shell, nServoId) );
  
    pServo = shell.m_pDynaChain->GetServo(nServoId);

    if( pServo == NULL )
    {
      shell.Error("Servo %d: Servo not found.\n");
    }
    else
    {
      pServo->SetSoftTorqueThresholds(uOverTorqueTh, uClearTorqueTh);
      shell.Ok();
    }
  }

protected:
};


// -----------------------------------------------------------------------------
// DynaShellCmdReadCurPos Class
// -----------------------------------------------------------------------------

/*!
 * \brief Read servos current positions.
 */
class DynaShellCmdReadCurPos : public DynaShellCmdChainIn
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdReadCurPos()
  {
    m_sCmdName      = "curpos";
    m_sCmdHelpBrief = "Read current odometer positions.";
    m_sCmdHelpArgs  = "<servo_id> [servo_id...]\nchain";
    m_sCmdHelpDesc  = "Read the current odometer positions for the "
                      "specified servos. If the keyword 'chain' is specified, "
                      "then the current positions for all of the servos in "
                      "the chain are read.\n"
                      "  <servo_id>  Servo id [0-253].";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdReadCurPos() { }

protected:

  /*!
   * \brief Read current position for servo.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo.
   */
  virtual void doExec(DynaShell &shell, DynaServo *pServo)
  {
    int     nCurPos;
    int     rc;

    rc = pServo->ReadCurPos(&nCurPos);

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Servo %d: Read current position failed.",
          pServo->GetServoId());
      return;
    }

    switch( pServo->GetServoMode() )
    {
      case DYNA_MODE_CONTINUOUS:
        shell.Response("Servo %3d: %d (continuous mode)\n",
            pServo->GetServoId(), nCurPos);
        break;
      case DYNA_MODE_SERVO:
      default:
        shell.Response("Servo %3d: %d\n", pServo->GetServoId(), nCurPos);
        break;
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdReadCurSpeed Class
// -----------------------------------------------------------------------------

/*!
 * \brief Read servos current positions.
 */
class DynaShellCmdReadCurSpeed : public DynaShellCmdChainIn
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdReadCurSpeed()
  {
    m_sCmdName      = "curspeed";
    m_sCmdHelpBrief = "Read the current speeds.";
    m_sCmdHelpArgs  = "<servo_id> [servo_id...]\nchain";
    m_sCmdHelpDesc  = "Read the current speeds for the specified servos. "
                      "Speeds <0 are clockwise rotations, >0 are "
                      "conterclockwise. If the keyword 'chain' is specified, "
                      "then the current speeds for all of the servos in the "
                      "chain are read.\n"
                      "  <servo_id>  Servo id [0-253].";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdReadCurSpeed() { }

protected:

  /*!
   * \brief Read current position for servo.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo.
   */
  virtual void doExec(DynaShell &shell, DynaServo *pServo)
  {
    int         nCurSpeed;
    int         rc;

    rc = pServo->ReadCurSpeed(&nCurSpeed);

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Servo %d: Read current speed failed.",
          pServo->GetServoId());
      return;
    }

    shell.Response("Servo %3d: %d %s\n",
            pServo->GetServoId(), nCurSpeed, rotdirstr(nCurSpeed));
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdReadDynamics Class
// -----------------------------------------------------------------------------

/*!
 * \brief Read servos goal positions.
 */
class DynaShellCmdReadDynamics : public DynaShellCmdChainIn
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdReadDynamics()
  {
    m_sCmdName      = "dynamics";
    m_sCmdHelpBrief = "Read the current servo dynamics.";
    m_sCmdHelpArgs  = "<servo_id> [servo_id...]\nchain";
    m_sCmdHelpDesc  = "Read the current dynamics for the specified servos. "
                      "Dynamics are position, speed, and torque. "
                      "If the keyword 'chain' is specified, then the dynamics "
                      "for all of the servos in the chain are read.\n"
                      "  <servo_id>  Servo id [0-253].";
  }
  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdReadDynamics() { }


protected:

  /*!
   * \brief Read dynamics for servo.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo.
   */
  virtual void doExec(DynaShell &shell, DynaServo *pServo)
  {
    int         nCurPos;
    int         nCurSpeed;
    int         nCurLoad;
    int         rc;

    rc = pServo->ReadDynamics(&nCurPos, &nCurSpeed, &nCurLoad);

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Servo %d: Failed to read the dynamics.",
          pServo->GetServoId());
      return;
    }

    shell.Response("Servo %3d: pos %5d, speed %5d, load %5d\n",
        pServo->GetServoId(), nCurPos, nCurSpeed, nCurLoad);
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdReadHealth Class
// -----------------------------------------------------------------------------

/*!
 * \brief Read servos goal positions.
 */
class DynaShellCmdReadHealth : public DynaShellCmdChainIn
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdReadHealth()
  {
    m_sCmdName      = "health";
    m_sCmdHelpBrief = "Read the current servo health.";
    m_sCmdHelpArgs  = "<servo_id> [servo_id...]\nchain";
    m_sCmdHelpDesc  = "Read the current health of the specified servos. "
                      "Servo health is the combined state of servo alarms, "
                      "torque, temperature, and voltage. "
                      "If the keyword 'chain' is specified, then the health "
                      "for all of the servos in the chain are read.\n"
                      "  <servo_id>  Servo id [0-253].";
  }
  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdReadHealth() { }


protected:

  /*!
   * \brief Read dynamics for servo.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo.
   */
  virtual void doExec(DynaShell &shell, DynaServo *pServo)
  {
    uint_t      uAlarms;
    string      strAlarms;
    int         nCurLoad;
    uint_t      uCurVolt;
    uint_t      uCurTemp;
    int         rc;

    rc = pServo->ReadHealth(&uAlarms, &nCurLoad, &uCurVolt, &uCurTemp);

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Servo %d: Failed to read the health.",
          pServo->GetServoId());
      return;
    }

    strAlarms = DynaComm::GetAlarmsString(uAlarms);

    shell.Response("Servo %3d: load %5d, voltage %3u, temperature %3u, "
        "alarms(0x%02x) %s\n",
        pServo->GetServoId(), nCurLoad, uCurVolt, uCurTemp, uAlarms,
        strAlarms.c_str());
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdReadIsMoving Class
// -----------------------------------------------------------------------------

/*!
 * \brief Determine if servos are moving.
 */
class DynaShellCmdReadIsMoving : public DynaShellCmdChainIn
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdReadIsMoving()
  {
    m_sCmdName      = "ismoving";
    m_sCmdHelpBrief = "Check if servos are moving.";
    m_sCmdHelpArgs  = "<servo_id> [servo_id...]\nchain";
    m_sCmdHelpDesc  = "Determine if the specified servos are moving. "
                      "The determination is made by reading the servos states. "
                      "If the keyword 'chain' is specified, then all servos "
                      "in the chain are checked.\n"
                      "  <servo_id>  Servo id [0-253].";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdReadIsMoving() { }

protected:

  /*!
   * \brief Determine if servo is moving.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo.
   */
  virtual void doExec(DynaShell &shell, DynaServo *pServo)
  {
    bool        bIsMoving;
    int         rc;

    rc = pServo->ReadIsMoving(&bIsMoving);

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Servo %d: Read motion state failed.",
          pServo->GetServoId());
      return;
    }

    shell.Response("Servo %3d: %s\n", pServo->GetServoId(),
            (bIsMoving? "moving": "stopped"));
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdClearAlarms Class
// -----------------------------------------------------------------------------

/*!
 * \brief Determine if servos are moving.
 */
class DynaShellCmdClearAlarms : public DynaShellCmdChainIn
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdClearAlarms()
  {
    m_sCmdName      = "alarms";
    m_sCmdHelpBrief = "Clear servo alarms (if possible).";
    m_sCmdHelpArgs  = "<servo_id> [servo_id...]\nchain";
    m_sCmdHelpDesc  = "Clear all clearable alarms raised by serovs. "
                      "If the keyword 'chain' is specified, then all servos "
                      "alarms in the chain are cleared.\n"
                      "  <servo_id>  Servo id [0-253].";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdClearAlarms() { }

protected:

  /*!
   * \brief Determine if servo is moving.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo.
   */
  virtual void doExec(DynaShell &shell, DynaServo *pServo)
  {
    uint_t      uAlarms;
    int         rc;

    uAlarms = pServo->GetAlarms();

    if( uAlarms == DYNA_ALARM_NONE )
    {
      shell.Response("Servo %3d: %s.\n", pServo->GetServoId(), "No alarms");
    }
    else if( uAlarms & DYNA_ALARM_LOAD )
    {
      pServo->Stop();
      rc = pServo->ReloadMaxTorqueLimit();
      if( rc != DYNA_OK )
      {
        shell.Error(rc, "Servo %3d: Failed to reload maximum torque limit.",
          pServo->GetServoId());
      }
      else if( uAlarms == DYNA_ALARM_LOAD )
      {
        shell.Response("Servo %3d: %s.\n", pServo->GetServoId(),
            "Over-load alarm cleared");
      }
      else
      {
        shell.Response("Servo %3d: %s.\n", pServo->GetServoId(),
          "Over-load alarm cleared, but other unclearable alarms are present");
      }
    }
    else
    {
      shell.Response("Servo %3d: %s.\n", pServo->GetServoId(),
          "Alarms present, but not clearable");
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdReadByte Class
// -----------------------------------------------------------------------------

/*!
 * \brief Read servo byte value at address.
 */
class DynaShellCmdReadByte : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdReadByte() : DynaShellCmd(2, 1+DYNA_ID_NUMOF)
  {
    m_sCmdName      = "byte";
    m_sCmdHelpBrief = "Read a byte from the servo control table.";
    m_sCmdHelpArgs  = "<addr> <servo_id> [servo_id...]\n<addr> chain";
    m_sCmdHelpDesc  = "Read an 8-bit byte at the address for the specified "
                      "servos. If the keyword 'chain' is specified, then"
                      "the byte value at <addr> is read for all servos in the "
                      "chain.\n"
                      "  <addr>      Servo memory address.\n"
                      "  <servo_id>  Servo id [0-253]."
                      "\n\n"
                      "Caution: This is a low-level function that bypasses all "
                      "state consistency checks.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdReadByte() { }

  /*!
   * \brief Execute read byte command on servos.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  virtual void Exec(DynaShell &shell, int argc, char *argv[])
  {
    uint_t  uAddr;
    int     nServoId;
    int     iter;

    TRY( ChkArgCnt(shell, argc) );
    TRY( ChkComm(shell) );
    TRY( ChkChainNotEmpty(shell) );

    TRY( ToInt(shell, argv[0], (int *)&uAddr) );

    if( !strcmp(argv[1], "chain") )
    {
      TRY( ChkArgCntEQ(shell, argc, 2) );

      for(nServoId = shell.m_pDynaChain->IterStart(&iter);
          nServoId != DYNA_ID_NONE;
          nServoId = shell.m_pDynaChain->IterNext(&iter))
      {
        doExec(shell, nServoId, uAddr);
      }
    }

    else
    {
      for(iter=1; iter<argc; ++iter)
      {
        TRY( ToInt(shell, argv[iter], &nServoId) );
        TRY( ChkChainHasServo(shell, nServoId) );

        doExec(shell, nServoId, uAddr);
      }
    }
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes NULL <servo_id> | chain [servo_id [servo_id ...]]
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
    int         nArgNum;

    char  buf[16];
  
    if( (shell.m_pDynaChain == NULL) ||
        (shell.m_pDynaChain->GetNumberInChain() == 0) )
    {
      return NULL;
    }
  
    // argument number of already (expanded) arguments
    nArgNum = ReadLine::wc(sContext) - ReadLine::wc(m_sPubName);

    // <addr> cannot complete adrress
    if( nArgNum == 0 )
    {
      return NULL;
    }

    //
    // New command to complete - initialize.
    //
    if( nState == ReadLine::FIRST )
    {
      m_nTabServoId = shell.m_pDynaChain->IterStart(&m_nTabIter);
  
      if( nArgNum == 1 )
      {
        m_bTabIncChain = true;
      }
      else
      {
        m_bTabIncChain = false;
      }
    }
  
    while( m_nTabServoId != DYNA_ID_NONE )
    {
      snprintf(buf, sizeof(buf), "%d", m_nTabServoId);
      buf[sizeof(buf)-1] = 0;
  
      m_nTabServoId = shell.m_pDynaChain->IterNext(&m_nTabIter);
  
      if( !strncmp(buf, sText, uTextLen) )
      {
        return ReadLine::dupstr(buf);
      }
    }
  
    if( m_bTabIncChain )
    {
      m_bTabIncChain = false;
      if( !strncmp("chain", sText, uTextLen) )
      {
        return ReadLine::dupstr("chain");
      }
    }
  
    // no more matches
    return NULL;
  }

protected:
  int  m_nTabIter;        ///< tab completion: servo id iterator
  int  m_nTabServoId;     ///< tab completion: current servo id
  bool m_bTabIncChain;    ///< tab completion: [do not] include chain keyword

  /*!
   * \brief Read byte.
   *
   * \param shell     Dynamixel shell.
   * \param nServoId  Servo id.
   * \param uAddr     Servo read address.
   */
  virtual void doExec(DynaShell &shell, int nServoId, uint_t uAddr)
  {
    uint_t      uVal;
    int         rc;

    rc = shell.m_pDynaComm->Read8(nServoId, uAddr, &uVal);

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Servo %d: Failed to read byte at 0x%02x.",
          nServoId, uAddr);
      return;
    }

    shell.Response("Servo %3d: 0x%02x.\n", nServoId, uVal);
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdWriteByte Class
// -----------------------------------------------------------------------------

/*!
 * \brief Write byte value to servo at address.
 */
class DynaShellCmdWriteByte : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdWriteByte() : DynaShellCmd(3, 1 + DYNA_ID_NUMOF * 2)
  {
    m_sCmdName      = "byte";
    m_sCmdHelpBrief = "Write a byte to the servo control table.";
    m_sCmdHelpArgs  = "<addr> <servo_id> <val> [<servo_id> <val> ...]\n"
                      "<addr> chain <val> ";
    m_sCmdHelpDesc  = "Write an 8-bit byte to the address for the specified "
                      "servos. "
                      "If the keyword 'chain' is specified, then "
                      "the byte value is written to <addr> for all servos in "
                      "the chain.\n"
                      "If more than 1 servo is specified, a synchronous write "
                      "is performed.\n"
                      "  <addr>      Servo memory address.\n"
                      "  <servo_id>  Servo id [0-253].\n"
                      "  <val>       New servo value."
                      "\n\n"
                      "Caution: This is a low-level function that bypasses all "
                      "state consistency checks.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdWriteByte() { }

  /*!
   * \brief Execute write byte command on servos.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  virtual void Exec(DynaShell &shell, int argc, char *argv[])
  {
    DynaSyncWriteTuple_T  tup[DYNA_ID_NUMOF];
    uint_t  uAddr;
    uint_t  uVal;
    int     nServoId;
    int     iter;
    int                   cnt;
    uint_t  uNumPairs;

    TRY( ChkArgCnt(shell, argc) );
    TRY( ChkComm(shell) );
    TRY( ChkChainNotEmpty(shell) );

    TRY( ToInt(shell, argv[0], (int *)&uAddr) );

    cnt = argc - 1; // less address

    if( (cnt % 2) != 0 )
    {
      shell.Error("Unmatched servo_id,value pairs.");
      return;
    }

    uNumPairs = (uint_t)(cnt / 2);

    if( uNumPairs > DYNA_ID_NUMOF )
    {
      shell.Error("%u: Too many servo_id,value pairs.", uNumPairs);
      return;
    }

    if( !strcmp(argv[1], "chain") )
    {
      TRY( ChkArgCntEQ(shell, argc, 3) );

      TRY( ToInt(shell, argv[2], (int *)&uVal) );

      for(nServoId = shell.m_pDynaChain->IterStart(&iter), cnt=0;
          nServoId != DYNA_ID_NONE;
          nServoId = shell.m_pDynaChain->IterNext(&iter), ++cnt)
      {
        tup[cnt].m_nServoId = nServoId;
        tup[cnt].m_uVal     = uVal;
      }
    }

    else
    {
      for(iter=1, cnt=0; iter<argc; iter+=2, ++cnt)
      {
        TRY( ToInt(shell, argv[iter], &nServoId) );
        TRY( ChkChainHasServo(shell, nServoId) );
        TRY( ToInt(shell, argv[iter+1], (int *)&uVal) );

        tup[cnt].m_nServoId = nServoId;
        tup[cnt].m_uVal     = uVal;
      }
    }

    doExec(shell, uAddr, tup, (uint_t)cnt);
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes NULL <servo_id> | chain [servo_id [servo_id ...]]
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
    int         nArgNum;

    char  buf[16];
  
    if( (shell.m_pDynaChain == NULL) ||
        (shell.m_pDynaChain->GetNumberInChain() == 0) )
    {
      return NULL;
    }
  
    // argument number of already (expanded) arguments
    nArgNum = ReadLine::wc(sContext) - ReadLine::wc(m_sPubName);

    // cannot complete adrress
    if( nArgNum == 0 )
    {
      return NULL;
    }

    // every even argument is the servo value, which cannot be expanded
    else if( (nArgNum & 0x01) == 0 )
    {
      return NULL;
    }

    // New command argument to complete - initialize.
    //
    if( nState == ReadLine::FIRST )
    {
      m_nTabServoId = shell.m_pDynaChain->IterStart(&m_nTabIter);
  
      if( nArgNum == 1 )
      {
        m_bTabIncChain = true;
      }
      else
      {
        m_bTabIncChain = false;
      }
    }
  
    while( m_nTabServoId != DYNA_ID_NONE )
    {
      snprintf(buf, sizeof(buf), "%d", m_nTabServoId);
      buf[sizeof(buf)-1] = 0;
  
      m_nTabServoId = shell.m_pDynaChain->IterNext(&m_nTabIter);
  
      if( !strncmp(buf, sText, uTextLen) )
      {
        return ReadLine::dupstr(buf);
      }
    }
  
    if( m_bTabIncChain )
    {
      m_bTabIncChain = false;
      if( !strncmp("chain", sText, uTextLen) )
      {
        return ReadLine::dupstr("chain");
      }
    }
  
    // no more matches
    return NULL;
  }

protected:
  int  m_nTabIter;        ///< tab completion: servo id iterator
  int  m_nTabServoId;     ///< tab completion: current servo id
  bool m_bTabIncChain;    ///< tab completion: [do not] include chain keyword

  /*!
   * \brief Write byte.
   *
   * \param shell   Dynamixel shell.
   * \param uAddr   Write address.
   * \param tup     Array of (servo id, value) 2-tuples.
   * \param uCount  Number of tuples.
   */
  virtual void doExec(DynaShell            &shell, 
                      uint_t                uAddr,
                      DynaSyncWriteTuple_T  tup[],
                      uint_t                uCount)
  {
    int         rc;

    if( uCount == 1 )
    {
      rc = shell.m_pDynaComm->Write8(tup[0].m_nServoId, uAddr, tup[0].m_uVal);

      if( rc != DYNA_OK )
      {
        shell.Error(rc,
          "Servo %d: Failed to write byte 0x%02x to 0x%02x.",
          tup[0].m_nServoId, tup[0].m_uVal, uAddr);
        return;
      }
    }
    else
    {
      rc = shell.m_pDynaComm->SyncWrite(uAddr, 1, tup, uCount);

      if( rc != DYNA_OK )
      {
        shell.Error(rc,
          "Failed to syncwrite to 0x%02x for %u servos.",
          uAddr, uCount);
        return;
      }
    }

    shell.Ok();
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdReadWord Class
// -----------------------------------------------------------------------------

/*!
 * \brief Read servo byte value at address.
 */
class DynaShellCmdReadWord : public DynaShellCmdReadByte
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdReadWord()
  {
    m_sCmdName      = "word";
    m_sCmdHelpBrief = "Read a word from the servo control table.";
    m_sCmdHelpArgs  = "<addr> <servo_id> [servo_id...]\n<addr> chain";
    m_sCmdHelpDesc  = "Read a 16-bit word at the address for the specified "
                      "servos.\n"
                      "If the keyword 'chain' is specified, then"
                      "the word value at <addr> is read for all servos in the "
                      "chain.\n"
                      "  <addr>      Servo memory address.\n"
                      "  <servo_id>  Servo id [0-253]."
                      "\n\n"
                      "Caution: This is a low-level function that bypasses all "
                      "state consistency checks.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdReadWord() { }

protected:

  /*!
   * \brief Read word.
   *
   * \param shell     Dynamixel shell.
   * \param nServoId  Servo id.
   * \param uAddr     Servo read address.
   */
  virtual void doExec(DynaShell &shell, int nServoId, uint_t uAddr)
  {
    uint_t      uVal;
    int         rc;

    rc = shell.m_pDynaComm->Read16(nServoId, uAddr, &uVal);

    if( rc != DYNA_OK )
    {
      shell.Error(rc, "Servo %d: Failed to read byte at 0x%02x.",
          nServoId, uAddr);
      return;
    }

    shell.Response("Servo %3d: 0x%04x.\n", nServoId, uVal);
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdWriteWord Class
// -----------------------------------------------------------------------------

/*!
 * \brief Write byte value to servo at address.
 */
class DynaShellCmdWriteWord : public DynaShellCmdWriteByte
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdWriteWord()
  {
    m_sCmdName      = "word";
    m_sCmdHelpBrief = "Write a word to the servo control table.";
    m_sCmdHelpArgs  = "<addr> <servo_id> <val> [<servo_id> <val> ...]\n"
                      "<addr> chain <val> ";
    m_sCmdHelpDesc  = "Write a 16-bit word to the address for the specified "
                      "servos. "
                      "If the keyword 'chain' is specified, then "
                      "the word <val> is written to <addr> for all servos in "
                      "the chain.\n"
                      "If more than 1 servo is specified, a synchronous write "
                      "is performed.\n"
                      "  <addr>      Servo memory address.\n"
                      "  <servo_id>  Servo id [0-253].\n"
                      "  <val>       New servo value."
                      "\n\n"
                      "Caution: This is a low-level function that bypasses all "
                      "state consistency checks.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdWriteWord() { }

protected:
  /*!
   * \brief Write word.
   *
   * \param shell   Dynamixel shell.
   * \param uAddr   Write address.
   * \param tup     Array of (servo id, value) 2-tuples.
   * \param uCount  Number of tuples.
   */
  virtual void doExec(DynaShell            &shell, 
                      uint_t                uAddr,
                      DynaSyncWriteTuple_T  tup[],
                      uint_t                uCount)
  {
    int         rc;

    if( uCount == 1 )
    {
      rc = shell.m_pDynaComm->Write16(tup[0].m_nServoId, uAddr, tup[0].m_uVal);

      if( rc != DYNA_OK )
      {
        shell.Error(rc,
          "Servo %d: Failed to write word 0x%04x to 0x%02x.",
          tup[0].m_nServoId, tup[0].m_uVal, uAddr);
        return;
      }
    }
    else
    {
      rc = shell.m_pDynaComm->SyncWrite(uAddr, 2, tup, uCount);

      if( rc != DYNA_OK )
      {
        shell.Error(rc,
          "Failed to syncwrite to 0x%02x for %u servos.",
          uAddr, uCount);
        return;
      }
    }

    shell.Ok();
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdDump Class
// -----------------------------------------------------------------------------

/*!
 * \brief Read servo byte value at address.
 */
class DynaShellCmdDump : public DynaShellCmdChainIn
{
public:
  /*
   * \brief Default constructor.
   */
  DynaShellCmdDump() : DynaShellCmdChainIn()
  {
    m_sCmdName      = "tables";
    m_sCmdHelpBrief = "Dump servo EEPROM and RAM control tables.";
    m_sCmdHelpArgs  = "<servo_id> [servo_id...]\n"
                      "chain";
    m_sCmdHelpDesc  = "Dump servo control tables  "
                      "for the specified "
                      "servos. If the keyword 'chain' is specified, then "
                      "the memory is dumped for all servos in the "
                      "chain.\n"
                      "  <servo_id>    Servo id [0-253].";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdDump() { }

protected:
  /*!
   * \brief Read byte.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo.
   */
  virtual void doExec(DynaShell &shell, DynaServo *pServo)
  {
    uint_t      uAddr;
    uint_t      uVal;
    int         rc;

    shell.Response("\n");
    shell.Response( "-------------------------"
                    " Servo %d Control Tables "
                    "-------------------------\n",
        pServo->GetServoId());
    pServo->Dump();
  }
};


// -----------------------------------------------------------------------------
// Public Interface.
// -----------------------------------------------------------------------------

/*!
 * \brief Publish shell servo commands to shell.
 *
 * \brief shell   Dynamixel shell.
 */
void PublishShellServoCommands(DynaShell &shell)
{
  shell.PublishCommand(NULL, new DynaShellCmdMoveTo());
  shell.PublishCommand(NULL, new DynaShellCmdMoveAtSpeedTo());
  shell.PublishCommand(NULL, new DynaShellCmdEStop());
  shell.PublishCommand(NULL, new DynaShellCmdStop());
  shell.PublishCommand(NULL, new DynaShellCmdFreeze());
  shell.PublishCommand(NULL, new DynaShellCmdRelease());

  shell.PublishCommand("get", new DynaShellCmdGetOdometer());
  shell.PublishCommand("set", new DynaShellCmdSetOdometer());

  shell.PublishCommand("write", new DynaShellCmdCfgWriteServoMode());

  shell.PublishCommand("read", new DynaShellCmdReadTorqueEnable());
  shell.PublishCommand("write", new DynaShellCmdWriteTorqueEnable());

  shell.PublishCommand("read", new DynaShellCmdReadGoalPos());
  //shell.PublishCommand("read", new DynaShellCmdWriteGoalPos());

  shell.PublishCommand("read", new DynaShellCmdReadGoalSpeed());
  shell.PublishCommand("write", new DynaShellCmdWriteGoalSpeed());

  shell.PublishCommand("read", new DynaShellCmdReadMaxTorqueLimit());
  shell.PublishCommand("write", new DynaShellCmdWriteMaxTorqueLimit());

  shell.PublishCommand("get", new DynaShellCmdGetSoftTorqueTh());
  shell.PublishCommand("set", new DynaShellCmdSetSoftTorqueTh());

  shell.PublishCommand("read", new DynaShellCmdReadCurPos());
  shell.PublishCommand("read", new DynaShellCmdReadCurSpeed());

  shell.PublishCommand("read", new DynaShellCmdReadDynamics());
  shell.PublishCommand("read", new DynaShellCmdReadHealth());
  shell.PublishCommand("clear", new DynaShellCmdClearAlarms());

  shell.PublishCommand("read", new DynaShellCmdReadIsMoving());

  shell.PublishCommand("read", new DynaShellCmdReadByte());
  shell.PublishCommand("write", new DynaShellCmdWriteByte());

  shell.PublishCommand("read", new DynaShellCmdReadWord());
  shell.PublishCommand("write", new DynaShellCmdWriteWord());

  shell.PublishCommand("dump", new DynaShellCmdDump());
}
