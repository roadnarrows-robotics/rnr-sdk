################################################################################
#
# BsProxyPacker.py
#

""" BotSense Proxy Client Packer/Unpacker module.

The BsProxyPacker class provides byte stream packing and unpacker specific
to a BotSense Proxy client.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2007.11.11

Copyright (C) 2007, RoadNarrows LLC.
"""

#
# All Rights Reserved
#
# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that
# (1) The above copyright notice and the following two paragraphs
# appear in all copies of the source code and (2) redistributions
# including binaries reproduces these notices in the supporting
# documentation.   Substantial modifications to this software may be
# copyrighted by their authors and need not follow the licensing terms
# described here, provided that the new terms are clearly indicated in
# all files where they apply.
#
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
# OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
#
################################################################################

from Fusion.KHR2.Cmd.BsProxyMsgDef import *
import Fusion.Utils.Packer as Packer

#-------------------------------------------------------------------------------
# CLASS: BsProxyPacker
#-------------------------------------------------------------------------------
class BsProxyPacker(Packer.Packer):
  """ BotSense Proxy Client byte stream packer/unpacker class.
  """
  def __init__(self):
    """ Initialize packer instance.
    """
    self._tid = 0             # transaction id
    self._lasttid = 0         # last transaction id
    self._cmd_hdr_fmt = [     # command header packing format
        {'fval':0, 'ftype':'u8'},
        {'fval':0, 'ftype':'u8'},
        {'fval':0, 'ftype':'u8'}]
    self._rsp_hdr_fmt = [     # pass response header unpacking format
        {'fname':'msgid', 'ftype':'u8'},
        {'fname':'tid', 'ftype':'u8'},
        {'fname':'pf', 'ftype':'char'},
        {'fname':'blen', 'ftype':'u8'}]
    self._rsp_err_fmt = [{'fname':'_error_msg', 'ftype':'zstr'}]
                              # fail response header unpacking format

  #--
  def GenTid(self):
    """ Generate next transaction id

        Return value:
          Tracking id
    """
    self._lasttid = self._tid
    self._tid = (self._tid + 1) % 256
    return self._lasttid

  #--
  def GetLastTid(self):
    """ Get last transaction id sent.

        Return value:
          Last transaction id
    """
    return self._lasttid

  #--
  def PackMsg(self, msgId, packfmt):
    """ Pack BotSense Proxy client command message.

        The command header is automatically added.

        Parameters:
          msgId       - command message id
          packfmt     - See Packer.PackMsg()

        Return value:
          List of packed bytes.
    """
    body = Packer.Packer.PackMsg(self, packfmt)
    if type(msgId) == str:
      msgId = ord(msgId)
    self._cmd_hdr_fmt[BsProxyCmdHdrMsgIdIdx]['fval']  = msgId
    self._cmd_hdr_fmt[BsProxyCmdHdrTidIdx]['fval']    = self.GenTid()
    self._cmd_hdr_fmt[BsProxyCmdHdrBLenIdx]['fval']   = len(body)
    cmdhdr = Packer.Packer.PackMsg(self, self._cmd_hdr_fmt)
    return cmdhdr+body

  #--
  def UnpackMsg(self, msg, msgId, unpackfmt, tid=-1):
    """ Unpack BotSense Proxy server response message into fields.

        Parameters:
          msg       - message to unpacke
          msgId     - expected response message id
          unpackfmt - See Packer.UnpackMsg()
          tid       - expected response message transaction id.
                      not checked if tid == -1.

        Return value:
          See Packer.UnpackMsg(). Response header fname keyword values are:
            'msgid'   - response message id
            'tid'     - response transaction id
            'pf'      - pass/fail flag
            'blen'    - length of response message body (number of bytes)
    """
    if type(msgId) == str:
      msgId = ord(msgId)
    rsp = Packer.Packer.UnpackMsg(self, msg, self._rsp_hdr_fmt, 0)
    if rsp['_rc'] == 'error':
      rsp['_error_msg'] = "bad response header: " + rsp['_error_msg']
      return rsp
    elif rsp['msgid'] != msgId:
      return self._error(rsp, 0,
          "command 0x%02x - response 0x%02x message id mismatch" % \
              (msgId, rsp['msgid']))
    elif tid >= 0 and rsp['tid'] != tid:
      return self._error(rsp, 1, 
          "command %d - response %d transaction id mismatch" % \
              (tid, rsp['tid']))
    if rsp['pf'] == 'P':
      body = Packer.Packer.UnpackMsg(self, msg, unpackfmt, 4)
    else:
      body = Packer.Packer.UnpackMsg(self, msg, self._rsp_err_fmt, 4)
      body['_rc'] = 'error'
    rsp.update(**body)
    return rsp
