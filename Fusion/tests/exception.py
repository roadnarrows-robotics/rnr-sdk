import errno
import os

import Fusion.Utils.Enum as Enum

# 
# Channel Error Codes
#
ChErrCode = Enum.Enum(
    "NOTOPEN NOCONN CMDTIMEOUT NORSP RSPTIMEOUT "
    "BADRSP BADCHKSUM BADCRC BADSEQNUM",
    -1, -1)

#
# Channel Error Code Message Strings
#
ChErrStr = {
  ChErrCode.NOTOPEN:    "Channel not open",
  ChErrCode.NOCONN:     "No connection",
  ChErrCode.CMDTIMEOUT: "Command timed out",
  ChErrCode.NORSP:      "No response",
  ChErrCode.RSPTIMEOUT: "Response timed out",
  ChErrCode.BADRSP:     "Bad response",
  ChErrCode.BADCHKSUM:  "Bad check sum",
  ChErrCode.BADCRC:     "Bad CRC",
  ChErrCode.BADSEQNUM:  "Bad sequence number"
}

class ChannelError(Exception):
  """ Channel Error Exception Class.

      Attributes (in order):
        cherr   - channel error code
                    cherr < 0:  channel specific error code
                    cherr > 0:  system specific error code (errno)
        estr    - translated channel error code to standard message
                  string
        emsg    - context specific explanation of the error
  """
  #--
  def __init__(self, cherr, emsg=None):
    """ Initialize (raise) exception.

        Parameters:
          cherr   - channel error code
                      cherr < 0:  channel specific error code
                      cherr > 0:  system specific error code (errno)
          emsg    - context specific explanation of the error
    """
    self.cherr = cherr
    if cherr < 0:
      if ChErrCode.has_enum(cherr):
        self.ename = ChErrCode.name(cherr)
        self.estr = ChErrStr[cherr]
      else:
        self.ename = ''
        self.estr = "Unknown channel error %s" % repr(cherr)
    else:
      self.estr = os.strerror(cherr) # also maps unknown
      if cherr in errno.errorcode:
        self.ename = errno.errorcode[cherr]
      else:
        self.ename = repr(cherr)
    if emsg:
      self.emsg = emsg
    else:
      self.emsg = ""
    Exception.__init__(self, self.cherr, self.estr, self.emsg)

  #--
  def __str__(self):
    """ Return formated error string. """
    if self.emsg:
      return "[ChannelError: %s(%d)] %s: %s" \
          % (self.ename, self.cherr, self.estr, self.emsg)
    else:
      return "[ChannelError: %s(%d)] %s" \
          % (self.ename, self.cherr, self.estr)


def t1():
  """ Raise ChannelError(5) """
  try:
    raise ChannelError(5)
  except ChannelError as e:
    print("ChannelError: %d, %s" % (e.cherr, e.emsg))
    print(e)
    
def t2():
  """ Raise ChannelError(-1, "ugh") """
  try:
    raise ChannelError(-1, "ugh")
  except ChannelError as xxx_todo_changeme:
    (code, strerror, msg) = xxx_todo_changeme.args
    print("ChannelError: %d: %s: %s" % (code, strerror, msg))

def t3():
  """ Raise ChannelError(-2, "badboy") """
  try:
    raise ChannelError(-2, "badboy")
  except ChannelError as e:
    print(e)

def t4():
  """ Raise ChannelError(-99, "what?") """
  try:
    raise ChannelError(-99, "what?")
  except ChannelError as e:
    print(e)

def t5():
  """ Raise IOError(5, "file") """
  try:
    raise IOError(5, "file")
  except IOError as xxx_todo_changeme1:
    (x, y) = xxx_todo_changeme1.args
    print(x, y)

def t6():
  """ Open noexist file """
  try:
    f = open("/oogabooga", "r")
  except IOError as err:
    print(err)

def t6():
  """ Open noexist file v2 """
  try:
    f = open("/oogabooga", "r")
  except IOError as xxx_todo_changeme2:
    (errno, strerror) = xxx_todo_changeme2.args
    print(errno, strerror)
