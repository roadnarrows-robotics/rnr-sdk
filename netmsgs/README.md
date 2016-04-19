# netmsgs
RoadNarrows Robotics Network Messaging Packing/Unpacking Library

## Description
The netmsgs package provides both build-time and run-time support of
networked messaging presented in a format that is both machine architecture
independent and language independent. The messages can sent between
networked nodes over a communication channel. In the OSI Model, the netmsgs
package supports the Presentation Layer. The lower communication levels
and the Application Layer are not in the scope of netmsgs.

Supported message encodings:

* Structured, binary messages with the message fields formatted in an
      Identifier-Type-Value order.
      Both big and small endian value codings are supported.
* Flat, binary messages with the message fields formatted in a fixed order
      without any identification or type tagging. The fields are all of fixed
      Both big and small endian value codings are supported.
      length.
* Simple command-Line, ASCII messages with messages fields separatated by
      Inter-Field Separators (usually white space) and terminated by End-Of-Line
      sequences (usually CR, LF, or CR-LF). (future).

## Copyright
&#169; 2010-2016 RoadNarrows LLC<br>
[roadnarrows.com](http://roadnarrows.com)<br>
All Rights Reserved

## License
RoadNarrows Free. See the EULA for more details.

## RoadNarrows Required Packages
* librnr

## See Also
See documentation for more details.
