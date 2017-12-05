////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      StringTheory.cxx
//
/*! \file
 *
 * \brief Of string spaces and their strangian operators.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2016-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * \par License:
 * MIT
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <ctype.h>

#include <sstream>
#include <string>
#include <locale>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/StringTheory.h"

using namespace std;

namespace rnr
{
  namespace str
  {
    const char *FalseHood[] =
    {
      "0", "false", "f", "off", "low", "disable", "no", NULL
    };

    const char *TruthHood[] =
    {
      "1", "true", "t", "on", "high", "enable", "yes", NULL
    };

    size_t split(const string &str, const char delim, StringVec &elems)
    {
      size_t        n = elems.size();
      stringstream  ss;
      string        item;
    
      ss.str(str);
    
      while(std::getline(ss, item, delim) )
      {
        elems.push_back(item);
      }
    
      return elems.size() - n;
    }
    
    StringVec split(const string &str, const char delim)
    {
      StringVec elems;
    
      split(str, delim, elems);
    
      return elems;
    }
    
    string &replace(const string &what, const string &with, string &str)
    {
      size_t  len = what.length();
      size_t  n;

      // no search for string or with has a what
      if( (len == 0) || (with.find(what) != string::npos) )
      {
        return str;
      }

      // replace all whats
      while( (n = str.find(what)) != string::npos )
      {
        str.replace(n, len, with);
      }

      return str;
    }

    string lowercase(const string &str)
    {
      std::locale loc;
      string      lower;
    
      for(size_t i = 0; i < str.size(); ++i)
      {
        lower.push_back(std::tolower(str[i], loc));
      }
    
      return lower;
    }
    
    string uppercase(const string &str)
    {
      std::locale loc;
      string      upper;
    
      for(size_t i = 0; i < str.size(); ++i)
      {
        upper.push_back(std::toupper(str[i], loc));
      }
    
      return upper;
    }
    
    size_t gcss(const string &str1, const string &str2, const size_t pos)
    {
      size_t  i;
      size_t  len = 0;
    
      for(i = pos; (i < str1.size()) && (i < str2.size()); ++i)
      {
        if( str1[i] == str2[i] )
        {
          ++len;
        }
        else
        {
          break;
        }
      }
    
      return len;
    }

    int tobool(const string &str, bool &val)
    {
      size_t    i;
    
      for(i = 0; FalseHood[i] != NULL; ++i)
      {
        if( str == FalseHood[i] )
        {
          val = false;
          return OK;
        }
      }
    
      for(i = 0; TruthHood[i] != NULL; ++i)
      {
        if( str == TruthHood[i] )
        {
          val = true;
          return OK;
        }
      }
    
      return RC_ERROR;
    }
    
    int tolong(const string &str, long &val)
    {
      char  sniff;
    
      return sscanf(str.c_str(), "%li%c", &val, &sniff) == 1? OK: RC_ERROR;
    }
    
    int todouble(const string &str, double &val)
    {
      char  sniff;
    
      return sscanf(str.c_str(), "%lf%c", &val, &sniff) == 1? OK: RC_ERROR;
    }

    string prettify(const string &str)
    {
      string    realpurdy;
      bool      quoteit = false;
    
      for(size_t i = 0; i < str.length(); ++i)
      {
        switch( str[i] )
        {
          case '"':
            quoteit = true;
            realpurdy.append("\\\"");
            break;
          case ' ':
            quoteit = true;
            realpurdy.append(" ");
            break;
          case '\t':
            quoteit = true;
            realpurdy.append("\\t");
            break;
          case '\n':
            quoteit = true;
            realpurdy.append("\\n");
            break;
          case '\r':
            quoteit = true;
            realpurdy.append("\\r");
            break;
          case '\v':
            quoteit = true;
            realpurdy.append("\\v");
            break;
          case '\f':
            quoteit = true;
            realpurdy.append("\\f");
            break;
          case '\\':
            quoteit = true;
            realpurdy.append("\\");
            break;
          default:
            if( (str[i] < 0x21) || (str[i] > 0x7e) )
            {
              unsigned  hh = (unsigned)str[i];
              char      buf[8];
    
              sprintf(buf, "\\x%02x", hh);
              realpurdy.append(buf);
              quoteit = true;
            }
            else
            {
              realpurdy.push_back(str[i]);
            }
            break;
        }
      }
    
      if( quoteit )
      {
        realpurdy.insert(realpurdy.begin(), '"');
        realpurdy.insert(realpurdy.end(), '"');
      }
    
      return realpurdy;
    }

    string c14n(const string &str)
    {
      string  strc14n(str);
      size_t  pos;

      // strip leading and trailing whitespace
      trim(strc14n);

      return prettify(strc14n);
    }

    string c14n(const StringVec &tokens)
    {
      stringstream  ss;
      string        sep;

      for(size_t i = 0; i < tokens.size(); ++i)
      {
        ss << sep << prettify(tokens[i]);

        if( i == 0 )
        {
          sep = " ";
        }
      }

      return ss.str();
    }

  } // namespace str
} // namespace rnr
