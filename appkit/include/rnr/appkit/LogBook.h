////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      LogBook.h
//
/*! \file
 *
 * \brief 
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2017.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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

#ifndef _LOG_BOOK_H
#define _LOG_BOOK_H

#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <deque>
#include <map>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

namespace rnr
{
  class LogBook
  {
  public:
    //
    // Defaults
    //
    static const size_t MaxEntriesDft  = 100;     ///< max num entries default
    static const size_t MaxEntryLenDft = 0x2000;  ///< max entry length (8192)

    //
    // Whence it came
    //
    static const int  OLDEST = 0;   ///< oldest entries 
    static const int  NEWEST = 1;   ///< newest, most recent entries

    //
    // Formatting+ flags
    //
    static const unsigned FlagNone    = 0x0000; ///< no flags
    static const unsigned FlagONum    = 0x0001; ///< output log entry number
    static const unsigned FlagOTime   = 0x0002; ///< output log entry time
    static const unsigned FlagOMark   = 0x0004; ///< output log entry bookmarks
    static const unsigned FlagOAllF   = 0x0007; ///< output all entry fields
    static const unsigned FlagORev    = 0x0008; ///< output log reverse order
    static const unsigned FlagDebug   = 0x1000; ///< debug

    /*!
     * \brief Log entry structure.
     */
    struct Entry
    {
      timespec    m_timestamp;  ///< entry timestamp
      std::string m_strMark;    ///< entry bookmark, empty if no mark
      std::string m_strText;    ///< entry text, empty if no text

      /*!
       * \brief Default contructor.
       */
      Entry();

      /*!
       * \brief Initialization contructor.
       *
       * \param strMark   Entry bookmark.
       * \param strText   Entry text.
       */
      Entry(const std::string &strMark, const std::string &strText);

      /*!
       * \brief Copy contructor.
       *
       * \param src Source object.
       */
      Entry(const Entry &src);

      /*!
       * \brief Destructor.
       */
      virtual ~Entry();

      /*!
       * \brief Assignment operator.
       *
       * \param rhs   Right-hand side object.
       *
       * \return *this
       */
      Entry &operator=(const Entry &rhs);

      /*!
       * \brief Test if entry is empty ("no entry" entry).
       *
       * \return Returns true or false.
       */
      bool empty();
    };

    //
    // Some useful Entry types
    //
    typedef std::vector<Entry>   EntryList;       ///< entry list type
    typedef EntryList::iterator  EntryListIter;   ///< entry list iterator
    typedef EntryList::const_iterator  EntryListCIter;
                                                  ///< entry const list iterator

    /*!
     * \brief Log bookmark structure.
     */
    struct BookMark
    {
      std::string m_strMark;
      size_t      m_index;

      /*!
       * \brief Default contructor.
       */
      BookMark();

      /*!
       * \brief Copy contructor.
       *
       * \param src Source object.
       */
      BookMark(const BookMark &src);

      /*!
       * \brief Assignment operator.
       *
       * \param rhs   Right-hand side object.
       *
       * \return *this
       */
      BookMark &operator=(const BookMark &rhs);
    };

    //
    // Some useful BookMark types
    //
    typedef std::vector<BookMark>   BookMarkList; ///< bookmark list type
    typedef BookMarkList::iterator  BookMarkListIter;
                                                  ///< bookmark list iterator
    typedef BookMarkList::const_iterator  BookMarkListCIter;
                                                  ///< bookmark const list iter

    /*!
     * \brief Convenience macro to bookmark log using function name.
     */
#define BOOKMARKFUNC()  bookmark(__func__)

    /*!
     * \brief Default initialization constructor.
     *
     * \param strName       Name of log book.
     * \param uMaxEntries   Maximum number of entries in log book.
     * \param uMaxEntryLen  Maximum entry length.
     */
    LogBook(const std::string strName      = "Log Book",
            size_t            uMaxEntries  = MaxEntriesDft,
            size_t            uMaxEntryLen = MaxEntryLenDft);


    /*!
     * \brief Copy constructor.
     *
     * \param src Source object.
     */
    LogBook(const LogBook &src);

    /*!
     * \brief Destructor.
     */
    virtual ~LogBook();

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right-hand side object.
     *
     * \return *this
     */
    LogBook &operator=(const LogBook &rhs);

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // LogBook Insertion Operators
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
    //
    // Arithmetic
    //
    LogBook &operator<<(bool val);
    LogBook &operator<<(char val);
    LogBook &operator<<(int val);
    LogBook &operator<<(unsigned int val);
    LogBook &operator<<(long val);
    LogBook &operator<<(unsigned long val);
    LogBook &operator<<(long long val);
    LogBook &operator<<(unsigned long long val);
    LogBook &operator<<(float val);
    LogBook &operator<<(double val);
    LogBook &operator<<(long double val);

    //
    // Pointers
    //
    LogBook &operator<<(const char *val);
    LogBook &operator<<(void *val);

    //
    // Speical classes
    //
    LogBook &operator<<(const std::string &val);
    LogBook &operator<<(const LogBook &log);      // append

    //
    // LogBook special stream manipulators
    //
    LogBook &operator<<(LogBook& (*pf)(LogBook&));

    //
    // LogBook standard ostream manipulators
    //
    LogBook &operator<<(std::ostream& (*pf)(std::ostream&));
    LogBook &operator<<(std::ios& (*pf)(std::ios&));
    LogBook &operator<<(std::ios_base& (*pf)(std::ios_base&));

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // LogBook Editing, Access, and Utilities
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Log the given entry into the log book.
     *
     * Any pending log entry remains untouched.
     *
     * \note The alternative method to log an entry is to use the LogBook
     * insertion stream operators.
     *
     * \param strText   Log entry text.
     * \param strMark   Associated log entry bookmark. If empty (""), then no
     *                  bookmark added.
     *
     * \return Returns the new number of entries in the log book.
     */
    size_t logEntry(const std::string strText, const std::string strMark = "");

    /*!
     * \brief Log any pending entry into the log book. 
     *
     * The pending entry and associated bookmark are recorded into log. The
     * pending data are then erased.
     *
     * \note The alternative method to log an entry is to use the LogBook
     * insertion stream operators.
     *
     * \return Returns the new number of entries in the log book.
     */
    size_t logPending();

    /*!
     * \brief Set the pending log text.
     *
     * Any previously pending, unlogged text will be lost.
     *
     * \note The alternative method to log an entry is to use the LogBook
     * insertion stream operators.
     *
     * \param strText   Log entry text.
     */
    void setPendingText(const std::string strText);

    /*!
     * \brief Set the pending bookmark.
     *
     * The pending bookmark is associated with the pending log entry. Any 
     * previous pending, unlogged bookmark will be lost.
     *
     * \note The alternative (and preferred) method to set the pending mark is
     * to use the LogBook insertion stream operators.
     *
     * \param strMark Pending bookmark label.
     */
    void setPendingBookMark(const std::string strMark);

    /*!
     * \brief Set the pending log text and associated bookmark.
     *
     * \param strText   Log entry text.
     * \param strMark   Associated log entry bookmark label.
     */
    void setPending(const std::string strText, const std::string strMark);

    /*!
     * \brief Erase the pending log text.
     */
    void erasePendingText();

    /*!
     * \brief Erase the pending log bookmark.
     */
    void erasePendingBookMark();

    /*!
     * \brief Erase the pending log text and associated bookmark.
     */
    void erasePending();

    /*!
     * \brief Erases all entries from the log book up to the bookmark.
     *
     * If whence is OLDEST, then the entries erased are from the oldest entries
     * up to, but not including, the first encounter of the bookmarked entry.
     *
     * If whence is NEWEST, then the entries erased are from the most recent
     * logged entry up to and including the first encounter of the bookmarked
     * entry.
     *
     * If the bookmark is not found, then no entries are erased.
     *
     * \param strMark   Bookmark search label.
     * \param whence    Whence directive (OLDEST or NEWEST).
     *
     * \return Returns the new number of entries in the log book.
     */
    size_t eraseToMark(const std::string strMark, int whence);

    /*!
     * \brief Erases a given number of entries from the log book.
     *
     * If whence is OLDEST, then the entries are erased starting from the
     * oldest entry.
     *
     * If whence is NEWEST, then the entries are erased starting from the
     * most recent entry.
     *
     * \param uNumEntries   Number of entries to delete. If \>= number of 
     *                      entries, then the book is erased.
     * \param whence        Whence directive (OLDEST or NEWEST).
     *
     * \return Returns the new number of entries in the log book.
     */
    size_t eraseEntries(size_t uNumEntries, int whence);

    /*!
     * \brief Clear the log book and bookmarks, along with any pending entry.
     */
    void clear();

    /*!
     * \brief Get the entry at the bookmark.
     *
     * \param strMark   Bookmark label.
     *
     * \return
     * If the entry exists, reference to log entry is returned. 
     * Otherwise, a "no entry" entry is returned (test with Entry::empty()).
     */
    const Entry &entryAt(const std::string &strMark) const;

    /*!
     * \brief Get the entry at the index.
     *
     * \param index   Log book absolute index.
     *
     * \return
     * If the entry exists, reference to log entry is returned. 
     * Otherwise, a "no entry" entry is returned (test with Entry::empty()).
     */
    const Entry &entryAt(const size_t index) const;

    /*!
     * \brief Get the entry at relative index from whence.
     *
     * \param index     Relative log book index.
     * \param whence    Whence directive (OLDEST or NEWEST).
     *
     * \return
     * If the entry exists, reference to log entry is returned. 
     * Otherwise, a "no entry" entry is returned (test with Entry::empty()).
     */
    const Entry &entryAt(const size_t index, int whence) const;

    /*!
     * \brief Log book array index operator.
     *
     * Get the entry at the index.
     *
     * \param index   Log book absolute index.
     *
     * \return
     * If the entry exists, reference to log entry is returned. 
     * Otherwise, a "no entry" entry is returned (test with Entry::empty()).
     */
    const Entry &operator[](const size_t index) const;
    
    /*!
     * \brief Log book array mark operator.
     *
     * \param strMark   Bookmark label.
     *
     * \return
     * If the entry exists, reference to log entry is returned. 
     * Otherwise, a "no entry" entry is returned (test with Entry::empty()).
     */
    const Entry &operator[](const std::string &strMark) const;

    /*!
     * \brief Get the last (latest) log entry.
     *
     * \return
     * If the entry exists, reference to the last log entry is returned. 
     * Otherwise, a "no entry" entry is returned (test with Entry::empty()).
     */
    const Entry &lastEntry() const;

    /*!
     * \brief Get the entry text at the bookmark.
     *
     * \param strMark   Bookmark label.
     *
     * \return
     * If the entry exists, reference to the log entry text is returned. 
     * Otherwise, an empty string is returned.
     */
    const std::string &textAt(const std::string &strMark) const
    {
      return entryAt(strMark).m_strText;
    }

    /*!
     * \brief Get the entry text at the index.
     *
     * \param index   Log book absolute index.
     *
     * \return
     * If the entry exists, reference to the log entry text is returned. 
     * Otherwise, an empty string is returned.
     */
    const std::string &textAt(const size_t index) const
    {
      return entryAt(index).m_strText;
    }

    /*!
     * \brief Get the entry text at relative index from whence.
     *
     * \param index     Relative log book index.
     * \param whence    Whence directive (OLDEST or NEWEST).
     *
     * \return
     * If the entry exists, reference to the log entry text is returned. 
     * Otherwise, an empty string is returned.
     */
    const std::string &textAt(const size_t index, int whence) const
    {
      return entryAt(index, whence).m_strText;
    }

    /*!
     * \brief Get the last (latest) log entry text.
     *
     * \return
     * If the entry exists, reference to the last log entry is returned. 
     * Otherwise, a "no entry" entry is returned (test with Entry::empty()).
     */
    const std::string &lastText() const
    {
      return lastEntry().m_strText;
    }

    /*!
     * \brief Get a sorted list of bookmark labels.
     *
     * If whence is OLDEST, then sort from oldest to newest.
     * If whence is NEWEST, then sort from most recent to oldest.
     *
     * \param [out] list  List of sorted bookmark labels.
     * \param whence      Whence directive (OLDEST or NEWEST).
     *
     * \return Number of bookmarks in list.
     */
    size_t getBookMarks(BookMarkList &list, int whence) const;

    /*!
     * \brief Test if bookmark exists in log book.
     *
     * \param strMark   Bookmark label.
     *
     * \return Returns true or false.
     */
    bool hasBookMark(const std::string &strMark) const
    {
      return findMark(strMark) != m_book.end();
    }

    /*!
     * \brief Get the name of the log book.
     *
     * \return String.
     */
    const std::string &getName() const
    {
      return m_strName;
    }

    /*!
     * \brief Set the name of the log book.
     *
     * \param strName String.
     */
    void setName(const std::string &strName)
    {
      m_strName = strName;
    }

    /*!
     * \brief Return the maximum number entries the log book can hold.
     *
     * \return Size.
     */
    const size_t max_size() const
    {
      return m_uMaxEntries;
    }

    /*!
     * \brief Return the number of logged entries in the log book.
     *
     * \return Number of entries.
     */
    size_t size() const
    {
      return (size_t)m_book.size();
    }

    /*!
     * \brief Resize maximum size of log book.
     *
     * If the new size is smaller, then entries may be deleted.
     *
     * \param uMaxEntries   Maximum number of entries in log book.
     */
    void resize(size_t uMaxEntries);

    /*!
     * \brief Return the total number of entries ever added during log book's
     * life.
     *
     * \return Total number.
     */
    size_t numOfTotalEver() const
    {
      return m_uTotalEver;
    }

    /*!
     * \brief Get the current formatting flags.
     *
     * \return Bit list of or'ed flags.
     */
    unsigned getFlags() const
    {
      return m_uFlags;
    }

    /*!
     * \brief Set the current formatting flags.
     *
     * \param uFlags   Flags to set.
     *
     * \return New bit list of or'ed flags.
     */
    unsigned setFlags(const unsigned uFlags)
    {
      m_uFlags = uFlags;
      return m_uFlags;
    }

    /*!
     * \brief Or new flags into the current formatting flags.
     *
     * \param uFlags   Flags to or into bit list of flags.
     *
     * \return New bit list of or'ed flags.
     */
    unsigned orFlags(const unsigned uFlags)
    {
      m_uFlags |= uFlags;
      return m_uFlags;
    }

    /*!
     * \brief Apply one's Complement and And into the current formatting flags.
     *
     * \param uFlags   Flags to unset.
     *
     * \return New bit list of or'ed flags.
     */
    unsigned compAndFlags(const unsigned uFlags)
    {
      m_uFlags &= ~uFlags;
      return m_uFlags;
    }

    /*!
     * \brief Clear alll current formatting flags.
     */
    void clearFlags()
    {
      m_uFlags = FlagNone;
    }

    /*!
     * \brief Generate a unique bookmark label.
     *
     * \return Label string.
     */
    std::string makeBookMarkLabel();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // LogBook Output Stream Operators
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Print the entire log book to the output stream.
     *
     * The output is controlled by the current formatting flag bits.
     *
     * \verbatim
     * LogBook
     * entry_0    | ^   <-- oldest end point
     * entry_1    | |
     *   ...      | |
     * entry_n-1  v |   <-- newest end point
     *             \ \
     *              \ \
     *               \ reverse order
     *                \
     *                 forward order
     * \endverbatim
     *
     * \param os  Output stream.
     *
     * \return Reference to output stream.
     */
    std::ostream &printLog(std::ostream &os) const;

    /*!
     * \brief Print the log book entires between the bookmark and the specified
     * end point to the output stream.
     *
     * The output is controlled by the current formatting flag bits.
     *
     * If the opposite end point to the mark is the oldest entry, then the
     * bookmark entry is excluded. Otherwise it is included.
     *
     * \verbatim
     * LogBook 
     * entry_0    | ^   <-- oldest end point
     * entry_1    | |
     *   ...      | |
     * entry_k-1  v |   <-- sans mark end point
     *
     * entry_k    | ^   <-- mark end point
     * entry_k+1  | |
     *   ...      | |
     * entry_n-1  v |   <-- newest end point
     *             \ \
     *              \ \
     *               \ reverse order
     *                \
     *                 forward order
     * \endverbatim
     *
     * \param os        Output stream.
     * \param strMark   Bookmark search label.
     * \param endpt     Opposite end point to mark (OLDEST or NEWEST).
     *
     * \return Reference to output stream.
     */
    std::ostream &printToMark(std::ostream      &os,
                              const std::string strMark,
                              int               endpt) const;

    //std::ostream &printEntriesEarlierThan(std::ostream &os, int t);

    friend std::ostream &operator<<(std::ostream &os, const LogBook &log);

  protected:
    //
    // Convenience Types
    //
    typedef std::deque<Entry>               BookDeq;    ///< book type
    typedef BookDeq::iterator               BookIter;   ///< book iterator
    typedef BookDeq::const_iterator         BookCIter;  ///< book const iterator
    typedef BookDeq::reverse_iterator       BookRIter;  ///< reverse book iter
    typedef BookDeq::const_reverse_iterator BookCRIter; ///< const rev book iter

    //
    // Data
    //
    std::string       m_strName;        ///< name of log 
    size_t            m_uMaxEntries;    ///< maximum number of log entries
    size_t            m_uMaxEntryLen;   ///< maximum entry length
    size_t            m_uTotalEver;     ///< total entries added during lifetime
    unsigned          m_uFlags;         ///< formatting flags
    bool              m_bWarnThrottle;  ///< do [not] throttle warnings
    std::string       m_strMark;        ///< pending bookmark label
    std::stringstream m_ssText;         ///< pending text stream
    BookDeq           m_book;           ///< the log book

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Support
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Limit size of log book to defined limit.
     */
    void limitSize();

    /*!
     * \brief Flush the pending text stream.
     *
     * The bytes go to a better place, not the log.
     */
    void ssflush();

    /*!
     * \brief Check the length of the pending entry.
     *
     * \param uEstLen   Estimated length of the new object text to be added.
     *
     * \return Returns true if pending entry within bounds, false otherwise.
     */
    bool checkLen(size_t uEstLen = 1);

    /*!
     * \brief Find position it log book of the bookmarked entry.
     *
     * The search starts at the oldest entry.
     *
     * \param strMark   Bookmark search label.
     *
     * \return
     * If the bookmark entry is found, the position is returned.
     * Otherwise, book end() is returned.
     */
     BookIter findMark(const std::string &strMark);

    /*!
     * \brief Find position it log book of the bookmarked entry.
     *
     * Constant version.
     *
     * The search starts at the oldest entry.
     *
     * \param strMark   Bookmark search label.
     *
     * \return
     * If the bookmark entry is found, the position is returned.
     * Otherwise, book end() is returned.
     */
     BookCIter findMark(const std::string &strMark) const;

    /*!
     * \brief Sort bookmarks from oldest to newest.
     *
     * \param [out] sorted  List of sorted bookmarks.
     */
    void sortMarks(BookMarkList &sorted) const;

    /*!
     * \brief Reverse sort bookmarks from newest to oldest.
     *
     * \param [out] sorted  List of sorted bookmarks.
     */
    void rsortMarks(BookMarkList &sorted) const;

    /*!
     * \brief Copy log book verbatim to this log book.
     *
     * \param src   Source log book.
     */
    void copy(const LogBook &src);

    /*!
     * \brief Print log book from index 0 to index 1, inclusive.
     *
     * The output is controlled by the current formatting flag bits.
     *
     * \param os      Output stream.
     * \param index0  Starting log book index.
     * \param index1  Ending log book index.
     */
    void prt(std::ostream &os, size_t index0, size_t index1) const;

    /*!
     * \brief Reverse print log book from index 1 to index 0, inclusive.
     *
     * The output is controlled by the current formatting flag bits.
     *
     * \param os      Output stream.
     * \param index0  Starting log book index.
     * \param index1  Ending log book index.
     */
    void rprt(std::ostream &os, size_t index0, size_t index1) const;

  }; // class LogBook


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Output Stream Manipulators
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Stream insertion operator.
   *
   * The entire log book is inserted. Formatting is controlled by the current
   * flags settings.
   * 
   * \param os  Output stream.
   * \param lob Log book.
   *
   * \return Reference to output stream.
   */
  extern std::ostream &operator<<(std::ostream &os, const LogBook &log);


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // LogBook Stream Manipulators
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief LogBook end-of-entry stream manipulator.
   *
   * The pending entry is copied into the log and then flushed of data, ready
   * to begin recording new entry.
   *
   * ~~~~~~{.cxx}
   * LogBook log;
   *
   * log << eoe;
   * log << 5 << " was my happiest age." << eoe;
   * ~~~~~~
   *
   * \param log   LogBook reference.
   *
   * \return Returns reference to LogBook.
   */
  extern LogBook &eoe(LogBook &log);

  /*!
   * \brief LogBook bookmark stream manipulator.
   *
   * Bookmark the pending entry. The bookmark string is auto-generated.
   * The bookmark follows the the pending entry as it is logged and aged.
   *
   * ~~~~~~{.cxx}
   * LogBook log;
   *
   * log << bookmark;
   * log << "black birds are cawing" << eoe;
   * log << bookmark << " remember this to never forget." << eoe;
   * ~~~~~~
   *
   * \param log   LogBook reference.
   *
   * \return Returns reference to LogBook.
   */
  extern LogBook &bookmark(LogBook &log);

  /*!
   * \brief LogBook parametric bookmark stream manipulator structure.
   */
  struct lbmanip_bm_
  {
    std::string m_strMark;  ///< bookmark label
  };

  /*!
   * \brief LogBook bookmark parametric manipulator function. 
   *
   * \param str   Bookmark label.
   *
   * \return Parametric structure.
   */
  extern lbmanip_bm_ bookmark(const std::string str);

  /*!
   * \brief LogBook bookmark parametric manipulator stream operators.
   *
   * Bookmark the pending entry. The bookmark follows the the pending entry as
   * it is logged and aged.
   *
   * ~~~~~~{.cxx}
   * LogBook log;
   *
   * log << bookmark("GTO") << "Muscle cars of yore." << eoe;
   * ~~~~~~
   *
   * \param log   LogBook reference.
   * \param f     Bookmark parametric type.
   *
   * \return Returns reference to LogBook.
   */
  extern LogBook &operator<<(LogBook &log, lbmanip_bm_ f);

  /*!
   * \brief LogBook parametric formatting flags manipulator structure.
   */
  struct lbmanip_fg_
  {
    unsigned m_uFlags;
  };

  /*!
   * \brief LogBook formatting flags parametric manipulator function. 
   *
   * \param str   Bookmark label.
   *
   * \return Parametric structure.
   */
  extern lbmanip_fg_ setflags(const unsigned flags);

  /*!
   * \brief LogBook formatting flags parametric manipulator stream operators.
   *
   * Modify formatting behavior with flags.
   *
   * ~~~~~~{.cxx}
   * LogBook log;
   *
   * // log data
   *
   * unsigned flags = LogBook::FlagONum | LogBook::FlagOMark;
   *
   * log << setflags(flags);
   * std::cout << log;
   * ~~~~~~
   *
   * \param log   LogBook reference.
   * \param f     Flags parametric type.
   *
   * \return Returns reference to LogBook.
   */
  extern LogBook &operator<<(LogBook &log, lbmanip_fg_ f);

} // namespace rnr

#endif // _LOG_BOOK_H
