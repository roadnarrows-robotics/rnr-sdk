////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// File:      example_dlist.c
//
/*! \file
 *
 * $LastChangedDate: 2010-03-24 10:19:36 -0600 (Wed, 24 Mar 2010) $
 * $Rev: 307 $
 *
 * \brief Example of a librnr "derived" DListVoid doubly-linked lists.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2007-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"

#include "example_dlist.h"

/*!
 * \brief Animal catalog of the world.
 */
const char *AnimalCatalog[][2] =
{
  {"elephas maximus",             "Asiatic elephant"},
  {"loxodonta africana",          "African bush elephant"},  
  {"python sebae",                "African rock python"}, 
  {"alligator mississippiensis",  "American alligator"},
  {"ramphastos toco",             "Toco toucan"},
  {"lasiorhinus latifrons",       "southern hairy-nosed wombat"},
  {"rhinoceros sondaicus",        "Javan rhinoceros"},
  {NULL, NULL}
};

/*!
 * \brief Zoo we will "buy".
 */
static char *FlatEarthZooDeal[][2] =
{
  {"Shoes",     "alligator mississippiensis"},
  {"Fluffy",    "loxodonta africana"},
  {"Snuggles",  "python sebae"},
  {"Wild Bill", "ramphastos toco"},
  {"The Prez",  "lasiorhinus latifrons"},
  {NULL, NULL}
};

static Zid = 0;     ///< zoo id

/*!
 * \brief Allocate new zoo addition.
 *
 * \param sSciName    Scientific name.
 * \param sPetName    Pet name.
 *
 * \return New zoo node.
 */
static Zoo_T *NewZooAnimal(const char *sSciName, const char *sPetName)
{
  const char  **sCat;
  Zoo_T       *pZooAni;

  pZooAni             = NEW(Zoo_T);
  pZooAni->m_nZid     = Zid++;
  pZooAni->m_sPetName = new_strdup(sPetName);

  sCat = AnimalCatalog[0];

  // find animal species in catalog
  for(sCat=AnimalCatalog[0]; sCat[0]!=NULL; sCat++)
  {
    if( !strcmp(sCat[0], sSciName) )
    {
      // assign type
      pZooAni->m_sSciName = sCat[0];
      pZooAni->m_sComName = sCat[1];
      return pZooAni;
    }
  }

  // unknown animal type
  pZooAni->m_sSciName = "???";
  pZooAni->m_sComName = "???";

  return pZooAni;
}

/*!
 * \brief Example main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \par Exit Status:
 * Program exits with 0 success, \h_gt 0 on failure.
 */
int main(int argc, char *argv[])
{
  DListZoo_T     *pMyZoo;
  Zoo_T          *pMyAni;
  Zoo_T           zooMatch;
  DListZooIter_T  zooIter;
  int             i;

  // buy empty zoo
  pMyZoo = DListZooNew(DListZooDataCmp, DListZooDataDelete);

  printf("I bought an empty zoo.\n");
  DListZooPrint(pMyZoo, DListZooDataPrint, stdout);
  
  // purchase animals from zoo in FlatEarth, NE.
  for(i=0; FlatEarthZooDeal[i][0]!=NULL; ++i)
  {
    pMyAni = NewZooAnimal(FlatEarthZooDeal[i][1], FlatEarthZooDeal[i][0]);
    DListZooAppend(pMyZoo, pMyAni);
  }
  
  printf("\nI bought some animals from the zoo in Flat Earth, NE.\n");
  DListZooPrint(pMyZoo, DListZooDataPrint, stdout);

  // Fluffy dies
  zooMatch.m_sPetName = "Fluffy";
  DListZooDeleteNode(pMyZoo, DListZooFindNode(pMyZoo, &zooMatch));

  printf("\nAlas, Fluffy died.\n");
  DListZooPrint(pMyZoo, DListZooDataPrint, stdout);
  
  // purchase "eMouse" from kid down the block.
  pMyAni = NewZooAnimal("elephas maximus", "eMouse");
  DListZooPrepend(pMyZoo, pMyAni);

  printf("\nBut I got a new primo 'phant from the kid down the block.\n");
  DListZooPrint(pMyZoo, DListZooDataPrint, stdout);
  
  // give vitamin shots to the beasts
  printf("\nTime to give my menagerie their yearly vitamin shots.\n");
  for(pMyAni=DListZooIterDataFirst(pMyZoo, &zooIter);
      pMyAni!=NULL;
      pMyAni=DListZooIterDataNext(&zooIter))
  {
    printf("  injected %s\n", pMyAni->m_sPetName);
  }

  // intelligent procreation
  pMyAni = NewZooAnimal("lasiorhinus latifrons", "wittlewomby1");
  DListZooInsert(pMyZoo, 4, pMyAni);
  pMyAni = NewZooAnimal("lasiorhinus latifrons", "wittlewomby2");
  DListZooInsert(pMyZoo, 4, pMyAni);

  printf("\nI guess \"The Prez\" wasn't male. "
         "Pass out the bubble gum cigars!\n");
  DListZooPrint(pMyZoo, DListZooDataPrint, stdout);

  // let my animals go free
  DListZooDeleteAllNodes(pMyZoo);

  printf("\nI decided to let my animals go free - in Alaska.\n");
  DListZooPrint(pMyZoo, DListZooDataPrint, stdout);

  DListZooDelete(pMyZoo);

  printf("\nGoodbye.\n");

  return 0;
}
