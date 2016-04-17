////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      shm.c
//
/*! \file
 *
 * $LastChangedDate: 2013-02-07 16:51:44 -0700 (Thu, 07 Feb 2013) $
 * $Rev: 2675 $
 *
 * \brief Shared memory routines.
 *
 * \li Basic convenience funtions.
 * \li Shared memory implementation of the POSIX thread mutex.
 * With shared memory, the mutex may span mutilple applications, not just
 * threads within one application.
 * \li Other functions as needed.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/shm.h"


//------------------------------------------------------------------------------
// Basic Shared Memory Functions
//------------------------------------------------------------------------------

int shm_open(key_t                key,
             size_t               size,
             shm_mem_init_func_t  mem_init,
             shm_t               *pshm)
{
  int   perm = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH;
  int   shmId;
  void *addr;
  int   tmp;

  if( pshm == NULL )
  {
    LOGERROR("shm_t structure is NULL.");
    errno = EINVAL;
    return RC_ERROR;
  }

  // zzzsst
  pshm->m_shmKey  = 0;
  pshm->m_shmId   = 0;
  pshm->m_shmSize = 0;
  pshm->m_shmAddr = NULL;

  //
  // Try to create shared memory with the given key, returning the identifier.
  // Fails if already created.
  //
  shmId = shmget(key, size, IPC_CREAT | IPC_EXCL | perm);
  
  //
  // Created.
  //
  if( shmId >= 0 )
  {
    // attach shared memory to process address space
    addr = shmat(shmId, NULL, 0);
    
    if( addr == (void *)(-1) )
    {
      LOGSYSERROR("shmgat(%d, NULL, 0)",  shmId);
      tmp = errno;
      shmctl(shmId, IPC_RMID, NULL);
      errno = tmp;
      return RC_ERROR;
    }

    //
    // Initialize the shared memory. Only the caller that creates the shared
    // memory, initializes it, preventing race condition between caller
    // threads or applications.
    //
    if( mem_init != NULL )
    {
      mem_init(key, addr, size);
    }
  }

  // 
  // Already created, try to attach.
  //
  else if( errno == EEXIST )
  {
    // already exist, get the identifier
    shmId = shmget(key, size, perm);
      
    if( shmId < 0 )
    {
      LOGSYSERROR("shmget(%d=0x%x, %zu, ...)", key, key, size);
      return RC_ERROR;
    }

    // attach shared memory to process address space
    addr = shmat(shmId, NULL, 0);
    
    if( addr == (void *)(-1) )
    {
      LOGSYSERROR("shmgat(%d, NULL, 0)",  shmId);
      tmp = errno;
      shmctl(shmId, IPC_RMID, NULL);
      errno = tmp;
      return RC_ERROR;
    }
  }

  // 
  // Failed to create or get shared memory.
  //
  else
  {
    LOGSYSERROR("shmget(%d=0x%x, %zu, ...)", key, key, size);
    return RC_ERROR;
  }

  // set values
  pshm->m_shmKey  = key;
  pshm->m_shmId   = shmId;
  pshm->m_shmSize = size;
  pshm->m_shmAddr = addr;

  LOGDIAG3("Opened shared memory (id=%d, size=%zu).", key, size);

  return OK;
}

int shm_close(shm_t *pshm)
{
  if( pshm == NULL )
  {
    LOGERROR("shm_t structure is NULL.");
    errno = EINVAL;
    return RC_ERROR;
  }

  // mark shared memory to be detroyed
  else if( shmctl(pshm->m_shmId, IPC_RMID, NULL) < 0 )
  {
    LOGSYSERROR("shmctl(%d, ...)", pshm->m_shmId);
    return RC_ERROR;
  }

  // detach shared memory from application address space
  else if( shmdt(pshm->m_shmAddr) < 0 )
  {
    LOGSYSERROR("shmdt(%p)", pshm->m_shmAddr);
    return RC_ERROR;
  }

  pshm->m_shmKey  = 0;
  pshm->m_shmId   = 0;
  pshm->m_shmSize = 0;
  pshm->m_shmAddr = NULL;

  LOGDIAG3("Closed shared memory (id=%d).", pshm->m_shmId);

  return OK;
}


//------------------------------------------------------------------------------
// Mutex Shared Memory Functions
//------------------------------------------------------------------------------

/*!
 * \brief Create the share memory mutex.
 *
 * \param key   Shared memory key.
 * \param addr  Starting address of attached shared memory.
 * \param size  Shared memory size.
 */
static void shm_mutex_create(key_t key, void *addr, size_t size)
{
  pthread_mutex_t      *pmutex = (pthread_mutex_t *)addr;
  pthread_mutexattr_t   mutexAttr;

  pthread_mutexattr_init(&mutexAttr);

  //
  // Set the attribute to share between applications.
  //
  if(pthread_mutexattr_setpshared(&mutexAttr, PTHREAD_PROCESS_SHARED) < 0)
  {
    LOGSYSERROR("pthread_mutexattr_setpshared()");
  }

#ifndef ARCH_overo
  //
  // Set mutex robust attribute. If the owning thread or application terminates,
  // the next thread/application acquiring the mutex is notified by the 
  // return value EOWNERDEAD.
  //
  else if( pthread_mutexattr_setrobust(&mutexAttr, PTHREAD_MUTEX_ROBUST) < 0 )
  {
    LOGSYSERROR("pthread_mutexattr_setrobust()");
  }

  //
  // Allow recursive locks by the owning thread or application.
  //
  else if( pthread_mutexattr_settype(&mutexAttr, PTHREAD_MUTEX_RECURSIVE) < 0 )
  {
    LOGSYSERROR("pthread_mutexattr_settype()");
  }
#endif // ARCH_overo

  //
  // Initialize the pthread mutex with the give attributes
  //
  else if( pthread_mutex_init(pmutex, &mutexAttr) < 0 )
  {
    LOGSYSERROR("pthread_mutex_init()");
  }

  //
  // Success.
  //
  else
  {
    LOGDIAG4("Shared memory mutex created.");
  }

  pthread_mutexattr_destroy(&mutexAttr);
}

int shm_mutex_init(key_t key, shm_mutex_t *pshmmutex)
{
  size_t  size = sizeof(pthread_mutex_t);

  if( pshmmutex == NULL )
  {
    LOGERROR("shm_mutex_t structure is NULL.");
    errno = EINVAL;
    return RC_ERROR;
  }

  // create/attach to shared memory mutex
  if( shm_open(key, size, shm_mutex_create, pshmmutex) < 0 )
  {
    LOGSYSERROR("shm_mutex_init(%d=0x%x, ...)", key, key);
    return RC_ERROR;
  }

  LOGDIAG3("Shared memory mutex initialized (key=%d=0x%x, ...)", key, key);

  return OK;
}

int shm_mutex_destroy(shm_mutex_t *pshmmutex)
{
  return shm_close(pshmmutex);
}

int shm_mutex_lock(shm_mutex_t *pshmmutex)
{
  pthread_mutex_t  *pmutex;
  int               ntries;
  int               rc;

  if( pshmmutex == NULL )
  {
    LOGERROR("shm_mutex_t structure is NULL.");
    errno = EINVAL;
    return errno;
  }

  pmutex = (pthread_mutex_t *)(pshmmutex->m_shmAddr);

  for(ntries=0; ntries<SHM_MUTEX_N_TRIES; ++ntries)
  {
    if( (rc = pthread_mutex_lock(pmutex)) == 0 )
    {
      return OK;
    }

    switch( rc )
    {
      case EOWNERDEAD:  // owner of mutex went to a better place
#ifndef ARCH_overo
        pthread_mutex_consistent(pmutex);
#endif // ARCH_overo
        return OK;
      case EINVAL:      // mutex not initialized by creating thread/application
        usleep(SHM_MUTEX_T_TRIES);
        break;
      case EAGAIN:      // maximum recursive locks exceeded
        usleep(SHM_MUTEX_T_TRIES);
        break;
      default:          // other error
        errno = rc;
        return rc;
    }
  }

  errno = rc;
  return rc;
}

int shm_mutex_trylock(shm_mutex_t *pshmmutex)
{
  pthread_mutex_t  *pmutex;
  int               ntries;
  int               rc;

  if( pshmmutex == NULL )
  {
    LOGERROR("shm_mutex_t structure is NULL.");
    errno = EINVAL;
    return errno;
  }

  pmutex = (pthread_mutex_t *)(pshmmutex->m_shmAddr);

  for(ntries=0; ntries<SHM_MUTEX_N_TRIES; ++ntries)
  {
    if( (rc = pthread_mutex_trylock(pmutex)) == 0 )
    {
      return OK;
    }

    switch( rc )
    {
      case EOWNERDEAD:  // owner of mutex went to a better place
#ifndef ARCH_overo
        pthread_mutex_consistent(pmutex);
#endif // ARCH_overo
        return OK;
      case EINVAL:      // mutex not initialized by creating thread/application
        usleep(SHM_MUTEX_T_TRIES);
        break;
      case EAGAIN:      // maximum recursive locks exceeded
        usleep(SHM_MUTEX_T_TRIES);
        break;
      case EBUSY:       // already locked
      default:          // other error
        errno = rc;
        return rc;
    }
  }

  errno = rc;
  return rc;
}

int shm_mutex_unlock(shm_mutex_t *pshmmutex)
{
  pthread_mutex_t  *pmutex;

  if( pshmmutex == NULL )
  {
    LOGERROR("shm_mutex_t structure is NULL.");
    errno = EINVAL;
    return errno;
  }

  pmutex = (pthread_mutex_t *)(pshmmutex->m_shmAddr);

  return pthread_mutex_unlock(pmutex);
}

#if 0
static pthread_mutex_t * GlobalMutex;
static char * myLine = PROG1;
static int shmid;

key_t key = 0x11d7;

void * getSharedMemPtr(key_t key)
{
  FILE * fp;

  shmid = shmget(key,
                 sizeof(pthread_mutex_t) + sizeof(pthread_once_t),
                 IPC_CREAT | IPC_EXCL | 0x1b6);
  
  if( shmid < 0 )
  {
    switch( errno )
    {
      case EEXIST:
    printf("Could not create shared mem.\n");
      printf("Will attempt to find it.\n");
      
      shmid = shmget(key,
          sizeof(pthread_mutex_t) + sizeof(pthread_once_t),
          0x1b6);
      break;
    }
      
      if(0 > shmid)
  {
    printf("\tCouldnt find it either\n");
  }

      else
  {
    
    fp = fopen("MutextestFile.txt", "r+");
          fclose(fp);  
    
    myLine = PROG2;
    printf("\tFound shared memory");
    void * const poSharedMem = shmat(shmid, NULL, 0);
    
    if(((void *)-1) == poSharedMem)
      {
        printf("Could not attatch shared memory to address space.\n");
        return  NULL;
      }
    else
      {
        //printf("Shared memory attached and marked for deletion.\n");
        //shmctl(shmid, IPC_RMID, NULL);
        printf("Shared memory attached\n");
        return poSharedMem;       
      }
  }
    } 
  else
    {
      fp = fopen("MutextestFile.txt", "w+");
      fclose(fp);

      myLine = PROG1;
      printf("Shared memory created.\n");
      
      void * const poSharedMem = shmat(shmid, NULL, 0);
      
      if(((void *)-1) == poSharedMem)
  {
    printf("Could not attatch shared memory to address space.\n");
    return  NULL;
  }
      else
  {
    //printf("Shared memory attached and marked for deletion.\n");
    //shmctl(shmid, IPC_RMID, NULL);
    printf("Shared memory attached\n");
    return  poSharedMem;
  }
    }

  return  NULL;
}

int detatchFromSharedMem(void * poSharedMem)
{
  printf("Marking shared memory for destruction and detaching from it.\n");
  shmctl(shmid, IPC_RMID, NULL);
  return shmdt(poSharedMem);
}


int main(void)
{

  FILE * fp;
 
  void * poSharedMem = getSharedMemPtr();
  GlobalMutex = (pthread_mutex_t *)poSharedMem;

  pthread_once_t * pOnce = (pthread_once_t *)( ((char *)poSharedMem) + sizeof(pthread_mutex_t) );

  pthread_once(pOnce, sharedMemoryMutexInit);
  
  if (GlobalMutex == NULL)
    {
      return 0;
    }

  int mutexLockResult;
  
  for (int i =0; i < 100000; i ++)
    {
      mutexLockResult = pthread_mutex_lock(GlobalMutex);
      if (0 == mutexLockResult)
  {
    fp = fopen("MutextestFile.txt", "r+");
    if (NULL != fp)
      {
        fseek(fp, 0, SEEK_END);
        fprintf(fp, "%4d - ", i+1);
        fprintf(fp, myLine);
        fclose(fp);
        fp = NULL;
      }
    pthread_mutex_unlock(GlobalMutex);
  }
      else if (EOWNERDEAD == mutexLockResult)
  {
    fp = fopen("MutextestFile.txt", "r+");
    if (NULL != fp)
      {
        fseek(fp, 0, SEEK_END);
        fprintf(fp, "%4d - ", i+1);
        fprintf(fp, myLine);
        fclose(fp);
        fp = NULL;
      }
    pthread_mutex_consistent_np(GlobalMutex);
    pthread_mutex_unlock(GlobalMutex);
  }
    }
  
  
  if(NULL != GlobalMutex)
    {
      detatchFromSharedMem(GlobalMutex);
    }
}
#endif
