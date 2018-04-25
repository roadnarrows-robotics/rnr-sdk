////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Shared memory interface.
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/shm.h}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \pkgcopyright{2013-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
 *
 * \license{MIT}
 *
 * \EulaBegin
 * See the README and EULA files for any copyright and licensing information.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_SHM_H
#define _RNR_SHM_H

#include <sys/ipc.h>
#include <sys/shm.h>

#include "rnr/rnrconfig.h"

C_DECLS_BEGIN

/*!
 * \brief Shared memory structure type.
 */
typedef struct
{
  key_t     m_shmKey;     ///< shared memory key
  int       m_shmId;      ///< shared memory identifier
  size_t    m_shmSize;    ///< shared memory size in bytes
  void     *m_shmAddr;    ///< starting addressed of attached shared memory
} shm_t;

/*!
 * \brief Shared memory pthread mutex structure type.
 */
typedef shm_t shm_mutex_t;

/*!
 * \brief User-supplied shared memory initialization function.
 *
 * \param key   Shared memory key.
 * \param addr  Starting address of attached shared memory.
 * \param size  Shared memory size.
 */
typedef void (*shm_mem_init_func_t)(key_t key, void *add, size_t size);


#ifndef SHM_MUTEX_N_TRIES
#define SHM_MUTEX_N_TRIES   3     ///< maximum number of tries to acquire lock
#endif

#ifndef SHM_MUTEX_T_TRIES
#define SHM_MUTEX_T_TRIES   50000 ///< usec time between tries to acquire lock
#endif

/*!
 * \brief Open shared memory segement.
 * 
 * If the segment, associated by key, does not exist, then the shared memory is
 * created and, optionally, initialized.
 *
 * The shared memory segment is attached to the application address space.
 *
 * \param key             Shared memory key.
 * \param size            Shared memory size.
 * \param mem_init        Optional memory initializer function. Set to NULL for
 *                        no initialization.
 * \param [in,out] pshm   Pointer to shared memory structure defined in caller's
 *                        space.
 *
 * \return On success, returns \ref OK(0).\n
 * On failure, errno is set and \ref RC_ERROR(-1) is returned.
 */
extern int shm_open(key_t                key,
             size_t               size,
             shm_mem_init_func_t  mem_init,
             shm_t               *pshm);

/*!
 * \brief Close shared memory segement.
 * 
 * The segment is marked for deletion, and is actually destroy after the last
 * process detaches.
 * 
 * \param [in,out] pshm   Pointer to shared memory structure defined in caller's
 *                        space.
 *
 * \return On success, returns \ref OK(0).\n
 * On failure, errno is set and \ref RC_ERROR(-1) is returned.
 */
extern int shm_close(shm_t *pshm);

/*!
 * \brief Create and initialize a shared memory mutex.
 * 
 * \param key                 Shared memory key.
 * \param [in,out] pshmmutex  Pointer to shared memory mutext structure defined
 *                            in caller's space.
 *
 * \return On success, returns 0.\n
 * On failure, errno is set and returned.
 */
extern int shm_mutex_init(key_t key, shm_mutex_t *pshmmutex);

/*!
 * \brief Destroy a shared memory mutex.
 * 
 * The mutex is actually only destroyed when the last process attached to this
 * mutex calls destroy.
 *
 * \param [in,out] pshmmutex  Pointer to shared memory mutext structure defined
 *                            in caller's space.
 *
 * \return On success, returns 0.\n
 * On failure, errno is set and returned.
 */
extern int shm_mutex_destroy(shm_mutex_t *pshmmutex);

/*!
 * \brief Lock the mutex.
 * 
 * \param [in] pshmmutex  Pointer to shared memory mutext structure defined
 *                        in caller's space.
 *
 * \return On success, returns 0.\n
 * On failure, errno is set and returned.
 */
extern int shm_mutex_lock(shm_mutex_t *pshmmutex);

/*!
 * \brief Try to lock the mutex.
 * 
 * \param [in] pshmmutex  Pointer to shared memory mutext structure defined
 *                        in caller's space.
 *
 * \return On success, returns 0.\n
 * If the mutex is already locked, EBUSY is returned.\n
 * Otherwise, on failure, errno is set and returned.
 */
extern int shm_mutex_trylock(shm_mutex_t *pshmmutex);

/*!
 * \brief Unlock a lock the mutex.
 * 
 * \param [in] pshmmutex  Pointer to shared memory mutext structure defined
 *                        in caller's space.
 *
 * \return On success, returns 0.\n
 * On failure, errno is set and returned.
 */
extern int shm_mutex_unlock(shm_mutex_t *pshmmutex);

C_DECLS_END


#endif // _RNR_SHM_H

