//
//  IntelLucyRxPool.cpp
//  IntelLucy
//
//  Created by Laura Müller on 26.10.25.
//

#include "IntelLucyRxPool.hpp"

OSDefineMetaClassAndStructors(IntelLucyRxPool, OSObject);

#define super OSObject

bool IntelLucyRxPool::init()
{
    return true;
}

void IntelLucyRxPool::free()
{
    if (cPktHead) {
        mbuf_freem_list(cPktHead);
        cPktHead = cPktTail = NULL;
        cPktNum = 0;
    }
    if (mPktHead) {
        mbuf_freem_list(mPktHead);
        mPktHead = mPktTail = NULL;
        mPktNum = 0;
    }
    super::free();
}

bool IntelLucyRxPool::initWithCapacity(UInt32 mbufCapacity,
                                       UInt32 clustCapacity,
                                       UInt32 size)
{
    mbuf_t m;
    UInt32 i;
    errno_t err;
    unsigned int chunks;
    bool result = false;
    
    if ((mbufCapacity > 0) && (clustCapacity > 0)) {
        cPktHead = cPktTail = NULL;
        mPktHead = mPktTail = NULL;
        cCapacity = clustCapacity;
        mCapacity = mbufCapacity;
        cPktNum = 0;
        mPktNum = 0;
        cSize = size;
        maxCopySize = mbuf_get_mhlen();
        
        for (i = 0; i < mbufCapacity; i++) {
            chunks = 1;
            err = mbuf_allocpacket(MBUF_WAITOK, maxCopySize, &chunks, &m);

            if (err)
                goto fail_mbuf;

            if (mPktHead) {
                mbuf_setnext(mPktTail, m);
                mPktTail = m;
                mPktNum++;
            } else {
                mPktHead = mPktTail = m;
                mPktNum = 1;
            }
        }
        for (i = 0; i < clustCapacity; i++) {
            chunks = 1;
            err = mbuf_allocpacket(MBUF_WAITOK, cSize, &chunks, &m);

            if (err)
                goto fail_cluster;

            if (cPktHead) {
                mbuf_setnext(cPktTail, m);
                cPktTail = m;
                cPktNum++;
            } else {
                cPktHead = cPktTail = m;
                cPktNum = 1;
            }
        }
        result = true;
    }
done:
    return result;
    
fail_cluster:
    if (cPktHead) {
        mbuf_freem_list(cPktHead);
        cPktHead = cPktTail = NULL;
        cPktNum = 0;
    }

fail_mbuf:
    if (mPktHead) {
        mbuf_freem_list(mPktHead);
        mPktHead = mPktTail = NULL;
        mPktNum = 0;
    }
    goto done;
}

IntelLucyRxPool *
IntelLucyRxPool::withCapacity(UInt32 mbufCapacity,
                              UInt32 clustCapacity,
                              UInt32 size)
{
    IntelLucyRxPool *pool = new IntelLucyRxPool;
    
    if (pool && !pool->initWithCapacity(mbufCapacity,
                                        clustCapacity,
                                        size)) {
        pool->release();
        pool = NULL;
    }
    return pool;
}

mbuf_t IntelLucyRxPool::getPacket(UInt32 size)
{
    mbuf_t m = NULL;
    
    if (size > maxCopySize) {
        if (cPktNum > 1) {
            OSDecrementAtomic(&cPktNum);
            
            m = cPktHead;
            cPktHead = mbuf_next(cPktHead);
            mbuf_setnext(m, NULL);
        }
    } else {
        if (mPktNum > 1) {
            OSDecrementAtomic(&mPktNum);
            
            m = mPktHead;
            mPktHead = mbuf_next(mPktHead);
            mbuf_setnext(m, NULL);
        }
    }
    return m;
}

void IntelLucyRxPool::refillPool()
{
    mbuf_t m;
    errno_t err;
    unsigned int chunks;
    
    while (mPktNum < mCapacity) {
        chunks = 1;
        err = mbuf_allocpacket(MBUF_WAITOK, maxCopySize, &chunks, &m);

        if (!err) {
            mbuf_setnext(mPktTail, m);
            mPktTail = m;
            OSIncrementAtomic(&mPktNum);
        }
    }
    while (cPktNum < cCapacity) {
        chunks = 1;
        err = mbuf_allocpacket(MBUF_WAITOK, cSize, &chunks, &m);
        
        if (!err) {
            mbuf_setnext(cPktTail, m);
            cPktTail = m;
            OSIncrementAtomic(&cPktNum);
        }
    }
}

/*
 * This is an exact copy of IONetworkController's method
 * replaceOrCopyPacket(), except that it tries to get new
 * packets form one of our pools.
 */
mbuf_t IntelLucyRxPool::replaceOrCopyPacket(mbuf_t *mp,
                                            UInt32 len,
                                            bool * replaced)
{
    mbuf_t m = NULL;
    
    if ((mp != NULL) && (replaced != NULL)) {
        /*
         * Packet needs to be replaced. Try to get
         * one form the cluster buffer pool.
         */
        if (len > maxCopySize) {
            if (cPktNum > 1) {
                OSDecrementAtomic(&cPktNum);
                
                m = *mp;
                *mp = cPktHead;
                cPktHead = mbuf_next(cPktHead);
                mbuf_setnext(*mp, NULL);
            }
            *replaced = true;
        } else {
            /*
             * Packet should be copied. Try to get
             * one form the mbuf buffer pool.
             */
            if (mPktNum > 1) {
                OSDecrementAtomic(&mPktNum);
                
                m = mPktHead;
                mPktHead = mbuf_next(mPktHead);
                mbuf_setnext(m, NULL);
                
                mbuf_copy_pkthdr(m, *mp);
                mbuf_pkthdr_setheader(m, NULL);
                bcopy(mbuf_data(*mp), mbuf_data(m), len);
            } else {
                /*
                 * Try to alloc a fresh mbuf in case the
                 * pool is unable to provide a mbuf.
                 */
                if (mbuf_gethdr(MBUF_DONTWAIT, MBUF_TYPE_HEADER, &m) == 0) {
                    mbuf_copy_pkthdr(m, *mp);
                    mbuf_pkthdr_setheader(m, NULL);
                    bcopy(mbuf_data(*mp), mbuf_data(m), len);
                } else {
                    m = NULL;
                }
                DebugLog("Couldn't get mbuf from pool.\n");
            }
            *replaced = false;
        }
    }
    return m;
}
