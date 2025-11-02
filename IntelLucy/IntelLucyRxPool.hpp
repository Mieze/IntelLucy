//
//  IntelLucyRxPool.hpp
//  IntelLucy
//
//  Created by Laura Müller on 26.10.25.
//

#ifndef IntelLucyRxPool_hpp
#define IntelLucyRxPool_hpp

class IntelLucyRxPool : public OSObject
{
    OSDeclareDefaultStructors(IntelLucyRxPool);

public:
    virtual bool init() APPLE_KEXT_OVERRIDE;
    
    virtual void free() APPLE_KEXT_OVERRIDE;
    
    virtual bool initWithCapacity(UInt32 mbufCapacity,
                                  UInt32 clustCapacity,
                                  UInt32 size);

    static IntelLucyRxPool * withCapacity(UInt32 mbufCapacity,
                                          UInt32 clustCapacity,
                                          UInt32 size);

    virtual mbuf_t getPacket(UInt32 size);

    virtual void refillPool();
    
    mbuf_t replaceOrCopyPacket(mbuf_t *mp,
                               UInt32 len,
                               bool * replaced);
    
protected:
    mbuf_t cPktHead;
    mbuf_t cPktTail;
    mbuf_t mPktHead;
    mbuf_t mPktTail;
    UInt32 cCapacity;
    SInt32 cPktNum;
    UInt32 mCapacity;
    SInt32 mPktNum;
    UInt32 cSize;
    UInt32 maxCopySize;
};

#endif /* IntelLucyRxPool_hpp */
