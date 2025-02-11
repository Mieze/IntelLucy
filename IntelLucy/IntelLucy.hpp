/* IntelLucy.h -- IntelLucy driver class definition.
 *
 * Copyright (c) 2024 Laura Müller <laura-mueller@uni-duesseldorf.de>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * Driver for Intel PCIe 10Gbit ethernet controllers.
 *
 * This driver is based on Intel's ixgbe driver for Linux.
 */

#include "uapi-mdio.h"
#include "ixgbe_phy.h"
#include "ixgbe.h"
#include "ixgbe_txrx_common.h"
#include "ixgbe_dcb_82599.h"


#include "IntelLucySupport.hpp"

#define    RELEASE(x)    if(x){(x)->release();(x)=NULL;}

#define ReleaseMcAddrList() if(mcAddrList) { \
    IOFree(mcAddrList, mcListCount * sizeof(IOEthernetAddress)); \
    mcAddrList = NULL; \
    mcListCount = 0; }

#define super IOEthernetController

enum
{
    MIDX_AUTO = 0,
    MIDX_100,
    MIDX_100FC,
    MIDX_1000,
    MIDX_1000FC,
    MIDX_10GB,
    MIDX_10GBFC,
    MIDX_2500,
    MIDX_2500FC,
    MIDX_5000,
    MIDX_5000FC,
    MIDX_100_EEE,
    MIDX_100FC_EEE,
    MIDX_1000_EEE,
    MIDX_1000FC_EEE,
    MIDX_10GB_EEE,
    MIDX_10GBFC_EEE,
    MIDX_2500_EEE,
    MIDX_2500FC_EEE,
    MIDX_5000_EEE,
    MIDX_5000FC_EEE,
    MIDX_1GB_SX,
    MIDX_1GB_SX_FC,
    MIDX_10GB_SR,
    MIDX_10GB_SR_FC,
    MIDX_1GB_LX,
    MIDX_1GB_LX_FC,
    MIDX_10GB_LR,
    MIDX_10GB_LR_FC,
    MIDX_10GB_DA,
    MIDX_10GB_DA_FC,
    MIDX_10GB_DAL,
    MIDX_10GB_DAL_FC,
    MIDX_COUNT
};

#define MBit 1000000ULL

enum {
    kSpeed10000MBit = 10000*MBit,
    kSpeed5000MBit = 5000*MBit,
    kSpeed2500MBit = 2500*MBit,
    kSpeed1000MBit = 1000*MBit,
    kSpeed100MBit = 100*MBit,
    kSpeed10MBit = 10*MBit,
};

enum {
    kFlowControlTypeNone = 0,
    kFlowControlTypeRx = 1,
    kFlowControlTypeTx = 2,
    kFlowControlTypeRxTx = 3,
    kFlowControlTypeCount
};

enum {
    kEEETypeNo = 0,
    kEEETypeYes = 1,
    kEEETypeCount
};

#define kTransmitQueueCapacity  1000

/* With up to 40 segments we should be on the save side. */
#define kMaxSegs 40

#define kTxSpareDescs   16

/* The number of descriptors must be a power of 2. */
#define kNumTxDesc      1024    /* Number of Tx descriptors */
#define kNumRxDesc      1024     /* Number of Rx descriptors */
#define kTxLastDesc     (kNumTxDesc - 1)
#define kRxLastDesc     (kNumRxDesc - 1)
#define kTxDescMask     (kNumTxDesc - 1)
#define kRxDescMask     (kNumRxDesc - 1)
#define kNumTxRings     1
#define kNumRxRings     1
#define kTxDescRingSize (kNumTxDesc * sizeof(union ixgbe_adv_tx_desc))
#define kRxDescRingSize (kNumRxDesc * sizeof(union ixgbe_adv_rx_desc))
#define kTxDescMemSize  (kTxDescRingSize * kNumTxRings)
#define kRxDescMemSize  (kRxDescRingSize * kNumRxRings)
#define kTxBufArraySize (kNumTxDesc * sizeof(struct ixgbeTxBufferInfo))
#define kRxBufArraySize (kNumRxDesc * sizeof(struct ixgbeRxBufferInfo))
#define kTxBufMemSize (kTxBufArraySize * kNumTxRings)
#define kRxBufMemSize (kRxBufArraySize * kNumRxRings)

/* This is the receive buffer size (must be large enough to hold a packet). */
enum {
    kRxBufferPktSize2K = 2048,
    kRxBufferPktSize4K = 4096,
    kRxBufferPktSize8K = 8192,
    kRxBufferPktSize16K = 16384

};

#define kRxNumSpareMbufs 150
#define kMaxMtu 9216
#define kMaxPacketSize (kMaxMtu + ETH_HLEN + ETH_FCS_LEN)

/* statitics timer period in ms. */
#define kTimeout10us   10U
#define kTimeout100ms   100U
#define kTimeout1000ms  1000U
#define kTimeout2000ms  2000U
#define kTimeout2Gns  2000000000UL
#define kTimeout4Gns  4000000000UL
#define kTimespan4ms  4000000UL

/* Treshhold value to wake a stalled queue */
#define kTxQueueWakeTreshhold (2 + kMaxSegs * 2)

/* transmitter deadlock treshhold in seconds. */
#define kTxHungTreshhold 2

/* Maximum DMA latency in ns. */
#define kMaxDmaLatency 90000

/* Maximum DMA latency in ns for Sonoma and above. */
#define kMaxIntrLatencySono 45000

/* Maximum DMA latency in ns for Ventura and below. */
#define kMaxIntrLatencyVent 50000

#define kMacHdrLen      14
#define kIPv4HdrLen     20
#define kIPv6HdrLen     40

/* Additional bittime to account for IXGBE framing */
#define IXGBE_ETH_FRAMING 20

/* These definitions should have been in IOPCIDevice.h. */

enum
{
    kIOPCIPMCapability = 2,
    kIOPCIPMControl = 4,
    kIOPCISRIOVCapability = 0x10,
    kIOPCIDC2Capability = 0x28,
};

enum
{
    kIOPCIEDevCtl2 = 0x04,
    kIOPCIEDeviceControl = 8,
    kIOPCIELinkCapability = 12,
    kIOPCIELinkControl = 16,
    kIOPCIELinkStatus = 18,
};

enum
{
    kIOPCIELinkCtlASPM = 0x0003,    /* ASPM Control */
    kIOPCIELinkCtlL0s = 0x0001,     /* L0s Enable */
    kIOPCIELinkCtlL1 = 0x0002,      /* L1 Enable */
    kIOPCIELinkCtlCcc = 0x0040,     /* Common Clock Configuration */
    kIOPCIELinkCtlClkReqEn = 0x100, /* Enable clkreq */
};

enum
{
    kIOPCIELinkCapL0sSup = 0x00000400UL,
    kIOPCIELinkCapL1Sup = 0x00000800UL,
    kIOPCIELinkCapASPMCompl = 0x00400000UL,
};

enum
{
    kIOPCIEDevCtlReadQ = 0x7000,
};

enum
{
    kPowerStateOff = 0,
    kPowerStateOn,
    kPowerStateCount
};

#define kParamName "Driver Parameters"
#define kEnableTSO4Name "enableTSO4"
#define kEnableTSO6Name "enableTSO6"
#define kEnableWoAName "enableWakeOnAddr"
#define kEnableASPM "enableASPM"
#define kDriverVersionName "DriverVersion"
#define kRxCoalescingName "rxCoalescing"
#define kRxBufferSize4kName "rxBufferSize4k"
#define kPollTime10GName "µsPollTime10G"
#define kPollTime5GName "µsPollTime5G"
#define kPollTime2GName "µsPollTime2G"
#define kRxItrTimeName "µsRxItrTime"
#define kTxItrTimeName "µsTxItrTime"
#define kNameLenght 64

struct intelDevice {
    UInt16 pciDevId;
    UInt16 device;
    const char *deviceName;
    const struct ixgbe_info *deviceInfo;
};

enum IntelLucyRxTxStateFlags {
    __RX_ADAPT = 0,     /* adaptive ITR enabled*/
    __RX_RSC = 1,       /* RSC enabled */
};

enum IntelLucyRxTxStateMask {
    __RX_ADAPT_M = (1 << __RX_ADAPT),
    __RX_RSC_M = (1 << __RX_RSC),
};

#define ring_is_rsc_enabled(ring) \
    (__RX_RSC_M & (ring)->state)
#define set_ring_rsc_enabled(ring) \
    OSBitOrAtomic8(__RX_RSC_M, &(ring)->state)
#define clear_ring_rsc_enabled(ring) \
    OSBitAndAtomic8(~__RX_RSC_M, &(ring)->state)

#define ring_is_adaptive(ring) \
    (__RX_ADAPT_M & (ring)->state)
#define set_ring_adaptive(ring) \
    OSBitOrAtomic8(__RX_ADAPT_M, &(ring)->state)
#define clear_ring_adaptive(ring) \
    OSBitAndAtomic8(~__RX_ADAPT_M, &(ring)->state)

struct ixgbeTxBufferInfo {
    mbuf_t mbuf;
    UInt32 numDescs;
    UInt32 packetBytes;
};

struct ixgbeTxRing {
    union ixgbe_adv_tx_desc *txDescArray;
    struct ixgbeTxBufferInfo *txBufArray;
    struct ixgbeQueueVector vector;
    IOPhysicalAddress64 txPhyRingAddr;
    SInt64 txDescDone;
    SInt64 txDescDoneLast;
    SInt32 txNumFreeDesc;
    UInt16 txNextDescIndex;
    UInt16 txDirtyIndex;
    UInt16 txCleanBarrierIndex;
    UInt16 txHangSuspected;
    UInt8 regIndex;
    UInt8 state;
};

struct ixgbeRxBufferInfo {
    mbuf_t mbuf;
    IOPhysicalAddress64 phyAddr;
    mbuf_t rscHead;
    mbuf_t rscTail;
};

struct ixgbeRxRing {
    union ixgbe_adv_rx_desc *rxDescArray;
    struct ixgbeRxBufferInfo *rxBufArray;
    struct ixgbeQueueVector vector;
    IOPhysicalAddress64 rxPhyRingAddr;
    mbuf_t rxPacketHead;
    mbuf_t rxPacketTail;
    UInt32 rxPacketSize;
    UInt16 rxNextDescIndex;
    UInt16 rxCleanedCount;
    UInt8 regIndex;
    UInt8 state;
};

struct mediumTable {
    IOMediumType    type;
    UInt64          speed;
    UInt32          idx;
    UInt32          adv;
    UInt32          fc;
};

enum IntelLucyStateFlags {
    __ENABLED = 0,      /* driver is enabled */
    __LINK_UP = 1,      /* link is up */
    __PROMISC = 2,      /* promiscuous mode enabled */
    __M_CAST = 3,       /* multicast mode enabled */
    __POLL_MODE = 4,    /* poll mode is active */
    __POLLING = 5,      /* poll routine is polling */
};

enum IntelLucyStateMask {
    __ENABLED_M = (1 << __ENABLED),
    __LINK_UP_M = (1 << __LINK_UP),
    __PROMISC_M = (1 << __PROMISC),
    __M_CAST_M = (1 << __M_CAST),
    __POLL_MODE_M = (1 << __POLL_MODE),
    __POLLING_M = (1 << __POLLING),
};

/**
 *  Known kernel versions
 */
enum KernelVersion {
    Tiger         = 8,
    Leopard       = 9,
    SnowLeopard   = 10,
    Lion          = 11,
    MountainLion  = 12,
    Mavericks     = 13,
    Yosemite      = 14,
    ElCapitan     = 15,
    Sierra        = 16,
    HighSierra    = 17,
    Mojave        = 18,
    Catalina      = 19,
    BigSur        = 20,
    Monterey      = 21,
    Ventura       = 22,
    Sonoma        = 23,
    Sequoia       = 24,
};

/**
 *  Kernel version major
 */
extern const int version_major;


class IntelLucy : public super
{
    
    OSDeclareDefaultStructors(IntelLucy)
    
public:
    /* IOService (or its superclass) methods. */
    virtual bool start(IOService *provider) override;
    virtual void stop(IOService *provider) override;
    virtual bool init(OSDictionary *properties) override;
    virtual void free() override;
    
    /* Power Management Support */
    virtual IOReturn registerWithPolicyMaker(IOService *policyMaker) override;
    virtual IOReturn setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker ) override;
    virtual void systemWillShutdown(IOOptionBits specifier) override;
    
    /* IONetworkController methods. */
    virtual IOReturn enable(IONetworkInterface *netif) override;
    virtual IOReturn disable(IONetworkInterface *netif) override;
    
    virtual IOReturn outputStart(IONetworkInterface *interface, IOOptionBits options ) override;
    virtual IOReturn setInputPacketPollingEnable(IONetworkInterface *interface, bool enabled) override;
    virtual void pollInputPackets(IONetworkInterface *interface, uint32_t maxCount, IOMbufQueue *pollQueue, void *context) override;
    
    virtual void getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const override;
    
    virtual IOOutputQueue* createOutputQueue() override;
    
    virtual const OSString* newVendorString() const override;
    virtual const OSString* newModelString() const override;
    
    virtual IOReturn selectMedium(const IONetworkMedium *medium) override;
    virtual bool configureInterface(IONetworkInterface *interface) override;
    
    virtual bool createWorkLoop() override;
    virtual IOWorkLoop* getWorkLoop() const override;
    
    /* Methods inherited from IOEthernetController. */
    virtual IOReturn getHardwareAddress(IOEthernetAddress *addr) override;
    virtual IOReturn setHardwareAddress(const IOEthernetAddress *addr) override;
    virtual IOReturn setPromiscuousMode(bool active) override;
    virtual IOReturn setMulticastMode(bool active) override;
    virtual IOReturn setMulticastList(IOEthernetAddress *addrs, UInt32 count) override;
    virtual IOReturn getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput) override;
    virtual IOReturn getMinPacketSize(UInt32 *minSize) const override;
    virtual IOReturn setWakeOnMagicPacket(bool active) override;
    virtual IOReturn getPacketFilters(const OSSymbol *group, UInt32 *filters) const override;
    
    virtual UInt32 getFeatures() const override;
    virtual IOReturn getMaxPacketSize(UInt32 * maxSize) const override;
    virtual IOReturn setMaxPacketSize(UInt32 maxSize) override;

    void ixgbeServiceEventSchedule(struct ixgbe_adapter *adapter);

private:
    void getParams();
    bool createMediaTable();
    bool updateMediumDict();
    void updateSelectedMedium();
    void medium2Advertise(const IONetworkMedium *medium, UInt32 *adv, UInt32 *fc);
    bool initEventSources(IOService *provider);
    bool setupDMADescriptors();
    void freeDMADescriptors();
    bool allocTxBufferInfo();
    bool allocRxBuffers();
    void refillSpareBuffers();
    
    static IOReturn refillAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4);

    static IOReturn setPowerStateWakeAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4);
    static IOReturn setPowerStateSleepAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4);

    bool initPCIConfigSpace(IOPCIDevice *provider);
    void initPCIPowerManagment(IOPCIDevice *provider, const struct e1000_info *ei);
    inline void ixgbeEnablePCIDevice(IOPCIDevice *provider);
    void interruptOccurred(OSObject *client, IOInterruptEventSource *src, int count);
    
    void txCleanRing(struct ixgbeTxRing *ring);
    UInt32 rxCleanRing(IONetworkInterface *interface, struct ixgbeRxRing *ring,
                       uint32_t maxCount, IOMbufQueue *pollQueue, void *context);

    /* Hardware specific methods */
    bool ixgbeIdentifyChip();
    bool ixgbeStart(const struct intelDevice *devTable);
    bool ixgbeSwInit(struct ixgbe_adapter *adapter, const struct ixgbe_info *ii);
    bool ixgbeEnable(struct ixgbe_adapter *adapter);
    void ixgbeUp(struct ixgbe_adapter *adapter);
    void ixgbeReinit(struct ixgbe_adapter *adapter);

    void ixgbeConfigurePb(struct ixgbe_adapter *adapter);
    void ixgbePbthreshSetup(struct ixgbe_adapter *adapter);
    int ixgbeHpbthresh(struct ixgbe_adapter *adapter, int pb);
    int ixgbeLpbthresh(struct ixgbe_adapter *adapter, int pb);
    void ixgbeSetupGpie(struct ixgbe_adapter *adapter);
    void ixgbeConfigureMsi(struct ixgbe_adapter *adapter);

    void ixgbeStop(struct ixgbe_adapter *adapter);
    void ixgbeDisable(struct ixgbe_adapter *adapter);
    void ixgbeDown(struct ixgbe_adapter *adapter);
    void ixgbeReset(struct ixgbe_adapter *adapter);
    void ixgbeTxTimeoutReset(struct ixgbe_adapter *adapter);
    void ixgbeCheckHangSubtask(struct ixgbe_adapter *adapter);

    unsigned long ixgbeGetCompletionTimeout(struct ixgbe_adapter *adapter);
    SInt32 ixgbeDisablePCIePrimary(struct ixgbe_hw *hw);
    SInt32 ixgbeStopAdapterGeneric(struct ixgbe_hw *hw);
    UInt16 ixgbeGetPCIeMSIxCountGeneric(struct ixgbe_hw *hw);
    UInt32 ixgbePCIeTimeoutPoll(struct ixgbe_hw *hw);
    void ixgbeClearTxPending(struct ixgbe_hw *hw);
    
    SInt32 ixgbeResetHwX550em(struct ixgbe_hw *hw);
    SInt32 ixgbeResetHwX540(struct ixgbe_hw *hw);
    SInt32 ixgbeResetHw82599(struct ixgbe_hw *hw);
    SInt32 ixgbeInitHwGeneric(struct ixgbe_hw *hw);
    SInt32 ixgbeNonSfpLinkConfig(struct ixgbe_hw *hw);
    
    void ixgbeWatchdogLinkIsDown(struct ixgbe_adapter *adapter);
    void ixgbeWatchdogLinkIsUp(struct ixgbe_adapter *adapter);
    void ixgbeWatchdogUpdateLink(struct ixgbe_adapter *adapter);
    bool ixgbeRingTxPending(struct ixgbe_adapter *adapter);
    void ixgbeWatchdogFlushTx(struct ixgbe_adapter *adapter);
    void ixgbeWatchdogSubtask(struct ixgbe_adapter *adapter);

    void ixgbeCheckSfpEvent(struct ixgbe_adapter *adapter, u32 eicr);
    void ixgbeCheckOvertempSubtask(struct ixgbe_adapter *adapter);
    void ixgbeCheckOvertempEvent(struct ixgbe_adapter *adapter, u32 eicr);
    void ixgbeSfpDetectionSubtask(struct ixgbe_adapter *adapter);
    void ixgbeSfpLinkConfigSubtask(struct ixgbe_adapter *adapter);
    void ixgbePhyInterruptSubtask(struct ixgbe_adapter *adapter);
    void ixgbeResetSubtask(struct ixgbe_adapter *adapter);

    void ixgbeServiceTask();
    void ixgbeServiceEventComplete(struct ixgbe_adapter *adapter);

    int ixgbeWriteMcAddrList(IOEthernetAddress *addrs, UInt32 count);
    int ixgbeSyncMcAddrList();

    void ixgbeConfigureSrrctl(struct ixgbe_adapter *adapter);
    void ixgbeSetupMtqc(struct ixgbe_adapter *adapter);
    void ixgbeSetRxMode();
    void ixgbeDisableRx(struct ixgbe_adapter *adapter);
    void ixgbeSetRxBufferLen(struct ixgbe_adapter *adapter);
    void ixgbeVlanStripEnable(struct ixgbe_adapter *adapter);
    
    void ixgbeSetNumQueues(struct ixgbe_adapter *adapter);
    void ixgbeConfigureTx(struct ixgbe_adapter *adapter);
    
    void ixgbeConfigureRx(struct ixgbe_adapter *adapter);
    void ixgbeConfigure(struct ixgbe_adapter *adapter);
    void ixgbeUpComplete(struct ixgbe_adapter *adapter);
    void ixgbeConfigRelaxOrder(struct ixgbe_adapter *adapter);

    void ixgbeClearTxRings(struct ixgbe_adapter *adapter);
    void ixgbeClearRxRings(struct ixgbe_adapter *adapter);
    void ixgbeDisableTx(struct ixgbe_adapter *adapter);

    void ixgbeConfigLink(const IONetworkMedium *medium);

    void ixgbeSetupQueueVectors(struct ixgbe_adapter *adapter);
    void ixgbeConfigureTxRing(struct ixgbe_adapter *adapter,
                                         struct ixgbeTxRing *ring);
    void ixgbeConfigureRxRing(struct ixgbe_adapter *adapter,
                        struct ixgbeRxRing *ring);
    void ixgbeRxDescQueueEnable(struct ixgbe_adapter *adapter,
                                struct ixgbeRxRing *ring);
    void ixgbeUpdateStats(struct ixgbe_adapter *adapter);
    
    /* Jumbo frame support methods */
    void ixgbeDropPktFragment(struct ixgbeRxRing *ring);
    
    inline void ixgbeGetChecksumResult(mbuf_t m, UInt32 status);

    /* timer action */
    void timerAction(IOTimerEventSource *timer);

private:
    IOWorkLoop *workLoop;
    IOCommandGate *commandGate;
    IOPCIDevice *pciDevice;
    OSDictionary *mediaDict;
    IONetworkMedium *mediaTable[MIDX_COUNT];
    IOBasicOutputQueue *txQueue;
    
    IOInterruptEventSource *interruptSource;
    IOTimerEventSource *timerSource;
    IOEthernetInterface *netif;
    IOMemoryMap *baseMap;
    IOMapper *mapper;
    volatile void *baseAddr;
    
    /* transmitter data */
    struct ixgbeTxRing txRing[kNumTxRings];
    void *txBufArrayMem;
    IOPhysicalAddress64 txPhyAddr;
    IODMACommand *txDescDmaCmd;
    IOBufferMemoryDescriptor *txBufDesc;
    IOMbufNaturalMemoryCursor *txMbufCursor;
    UInt32 txActiveQueueMask;
    UInt32 mtu;
    UInt32 maxLatency;
    
    /* receiver data */
    struct ixgbeRxRing rxRing[kNumRxRings];
    void *rxBufArrayMem;
    IOPhysicalAddress64 rxPhyAddr;
    IODMACommand *rxDescDmaCmd;
    IOBufferMemoryDescriptor *rxBufDesc;
    IOMbufNaturalMemoryCursor *rxMbufCursor;
    mbuf_t sparePktHead;
    mbuf_t sparePktTail;
    IONetworkPacketPollingParameters pollParams;
    IOEthernetAddress *mcAddrList;
    UInt32 mcListCount;
    UInt32 rxActiveQueueMask;
    UInt32 rxBufferPktSize;
    SInt32 spareNum;

    /* power management data */
    unsigned long powerState;
    
    /* statistics data */
    IONetworkStats *netStats;
    IOEthernetStats *etherStats;
    
    /* time values */
    UInt64 sfpPollTime;
    UInt64 timeoutCheck;
    UInt64 hangTimeout;
    UInt64 itrUpdatePeriod;
    
    UInt32 chip;
    struct ixgbe_adapter adapterData;
    struct pci_dev pciDeviceData;
    struct mii_bus miiBus;

    IOByteCount pcieCapOffset;
    IOByteCount pciPMCtrlOffset;
    UInt16 pciSRIOVOffset;

    /* poll intervals in ns */
    UInt64 pollTime10G;
    UInt64 pollTime5G;
    UInt64 pollTime2G;

    /* interrupt throttling times in µs */
    UInt32 rxThrottleTime;
    UInt32 txThrottleTime;

    /* maximum interrupt latency */
    UInt32 intrLatency;
    
    /* flags */
    UInt32 stateFlags;
        
    bool wolActive;
    bool enableTSO4;
    bool enableTSO6;
    bool enableASPM;
    bool enableRSC;
    bool allowUnsupportedSFP;
};

/*
 * This is an ugly solution but the only way to call
 * c++ code from C code.
 */
#ifdef __cplusplus
extern "C" {
#endif

void ixgbe_service_event_schedule(void *owner, struct ixgbe_adapter *adapter);

#ifdef __cplusplus
}
#endif
