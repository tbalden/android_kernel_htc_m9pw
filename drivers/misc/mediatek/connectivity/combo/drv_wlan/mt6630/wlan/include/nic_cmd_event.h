

#ifndef _NIC_CMD_EVENT_H
#define _NIC_CMD_EVENT_H



#define CMD_PQ_ID           (0x8000)
#define CMD_PACKET_TYPE_ID  (0xA0)

#define CMD_STATUS_SUCCESS      0
#define CMD_STATUS_REJECTED     1
#define CMD_STATUS_UNKNOWN      2

#define EVENT_HDR_SIZE          OFFSET_OF(WIFI_EVENT_T, aucBuffer[0])

#define MAX_IE_LENGTH       (600)
#define MAX_WSC_IE_LENGTH   (400)

#define CMD_CH_ACTION_REQ           0
#define CMD_CH_ACTION_ABORT         1

#define EVENT_CH_STATUS_GRANT       0



typedef enum _ENUM_CMD_ID_T {
    CMD_ID_TEST_CTRL            = 0x01, 
    CMD_ID_BASIC_CONFIG,                
    CMD_ID_SCAN_REQ_V2,                 
    CMD_ID_NIC_POWER_CTRL,              
    CMD_ID_POWER_SAVE_MODE,             
    CMD_ID_LINK_ATTRIB,                 
    CMD_ID_ADD_REMOVE_KEY,              
    CMD_ID_DEFAULT_KEY_ID,              
    CMD_ID_INFRASTRUCTURE,              
    CMD_ID_SET_RX_FILTER,               
    CMD_ID_DOWNLOAD_BUF,                
    CMD_ID_WIFI_START,                  
    CMD_ID_CMD_BT_OVER_WIFI,            
    CMD_ID_SET_MEDIA_CHANGE_DELAY_TIME, 
    CMD_ID_SET_DOMAIN_INFO,             
    CMD_ID_SET_IP_ADDRESS,              
    CMD_ID_BSS_ACTIVATE_CTRL,           
    CMD_ID_SET_BSS_INFO,                
    CMD_ID_UPDATE_STA_RECORD,           
    CMD_ID_REMOVE_STA_RECORD,           
    CMD_ID_INDICATE_PM_BSS_CREATED,     
    CMD_ID_INDICATE_PM_BSS_CONNECTED,   
    CMD_ID_INDICATE_PM_BSS_ABORT,       
    CMD_ID_UPDATE_BEACON_CONTENT,       
    CMD_ID_SET_BSS_RLM_PARAM,           
    CMD_ID_SCAN_REQ,                    
    CMD_ID_SCAN_CANCEL,                 
    CMD_ID_CH_PRIVILEGE,                
    CMD_ID_UPDATE_WMM_PARMS,            
    CMD_ID_SET_WMM_PS_TEST_PARMS,       
    CMD_ID_TX_AMPDU,                    
    CMD_ID_ADDBA_REJECT,                
    CMD_ID_SET_PS_PROFILE_ADV,          
    CMD_ID_SET_RAW_PATTERN,             
    CMD_ID_CONFIG_PATTERN_FUNC,         
    CMD_ID_SET_TX_PWR,                  
    CMD_ID_SET_PWR_PARAM,               
    CMD_ID_P2P_ABORT,                   

    
    CMD_ID_RANDOM_RX_RESET_EN   = 0x2C, 
    CMD_ID_RANDOM_RX_RESET_DE   = 0x2D, 
    CMD_ID_SAPP_EN              = 0x2E, 
    CMD_ID_SAPP_DE              = 0x2F, 

    CMD_ID_ROAMING_TRANSIT      = 0x30, 
    CMD_ID_SET_PHY_PARAM,               
    CMD_ID_SET_NOA_PARAM,               
    CMD_ID_SET_OPPPS_PARAM,             
    CMD_ID_SET_UAPSD_PARAM,             
    CMD_ID_SET_SIGMA_STA_SLEEP,         
    CMD_ID_SET_EDGE_TXPWR_LIMIT,        
    CMD_ID_SET_DEVICE_MODE,             
    CMD_ID_SET_TXPWR_CTRL,              
    CMD_ID_SET_AUTOPWR_CTRL,            
    CMD_ID_SET_WFD_CTRL,                
    CMD_ID_SET_NLO_REQ,                 
    CMD_ID_SET_NLO_CANCEL,              
    CMD_ID_SET_GTK_REKEY_DATA,          
    CMD_ID_ROAMING_CONTROL,             
    CMD_ID_RESET_BA_SCOREBOARD  = 0x3f, 
	CMD_ID_SET_EDGE_TXPWR_LIMIT_5G = 0x40,	
    CMD_ID_SET_CHANNEL_PWR_OFFSET,          
    CMD_ID_SET_80211AC_TX_PWR,          
    CMD_ID_SET_PATH_COMPASATION,        
    
    CMD_ID_SET_BATCH_REQ        = 0x47,	
    CMD_ID_SET_NVRAM_SETTINGS,          
    CMD_ID_SET_COUNTRY_POWER_LIMIT,     

    CMD_ID_GET_NIC_CAPABILITY   = 0x80, 
    CMD_ID_GET_LINK_QUALITY,            
    CMD_ID_GET_STATISTICS,              
    CMD_ID_GET_CONNECTION_STATUS,       
    CMD_ID_GET_STA_STATISTICS = 0x85,   

    CMD_ID_GET_LTE_CHN = 0x87,   	   
    CMD_ID_GET_CHN_LOADING = 0x88,     

    CMD_ID_ACCESS_REG           = 0xc0, 
    CMD_ID_MAC_MCAST_ADDR,              
    CMD_ID_802_11_PMKID,                
    CMD_ID_ACCESS_EEPROM,               
    CMD_ID_SW_DBG_CTRL,                 
    CMD_ID_SEC_CHECK,                   
    CMD_ID_DUMP_MEM,                    
    CMD_ID_RESOURCE_CONFIG,             	
    CMD_ID_CHIP_CONFIG          = 0xCA,     
    CMD_ID_SET_RDD_CH           = 0xE1,
    CMD_ID_SET_BWCS             = 0xF1,
    CMD_ID_SET_OSC              = 0xF2,

    CMD_ID_GET_BUILD_DATE_CODE = 0xF8,   
    CMD_ID_GET_BSS_INFO = 0xF9,          
    CMD_ID_SET_HOTSPOT_OPTIMIZATION = 0xFA,    
    CMD_ID_SET_TDLS_CH_SW = 0xFB,
    CMD_ID_SET_MONITOR = 0xFC,          
    CMD_ID_END
} ENUM_CMD_ID_T, *P_ENUM_CMD_ID_T;

typedef enum _ENUM_EVENT_ID_T {
    EVENT_ID_NIC_CAPABILITY     = 0x01, 
    EVENT_ID_LINK_QUALITY,              
    EVENT_ID_STATISTICS,                
    EVENT_ID_MIC_ERR_INFO,              
    EVENT_ID_ACCESS_REG,                
    EVENT_ID_ACCESS_EEPROM,             
    EVENT_ID_SLEEPY_INFO,               
    EVENT_ID_BT_OVER_WIFI,              
    EVENT_ID_TEST_STATUS,               
    EVENT_ID_RX_ADDBA,                  
    EVENT_ID_RX_DELBA,                  
    EVENT_ID_ACTIVATE_STA_REC,          
    EVENT_ID_SCAN_DONE,                 
    EVENT_ID_RX_FLUSH,                  
    EVENT_ID_TX_DONE,                   
    EVENT_ID_CH_PRIVILEGE,              
    EVENT_ID_BSS_ABSENCE_PRESENCE,      
    EVENT_ID_STA_CHANGE_PS_MODE,        
    EVENT_ID_BSS_BEACON_TIMEOUT,        
    EVENT_ID_UPDATE_NOA_PARAMS,         
    EVENT_ID_AP_OBSS_STATUS,            
    EVENT_ID_STA_UPDATE_FREE_QUOTA,     
    EVENT_ID_SW_DBG_CTRL,               
    EVENT_ID_ROAMING_STATUS,            
    EVENT_ID_STA_AGING_TIMEOUT,         
    EVENT_ID_SEC_CHECK_RSP,             
    EVENT_ID_SEND_DEAUTH,               
    EVENT_ID_UPDATE_RDD_STATUS,         
    EVENT_ID_UPDATE_BWCS_STATUS,        
    EVENT_ID_UPDATE_BCM_DEBUG,          
    EVENT_ID_RX_ERR,                    
    EVENT_ID_DUMP_MEM = 0x20,           
    EVENT_ID_STA_STATISTICS,            
    EVENT_ID_STA_STATISTICS_UPDATE,     
    EVENT_ID_NLO_DONE,                  
    EVENT_ID_ADD_PKEY_DONE,             
    EVENT_ID_ICAP_DONE,                 
    EVENT_ID_RESOURCE_CONFIG = 0x26,    
    EVENT_ID_DEBUG_MSG = 0x27,          
    EVENT_ID_RTT_CALIBR_DONE = 0x28,    
    EVENT_ID_RTT_UPDATE_RANGE = 0x29,   
    EVENT_ID_CHECK_REORDER_BUBBLE = 0x2a,      
    EVENT_ID_BATCH_RESULT = 0x2b,		

    EVENT_ID_UART_ACK = 0x40,           
    EVENT_ID_UART_NAK,                  
    EVENT_ID_GET_CHIPID,                
	EVENT_ID_SLT_STATUS,	
    EVENT_ID_CHIP_CONFIG,               

	EVENT_ID_TDLS = 0x80,	

    EVENT_ID_BUILD_DATE_CODE = 0xF8,
    EVENT_ID_GET_AIS_BSS_INFO = 0xF9,
    EVENT_ID_DEBUG_CODE = 0xFB,
    EVENT_ID_RFTEST_READY = 0xFC,   

    EVENT_ID_END
} ENUM_EVENT_ID_T, *P_ENUM_EVENT_ID_T;


#ifndef LINUX
typedef UINT_8 CMD_STATUS;
#endif
    
typedef struct _WIFI_CMD_T {
    UINT_16     u2TxByteCount;  
    UINT_16     u2PQ_ID;        
    UINT_8      ucCID;
    UINT_8      ucPktTypeID;    
    UINT_8      ucSetQuery;
    UINT_8      ucSeqNum;

    UINT_8      aucBuffer[0];
} WIFI_CMD_T, *P_WIFI_CMD_T;

    
typedef struct _WIFI_EVENT_T {
    UINT_16     u2PacketLength;
    UINT_16     u2PacketType;    
    UINT_8      ucEID;
    UINT_8      ucSeqNum;
    UINT_8      aucReserved[2];

    UINT_8      aucBuffer[0];
} WIFI_EVENT_T, *P_WIFI_EVENT_T;

typedef struct _CMD_TEST_CTRL_T {
    UINT_8      ucAction;
    UINT_8      aucReserved[3];
    union {
        UINT_32                 u4OpMode;
        UINT_32                 u4ChannelFreq;
        PARAM_MTK_WIFI_TEST_STRUC_T rRfATInfo;
    } u;
} CMD_TEST_CTRL_T, *P_CMD_TEST_CTRL_T;

typedef struct _PARAM_CUSTOM_RFTEST_TX_STATUS_STRUC_T {
    UINT_32             u4PktSentStatus;
    UINT_32             u4PktSentCount;
    UINT_16             u2AvgAlc;
    UINT_8              ucCckGainControl;
    UINT_8              ucOfdmGainControl;
} PARAM_CUSTOM_RFTEST_TX_STATUS_STRUC_T, *P_PARAM_CUSTOM_RFTEST_TX_STATUS_STRUC_T;

typedef struct  _PARAM_CUSTOM_RFTEST_RX_STATUS_STRUC_T {
    UINT_32             u4IntRxOk;            
    UINT_32             u4IntCrcErr;          
    UINT_32             u4IntShort;           
    UINT_32             u4IntLong;            
    UINT_32             u4PauRxPktCount;      
    UINT_32             u4PauCrcErrCount;     
    UINT_32             u4PauRxFifoFullCount; 
    UINT_32             u4PauCCACount;        
} PARAM_CUSTOM_RFTEST_RX_STATUS_STRUC_T, *P_PARAM_CUSTOM_RFTEST_RX_STATUS_STRUC_T;

typedef union _EVENT_TEST_STATUS {
    PARAM_MTK_WIFI_TEST_STRUC_T             rATInfo;
} EVENT_TEST_STATUS, *P_EVENT_TEST_STATUS;

typedef struct _CMD_BUILD_CONNECTION {
    UINT_8      ucInfraMode;
    UINT_8      ucAuthMode;
    UINT_8      ucEncryptStatus;
    UINT_8      ucSsidLen;
    UINT_8      aucSsid[PARAM_MAX_LEN_SSID];
    UINT_8      aucBssid[PARAM_MAC_ADDR_LEN];

    
    UINT_16     u2BeaconPeriod;
    UINT_16     u2ATIMWindow;
    UINT_8      ucJoinOnly;
    UINT_8      ucReserved;
    UINT_32     u4FreqInKHz;

    
    UINT_8      aucScanResult[0];
} CMD_BUILD_CONNECTION, *P_CMD_BUILD_CONNECTION;


typedef struct _CMD_802_11_KEY {
    UINT_8      ucAddRemove;
    UINT_8      ucTxKey;
    UINT_8      ucKeyType;
    UINT_8      ucIsAuthenticator;
    UINT_8      aucPeerAddr[6];
    UINT_8      ucBssIdx;
    UINT_8      ucAlgorithmId;
    UINT_8      ucKeyId;
    UINT_8      ucKeyLen;
    UINT_8      ucWlanIndex;
    UINT_8      ucReverved;
    UINT_8      aucKeyMaterial[32];
    UINT_8      aucKeyRsc[16];
} CMD_802_11_KEY, *P_CMD_802_11_KEY;

typedef struct _CMD_DEFAULT_KEY {
    UINT_8      ucBssIdx;
    UINT_8      ucKeyId;
    UINT_8      ucUnicast;
    UINT_8      ucMulticast;
} CMD_DEFAULT_KEY, *P_CMD_DEFAULT_KEY;

typedef struct _PMKID_ENTRY_T {
    PARAM_BSSID_INFO_T  rBssidInfo;
    BOOLEAN             fgPmkidExist;
} PMKID_ENTRY_T, *P_PMKID_ENTRY_T;

typedef struct _CMD_802_11_PMKID {
	UINT_32 u4BSSIDInfoCount;
    P_PMKID_ENTRY_T     arPMKIDInfo[1];
} CMD_802_11_PMKID, *P_CMD_802_11_PMKID;

typedef struct _CMD_GTK_REKEY_DATA_T {
    UINT_8              aucKek[16];
    UINT_8              aucKck[16];
    UINT_8              aucReplayCtr[8];
} CMD_GTK_REKEY_DATA_T, *P_CMD_GTK_REKEY_DATA_T;

typedef struct _CMD_CSUM_OFFLOAD_T {
	UINT_16 u2RxChecksum;	
	UINT_16 u2TxChecksum;	
} CMD_CSUM_OFFLOAD_T, *P_CMD_CSUM_OFFLOAD_T;

typedef struct _CMD_BASIC_CONFIG_T {
    UINT_8              ucNative80211;
    UINT_8              aucReserved[3];

    CMD_CSUM_OFFLOAD_T  rCsumOffload;
} CMD_BASIC_CONFIG_T, *P_CMD_BASIC_CONFIG_T;


typedef struct _CMD_MAC_MCAST_ADDR {
    UINT_32             u4NumOfGroupAddr;
    UINT_8              ucBssIndex;
    UINT_8              aucReserved[3];
    PARAM_MAC_ADDRESS   arAddress[MAX_NUM_GROUP_ADDR];
} CMD_MAC_MCAST_ADDR, *P_CMD_MAC_MCAST_ADDR, EVENT_MAC_MCAST_ADDR, *P_EVENT_MAC_MCAST_ADDR;

typedef struct _CMD_ACCESS_EEPROM {
    UINT_16             u2Offset;
    UINT_16             u2Data;
} CMD_ACCESS_EEPROM, *P_CMD_ACCESS_EEPROM, EVENT_ACCESS_EEPROM, *P_EVENT_ACCESS_EEPROM;

typedef struct _CMD_CUSTOM_NOA_PARAM_STRUC_T {
    UINT_32  u4NoaDurationMs;
    UINT_32  u4NoaIntervalMs;
    UINT_32  u4NoaCount;
} CMD_CUSTOM_NOA_PARAM_STRUC_T, *P_CMD_CUSTOM_NOA_PARAM_STRUC_T;

typedef struct _CMD_CUSTOM_OPPPS_PARAM_STRUC_T {
    UINT_32  u4CTwindowMs;
} CMD_CUSTOM_OPPPS_PARAM_STRUC_T, *P_CMD_CUSTOM_OPPPS_PARAM_STRUC_T;

typedef struct _CMD_CUSTOM_UAPSD_PARAM_STRUC_T {
    UINT_8  fgEnAPSD;
    UINT_8  fgEnAPSD_AcBe;
    UINT_8  fgEnAPSD_AcBk;
    UINT_8  fgEnAPSD_AcVo;
    UINT_8  fgEnAPSD_AcVi;
    UINT_8  ucMaxSpLen;
    UINT_8  aucResv[2];
} CMD_CUSTOM_UAPSD_PARAM_STRUC_T, *P_CMD_CUSTOM_UAPSD_PARAM_STRUC_T;

#if CFG_M0VE_BA_TO_DRIVER
typedef struct _CMD_RESET_BA_SCOREBOARD_T {
    UINT_8  ucflag;
    UINT_8  ucTID;
    UINT_8  aucMacAddr[PARAM_MAC_ADDR_LEN];
} CMD_RESET_BA_SCOREBOARD_T, *P_CMD_RESET_BA_SCOREBOARD_T;
#endif

typedef struct _EVENT_CONNECTION_STATUS {
    UINT_8  ucMediaStatus;
    UINT_8  ucReasonOfDisconnect;

    UINT_8  ucInfraMode;
    UINT_8  ucSsidLen;
    UINT_8  aucSsid[PARAM_MAX_LEN_SSID];
    UINT_8  aucBssid[PARAM_MAC_ADDR_LEN];
    UINT_8  ucAuthenMode;
    UINT_8  ucEncryptStatus;
    UINT_16 u2BeaconPeriod;
    UINT_16 u2AID;
    UINT_16 u2ATIMWindow;
    UINT_8  ucNetworkType;
    UINT_8  aucReserved[1];
    UINT_32 u4FreqInKHz;

#if CFG_ENABLE_WIFI_DIRECT
    UINT_8 aucInterfaceAddr[PARAM_MAC_ADDR_LEN];
#endif

} EVENT_CONNECTION_STATUS, *P_EVENT_CONNECTION_STATUS;

typedef struct _EVENT_NIC_CAPABILITY_T {
    UINT_16     u2ProductID;
    UINT_16     u2FwVersion;
    UINT_16     u2DriverVersion;
    UINT_8      ucHw5GBandDisabled;
    UINT_8      ucEepromUsed;
    UINT_8      aucMacAddr[6];
    UINT_8      ucEndianOfMacAddrNumber;
    UINT_8      ucReserved;

    UINT_8      ucRfVersion;
    UINT_8      ucPhyVersion;
    UINT_8      ucRfCalFail;
    UINT_8      ucBbCalFail;
    UINT_8      aucDateCode[16];
    UINT_32     u4FeatureFlag0;
    UINT_32     u4FeatureFlag1;
    UINT_32     u4CompileFlag0;
    UINT_32     u4CompileFlag1;
    UINT_8      aucReserved0[64];
} EVENT_NIC_CAPABILITY_T, *P_EVENT_NIC_CAPABILITY_T;

typedef struct _WLAN_BEACON_FRAME_BODY_T_LOCAL {
    
    UINT_32     au4Timestamp[2];            
    UINT_16     u2BeaconInterval;           
    UINT_16     u2CapInfo;                  
    UINT_8      aucInfoElem[MAX_IE_LENGTH]; 
    UINT_16     u2IELength;                 
} WLAN_BEACON_FRAME_BODY_T_LOCAL, *P_WLAN_BEACON_FRAME_BODY_T_LOCAL;

typedef struct _EVENT_SCAN_RESULT_T {
    INT_32                          i4RSSI;
    UINT_32                         u4LinkQuality;
    UINT_32                         u4DSConfig; 
    UINT_32                         u4DomainInfo; 
    UINT_32                         u4Reserved;
    UINT_8                          ucNetworkType;
    UINT_8                          ucOpMode;
    UINT_8                          aucBssid[MAC_ADDR_LEN];
    UINT_8                          aucRatesEx[PARAM_MAX_LEN_RATES_EX];
    WLAN_BEACON_FRAME_BODY_T_LOCAL  rBeaconFrameBody;
} EVENT_SCAN_RESULT_T, *P_EVENT_SCAN_RESULT_T;

typedef struct _EVENT_MIC_ERR_INFO {
    UINT_32     u4Flags;
} EVENT_MIC_ERR_INFO, *P_EVENT_MIC_ERR_INFO;

typedef struct _EVENT_ADD_KEY_DONE_INFO {
    UINT_8      ucBSSIndex;
    UINT_8      ucReserved;
    UINT_8      aucStaAddr[6];
} EVENT_ADD_KEY_DONE_INFO, *P_EVENT_ADD_KEY_DONE_INFO;

typedef struct _EVENT_PMKID_CANDIDATE_LIST_T {
    UINT_32     u4Version;            
    UINT_32     u4NumCandidates;      
    PARAM_PMKID_CANDIDATE_T   arCandidateList[1];
} EVENT_PMKID_CANDIDATE_LIST_T, *P_EVENT_PMKID_CANDIDATE_LIST_T;

typedef struct _EVENT_CMD_RESULT {
    UINT_8      ucCmdID;
    UINT_8      ucStatus;
    UINT_8      aucReserved[2];
} EVENT_CMD_RESULT, *P_EVENT_CMD_RESULT;

typedef struct _CMD_ACCESS_REG {
    UINT_32     u4Address;
    UINT_32     u4Data;
} CMD_ACCESS_REG, *P_CMD_ACCESS_REG;

#if CFG_AUTO_CHANNEL_SEL_SUPPORT

typedef struct _CMD_ACCESS_CHN_LOAD {
    UINT_32     u4Address;
    UINT_32     u4Data;
	UINT_16		u2Channel;
	UINT_8      aucReserved[2];
} CMD_ACCESS_CHN_LOAD, *P_ACCESS_CHN_LOAD;

#endif
typedef struct _CMD_DUMP_MEM {
    UINT_32     u4Address;
    UINT_32     u4Length;
    UINT_32     u4RemainLength;
    UINT_8      ucFragNum;
} CMD_DUMP_MEM, *P_CMD_DUMP_MEM;

typedef struct _EVENT_DUMP_MEM_T {
    UINT_32     u4Address;
    UINT_32     u4Length;
    UINT_32     u4RemainLength;
    UINT_8      ucFragNum;
    UINT_8      aucBuffer[1];
} EVENT_DUMP_MEM_T, *P_EVENT_DUMP_MEM_T;

typedef struct _CMD_SW_DBG_CTRL_T {
    UINT_32     u4Id;
    UINT_32     u4Data;
    
    UINT_32     u4DebugCnt[64];
} CMD_SW_DBG_CTRL_T, *P_CMD_SW_DBG_CTRL_T;


typedef struct _CMD_CHIP_CONFIG_T {
    UINT_16  u2Id;
    UINT_8   ucType;
    UINT_8   ucRespType;
    UINT_16  u2MsgSize;
    UINT_8   aucReserved0[2];
    UINT_8   aucCmd[CHIP_CONFIG_RESP_SIZE];
} CMD_CHIP_CONFIG_T, *P_CMD_CHIP_CONFIG_T;



typedef struct _CMD_LINK_ATTRIB {
    INT_8       cRssiTrigger;
    UINT_8      ucDesiredRateLen;
    UINT_16     u2DesiredRate[32];
    UINT_8      ucMediaStreamMode;
    UINT_8      aucReserved[1];
} CMD_LINK_ATTRIB, *P_CMD_LINK_ATTRIB;

typedef struct _CMD_NIC_POWER_CTRL {
    UINT_8  ucPowerMode;
    UINT_8  aucReserved[3];
} CMD_NIC_POWER_CTRL, *P_CMD_NIC_POWER_CTRL;

typedef struct _CMD_PS_PROFILE_T {
    UINT_8      ucBssIndex;
    UINT_8      ucPsProfile;
    UINT_8      aucReserved[2];
} CMD_PS_PROFILE_T, *P_CMD_PS_PROFILE_T;


#if 1
typedef struct _LINK_QUALITY_ {
    INT_8       cRssi; 
    INT_8       cLinkQuality;
    UINT_16     u2LinkSpeed;            
    UINT_8      ucMediumBusyPercentage; 
    UINT_8      ucIsLQ0Rdy;                 
} LINK_QUALITY, *P_LINK_QUALITY;

typedef struct _EVENT_LINK_QUALITY_V2 {
    LINK_QUALITY rLq[BSSID_NUM];
} EVENT_LINK_QUALITY_V2, *P_EVENT_LINK_QUALITY_V2;


typedef struct _EVENT_LINK_QUALITY {
    INT_8       cRssi;
    INT_8       cLinkQuality;
    UINT_16     u2LinkSpeed;
    UINT_8      ucMediumBusyPercentage;
} EVENT_LINK_QUALITY, *P_EVENT_LINK_QUALITY;
#endif

#if CFG_SUPPORT_P2P_RSSI_QUERY
typedef struct _EVENT_LINK_QUALITY_EX {
    INT_8       cRssi;
    INT_8       cLinkQuality;
    UINT_16     u2LinkSpeed;
    UINT_8      ucMediumBusyPercentage;
    UINT_8      ucIsLQ0Rdy;
    INT_8       cRssiP2P;   
    INT_8       cLinkQualityP2P;
    UINT_16     u2LinkSpeedP2P;
    UINT_8      ucMediumBusyPercentageP2P;
    UINT_8      ucIsLQ1Rdy;
} EVENT_LINK_QUALITY_EX, *P_EVENT_LINK_QUALITY_EX;
#endif





typedef struct _EVENT_STATISTICS {
    LARGE_INTEGER   rTransmittedFragmentCount;
    LARGE_INTEGER   rMulticastTransmittedFrameCount;
    LARGE_INTEGER   rFailedCount;
    LARGE_INTEGER   rRetryCount;
    LARGE_INTEGER   rMultipleRetryCount;
    LARGE_INTEGER   rRTSSuccessCount;
    LARGE_INTEGER   rRTSFailureCount;
    LARGE_INTEGER   rACKFailureCount;
    LARGE_INTEGER   rFrameDuplicateCount;
    LARGE_INTEGER   rReceivedFragmentCount;
    LARGE_INTEGER   rMulticastReceivedFrameCount;
    LARGE_INTEGER   rFCSErrorCount;
} EVENT_STATISTICS, *P_EVENT_STATISTICS;

typedef struct _EVENT_SLEEPY_INFO_T {
    UINT_8      ucSleepyState;
    UINT_8      aucReserved[3];
} EVENT_SLEEPY_INFO_T, *P_EVENT_SLEEPY_INFO_T;

typedef struct _EVENT_ACTIVATE_STA_REC_T {
    UINT_8      aucMacAddr[6];
    UINT_8      ucStaRecIdx;
    UINT_8      ucBssIndex;
} EVENT_ACTIVATE_STA_REC_T, *P_EVENT_ACTIVATE_STA_REC_T;

typedef struct _EVENT_DEACTIVATE_STA_REC_T {
    UINT_8      ucStaRecIdx;
    UINT_8      aucReserved[3];
} EVENT_DEACTIVATE_STA_REC_T, *P_EVENT_DEACTIVATE_STA_REC_T;

typedef struct _CMD_BT_OVER_WIFI {
    UINT_8              ucAction;       
    UINT_8              ucChannelNum;
    PARAM_MAC_ADDRESS   rPeerAddr;
    UINT_16             u2BeaconInterval;
    UINT_8              ucTimeoutDiscovery;
    UINT_8              ucTimeoutInactivity;
    UINT_8              ucRole;
    UINT_8              PAL_Capabilities;
    UINT_8              cMaxTxPower;
    UINT_8              ucChannelBand;
    UINT_8              ucReserved[1];
} CMD_BT_OVER_WIFI, *P_CMD_BT_OVER_WIFI;

typedef struct _EVENT_BT_OVER_WIFI {
    UINT_8      ucLinkStatus;
    UINT_8      ucSelectedChannel;
    INT_8       cRSSI;
    UINT_8      ucReserved[1];
} EVENT_BT_OVER_WIFI, *P_EVENT_BT_OVER_WIFI;

typedef struct _CMD_SUBBAND_INFO {
    UINT_8              ucRegClass;
    UINT_8              ucBand;
    UINT_8              ucChannelSpan;
    UINT_8              ucFirstChannelNum;
    UINT_8              ucNumChannels;
    UINT_8              aucReserved[3];
} CMD_SUBBAND_INFO, *P_CMD_SUBBAND_INFO;

typedef struct _CMD_SET_DOMAIN_INFO_T {
    UINT_16             u2CountryCode;
    UINT_16             u2Reserved;
    CMD_SUBBAND_INFO    rSubBand[6];

    UINT_8              uc2G4Bandwidth; 
    UINT_8              uc5GBandwidth;  
    UINT_8              aucReserved[2];
} CMD_SET_DOMAIN_INFO_T, *P_CMD_SET_DOMAIN_INFO_T;

#if CFG_SUPPORT_PWR_LIMIT_COUNTRY

typedef struct _CMD_CHANNEL_POWER_LIMIT {
    UINT_8                 ucCentralCh;
	INT_8                  cPwrLimitCCK;
    INT_8                  cPwrLimit20;
    INT_8                  cPwrLimit40;
    INT_8                  cPwrLimit80;
    INT_8                  cPwrLimit160;
    UINT_8                 ucFlag;
    UINT_8                 aucReserved[1];
}CMD_CHANNEL_POWER_LIMIT, *P_CMD_CHANNEL_POWER_LIMIT;


typedef struct _CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT_T {
    UINT_16                    u2CountryCode;
    UINT_8                     ucCountryFlag;
    UINT_8                     ucNum;
    UINT_8                     aucReserved[4];
    CMD_CHANNEL_POWER_LIMIT    rChannelPowerLimit[1];	
} CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT_T, *P_CMD_SET_COUNTRY_CHANNEL_POWER_LIMIT_T;

#endif

typedef struct _IPV4_NETWORK_ADDRESS {
    UINT_8      aucIpAddr[4];
} IPV4_NETWORK_ADDRESS, *P_IPV4_NETWORK_ADDRESS;

typedef struct _CMD_SET_NETWORK_ADDRESS_LIST {
    UINT_8      ucBssIndex;
    UINT_8      ucAddressCount;
    UINT_8      ucReserved[2];
    IPV4_NETWORK_ADDRESS arNetAddress[1];
} CMD_SET_NETWORK_ADDRESS_LIST, *P_CMD_SET_NETWORK_ADDRESS_LIST;

typedef struct _PATTERN_DESCRIPTION {
    UINT_8      fgCheckBcA1;
    UINT_8      fgCheckMcA1;
    UINT_8      ePatternHeader;
    UINT_8      fgAndOp;
    UINT_8      fgNotOp;
    UINT_8      ucPatternMask;
    UINT_16     u2PatternOffset;
    UINT_8      aucPattern[8];
} PATTERN_DESCRIPTION, *P_PATTERN_DESCRIPTION;

typedef struct _CMD_RAW_PATTERN_CONFIGURATION_T {
    PATTERN_DESCRIPTION arPatternDesc[4];
} CMD_RAW_PATTERN_CONFIGURATION_T, *P_CMD_RAW_PATTERN_CONFIGURATION_T;

typedef struct _CMD_PATTERN_FUNC_CONFIG {
    BOOLEAN      fgBcA1En;
    BOOLEAN      fgMcA1En;
    BOOLEAN      fgBcA1MatchDrop;
    BOOLEAN      fgMcA1MatchDrop;
} CMD_PATTERN_FUNC_CONFIG, *P_CMD_PATTERN_FUNC_CONFIG;


typedef struct _EVENT_TX_DONE_T {
    UINT_8      ucPacketSeq;
    UINT_8      ucStatus;
    UINT_16     u2SequenceNumber;
    UINT_8      ucWlanIndex;
    UINT_8      aucReserved1[3];
    UINT_32     au4Reserved2;
    UINT_32     au4Reserved3;
} EVENT_TX_DONE_T, *P_EVENT_TX_DONE_T;

typedef struct _CMD_BSS_ACTIVATE_CTRL {
    UINT_8      ucBssIndex;
    UINT_8      ucActive;
    UINT_8      ucNetworkType;
    UINT_8      ucOwnMacAddrIndex;
    UINT_8      aucBssMacAddr[6];
    UINT_8      ucBMCWlanIndex;
    UINT_8      ucReserved;
} CMD_BSS_ACTIVATE_CTRL, *P_CMD_BSS_ACTIVATE_CTRL;

typedef struct _CMD_SET_BSS_RLM_PARAM_T {
    UINT_8      ucBssIndex;
    UINT_8      ucRfBand;
    UINT_8      ucPrimaryChannel;
    UINT_8      ucRfSco;
    UINT_8      ucErpProtectMode;
    UINT_8      ucHtProtectMode;
    UINT_8      ucGfOperationMode;
    UINT_8      ucTxRifsMode;
    UINT_16     u2HtOpInfo3;
    UINT_16     u2HtOpInfo2;
    UINT_8      ucHtOpInfo1;
    UINT_8      ucUseShortPreamble;
    UINT_8      ucUseShortSlotTime;
    UINT_8      ucVhtChannelWidth;
    UINT_8      ucVhtChannelFrequencyS1;
    UINT_8      ucVhtChannelFrequencyS2;
    UINT_16     u2VhtBasicMcsSet;
} CMD_SET_BSS_RLM_PARAM_T, *P_CMD_SET_BSS_RLM_PARAM_T;

typedef struct _CMD_SET_BSS_INFO {
    UINT_8  ucBssIndex;
    UINT_8  ucConnectionState;
    UINT_8  ucCurrentOPMode;
    UINT_8  ucSSIDLen;
    UINT_8  aucSSID[32];
    UINT_8  aucBSSID[6];
    UINT_8  ucIsQBSS;
    UINT_8  ucReserved1;
    UINT_16 u2OperationalRateSet;
    UINT_16 u2BSSBasicRateSet;
    UINT_8  ucStaRecIdxOfAP;
    UINT_16 u2HwDefaultFixedRateCode;
    UINT_8  ucNonHTBasicPhyType; 
    UINT_8  ucAuthMode;
    UINT_8  ucEncStatus;
    UINT_8  ucPhyTypeSet;
    UINT_8  ucWapiMode;
    UINT_8  ucIsApMode;
    UINT_8  ucBMCWlanIndex;
    UINT_8  ucHiddenSsidMode;
    UINT_8  ucDisconnectDetectTh;
    UINT_32 u4PrivateData;
    CMD_SET_BSS_RLM_PARAM_T rBssRlmParam;
} CMD_SET_BSS_INFO, *P_CMD_SET_BSS_INFO;

typedef enum _ENUM_RTS_POLICY_T {
    RTS_POLICY_AUTO,
    RTS_POLICY_STATIC_BW,
    RTS_POLICY_DYNAMIC_BW,
    RTS_POLICY_LEGACY,
    RTS_POLICY_NO_RTS
} ENUM_RTS_POLICY;

typedef struct _CMD_UPDATE_STA_RECORD_T {
    UINT_8  ucStaIndex;
    UINT_8  ucStaType;
	
	UINT_8 aucMacAddr[MAC_ADDR_LEN];

    UINT_16 u2AssocId;
    UINT_16 u2ListenInterval;
    UINT_8  ucBssIndex;               
    UINT_8  ucDesiredPhyTypeSet;
    UINT_16 u2DesiredNonHTRateSet;

    UINT_16 u2BSSBasicRateSet;
    UINT_8  ucIsQoS;
    UINT_8  ucIsUapsdSupported;
    UINT_8  ucStaState;
    UINT_8  ucMcsSet;
    UINT_8  ucSupMcs32;
    UINT_8  aucReserved1[1];

    UINT_8  aucRxMcsBitmask[10];
    UINT_16 u2RxHighestSupportedRate;
    UINT_32 u4TxRateInfo;

    UINT_16 u2HtCapInfo;
    UINT_16 u2HtExtendedCap;
    UINT_32 u4TxBeamformingCap;

    UINT_8  ucAmpduParam;
    UINT_8  ucAselCap;
    UINT_8  ucRCPI;
    UINT_8  ucNeedResp;
    UINT_8  ucUapsdAc;                
    UINT_8  ucUapsdSp;                
    UINT_8  ucWlanIndex;              
    UINT_8  ucBMCWlanIndex;           

    UINT_32 u4VhtCapInfo;
    UINT_16 u2VhtRxMcsMap;
    UINT_16 u2VhtRxHighestSupportedDataRate;
    UINT_16 u2VhtTxMcsMap;
    UINT_16 u2VhtTxHighestSupportedDataRate;
    UINT_8  ucRtsPolicy;              
    UINT_8  aucReserved2[1];

    UINT_8  ucTrafficDataType;        
    UINT_8  ucTxGfMode;               
    UINT_8  ucTxSgiMode;              
    UINT_8  ucTxStbcMode;            
    UINT_16 u2HwDefaultFixedRateCode;
    UINT_8  ucTxAmpdu;                
    UINT_8  ucRxAmpdu;                
    UINT_32 u4FixedPhyRate;           
    UINT_16 u2MaxLinkSpeed;           
    UINT_16 u2MinLinkSpeed;

    UINT_32  u4Flags;
    
    UINT_8  ucTxBaSize;
    UINT_8  ucRxBaSize;
    
    UINT_8  aucReserved4[30];
} CMD_UPDATE_STA_RECORD_T, *P_CMD_UPDATE_STA_RECORD_T;

typedef struct _CMD_REMOVE_STA_RECORD_T {
    UINT_8  ucActionType;
    UINT_8  ucStaIndex;
    UINT_8  ucBssIndex;
    UINT_8  ucReserved;
} CMD_REMOVE_STA_RECORD_T, *P_CMD_REMOVE_STA_RECORD_T;

typedef struct _CMD_INDICATE_PM_BSS_CREATED_T {
    UINT_8  ucBssIndex;
    UINT_8  ucDtimPeriod;
    UINT_16 u2BeaconInterval;
    UINT_16 u2AtimWindow;
    UINT_8  aucReserved[2];
} CMD_INDICATE_PM_BSS_CREATED, *P_CMD_INDICATE_PM_BSS_CREATED;

typedef struct _CMD_INDICATE_PM_BSS_CONNECTED_T {
    UINT_8  ucBssIndex;
    UINT_8  ucDtimPeriod;
    UINT_16 u2AssocId;
    UINT_16 u2BeaconInterval;
    UINT_16 u2AtimWindow;
    UINT_8  fgIsUapsdConnection;
    UINT_8  ucBmpDeliveryAC;
    UINT_8  ucBmpTriggerAC;
    UINT_8  aucReserved[1];
} CMD_INDICATE_PM_BSS_CONNECTED, *P_CMD_INDICATE_PM_BSS_CONNECTED;

typedef struct _CMD_INDICATE_PM_BSS_ABORT {
    UINT_8  ucBssIndex;
    UINT_8  aucReserved[3];
} CMD_INDICATE_PM_BSS_ABORT, *P_CMD_INDICATE_PM_BSS_ABORT;

typedef struct _CMD_BEACON_TEMPLATE_UPDATE {
	UINT_8 ucUpdateMethod;
    UINT_8    ucBssIndex;
    UINT_8    aucReserved[2];
    UINT_16   u2Capability;
    UINT_16   u2IELen;
    UINT_8    aucIE[MAX_IE_LENGTH];
} CMD_BEACON_TEMPLATE_UPDATE, *P_CMD_BEACON_TEMPLATE_UPDATE;

typedef struct _CMD_SET_WMM_PS_TEST_STRUC_T {
    UINT_8  ucBssIndex;
    UINT_8  bmfgApsdEnAc;           
    UINT_8  ucIsEnterPsAtOnce;      
    UINT_8  ucIsDisableUcTrigger;   
} CMD_SET_WMM_PS_TEST_STRUC_T, *P_CMD_SET_WMM_PS_TEST_STRUC_T;


typedef struct _CHANNEL_INFO_T {
    UINT_8  ucBand;
    UINT_8  ucChannelNum;
} CHANNEL_INFO_T, *P_CHANNEL_INFO_T;

typedef struct _CMD_SCAN_REQ_T {
    UINT_8          ucSeqNum;
    UINT_8          ucBssIndex;
    UINT_8          ucScanType;
    UINT_8          ucSSIDType;     
    UINT_8          ucSSIDLength;
    UINT_8          ucNumProbeReq;
    UINT_16         u2ChannelMinDwellTime;
    UINT_16         u2ChannelDwellTime;
    UINT_16         u2TimeoutValue;
    UINT_8          aucSSID[32];
    UINT_8          ucChannelType;
    UINT_8          ucChannelListNum;
    UINT_8          aucReserved[2];
    CHANNEL_INFO_T  arChannelList[32];
    UINT_16         u2IELen;
    UINT_8          aucIE[MAX_IE_LENGTH];
} CMD_SCAN_REQ, *P_CMD_SCAN_REQ;

typedef struct _CMD_SCAN_REQ_V2_T {
    UINT_8          ucSeqNum;
    UINT_8          ucBssIndex;
    UINT_8          ucScanType;
    UINT_8          ucSSIDType;
    UINT_8          ucSSIDNum;
    UINT_8          ucNumProbeReq;
    UINT_8          aucReserved[2];
    PARAM_SSID_T    arSSID[4];
    UINT_16         u2ProbeDelayTime;
    UINT_16         u2ChannelDwellTime;
    UINT_16         u2TimeoutValue;
    UINT_8          ucChannelType;
    UINT_8          ucChannelListNum;
    CHANNEL_INFO_T  arChannelList[32];
    UINT_16         u2IELen;
    UINT_8          aucIE[MAX_IE_LENGTH];
} CMD_SCAN_REQ_V2, *P_CMD_SCAN_REQ_V2;

typedef struct _CMD_SCAN_CANCEL_T {
    UINT_8          ucSeqNum;
    UINT_8          ucIsExtChannel;     
    UINT_8          aucReserved[2];
} CMD_SCAN_CANCEL, *P_CMD_SCAN_CANCEL;


typedef struct _EVENT_SCAN_DONE_T {
    UINT_8          ucSeqNum;
    UINT_8          ucSparseChannelValid;
    CHANNEL_INFO_T  rSparseChannel;
    
    UINT_8          ucCompleteChanCount;
    UINT_8          ucCurrentState;
    UINT_8          ucScanDoneVersion;
    
    UINT_8          ucReserved;
    UINT_32         u4ScanDurBcnCnt;
    UINT_8          fgIsPNOenabled;
    UINT_8          aucReserving[3];
} EVENT_SCAN_DONE, *P_EVENT_SCAN_DONE;


#if CFG_SUPPORT_BATCH_SCAN
typedef struct _CMD_BATCH_REQ_T {
    UINT_8      ucSeqNum;
    UINT_8      ucNetTypeIndex;
    UINT_8      ucCmd;                
    UINT_8      ucMScan;              
    UINT_8      ucBestn;              
	UINT_8 ucRtt;		
    UINT_8      ucChannel;            
    UINT_8      ucChannelType;
    UINT_8      ucChannelListNum;
    UINT_8      aucReserved[3];
    UINT_32     u4Scanfreq;           
    CHANNEL_INFO_T arChannelList[32]; 
} CMD_BATCH_REQ_T, *P_CMD_BATCH_REQ_T;


typedef struct _EVENT_BATCH_RESULT_ENTRY_T {
    UINT_8      aucBssid[MAC_ADDR_LEN];
    UINT_8      aucSSID[ELEM_MAX_LEN_SSID];
    UINT_8      ucSSIDLen;
    INT_8       cRssi;
    UINT_32     ucFreq;
    UINT_32     u4Age;
    UINT_32     u4Dist;
    UINT_32     u4Distsd;
} EVENT_BATCH_RESULT_ENTRY_T, *P_EVENT_BATCH_RESULT_ENTRY_T;

typedef struct _EVENT_BATCH_RESULT_T {
    UINT_8      ucScanCount;
    UINT_8      aucReserved[3];
	EVENT_BATCH_RESULT_ENTRY_T arBatchResult[12];	
} EVENT_BATCH_RESULT_T, *P_EVENT_BATCH_RESULT_T;
#endif

typedef struct _CMD_CH_PRIVILEGE_T {
    UINT_8      ucBssIndex;
    UINT_8      ucTokenID;
    UINT_8      ucAction;
    UINT_8      ucPrimaryChannel;
    UINT_8      ucRfSco;
    UINT_8      ucRfBand;
    UINT_8      ucRfChannelWidth;   
    UINT_8      ucRfCenterFreqSeg1; 
    UINT_8      ucRfCenterFreqSeg2; 
    UINT_8      ucReqType;
    UINT_8      aucReserved[2];
    UINT_32     u4MaxInterval;      
} CMD_CH_PRIVILEGE_T, *P_CMD_CH_PRIVILEGE_T;

typedef struct _CMD_TX_PWR_T {
    INT_8       cTxPwr2G4Cck;       
    INT_8       cTxPwr2G4Dsss;      
    INT_8       acReserved[2];

    INT_8       cTxPwr2G4OFDM_BPSK;
    INT_8       cTxPwr2G4OFDM_QPSK;
    INT_8       cTxPwr2G4OFDM_16QAM;
    INT_8       cTxPwr2G4OFDM_Reserved;
    INT_8       cTxPwr2G4OFDM_48Mbps;
    INT_8       cTxPwr2G4OFDM_54Mbps;

    INT_8       cTxPwr2G4HT20_BPSK;
    INT_8       cTxPwr2G4HT20_QPSK;
    INT_8       cTxPwr2G4HT20_16QAM;
    INT_8       cTxPwr2G4HT20_MCS5;
    INT_8       cTxPwr2G4HT20_MCS6;
    INT_8       cTxPwr2G4HT20_MCS7;

    INT_8       cTxPwr2G4HT40_BPSK;
    INT_8       cTxPwr2G4HT40_QPSK;
    INT_8       cTxPwr2G4HT40_16QAM;
    INT_8       cTxPwr2G4HT40_MCS5;
    INT_8       cTxPwr2G4HT40_MCS6;
    INT_8       cTxPwr2G4HT40_MCS7;

    INT_8       cTxPwr5GOFDM_BPSK;
    INT_8       cTxPwr5GOFDM_QPSK;
    INT_8       cTxPwr5GOFDM_16QAM;
    INT_8       cTxPwr5GOFDM_Reserved;
    INT_8       cTxPwr5GOFDM_48Mbps;
    INT_8       cTxPwr5GOFDM_54Mbps;

    INT_8       cTxPwr5GHT20_BPSK;
    INT_8       cTxPwr5GHT20_QPSK;
    INT_8       cTxPwr5GHT20_16QAM;
    INT_8       cTxPwr5GHT20_MCS5;
    INT_8       cTxPwr5GHT20_MCS6;
    INT_8       cTxPwr5GHT20_MCS7;

    INT_8       cTxPwr5GHT40_BPSK;
    INT_8       cTxPwr5GHT40_QPSK;
    INT_8       cTxPwr5GHT40_16QAM;
    INT_8       cTxPwr5GHT40_MCS5;
    INT_8       cTxPwr5GHT40_MCS6;
    INT_8       cTxPwr5GHT40_MCS7;
} CMD_TX_PWR_T, *P_CMD_TX_PWR_T;

typedef struct _CMD_TX_AC_PWR_T {
    INT_8       ucBand;
#if 0
    INT_8       c11AcTxPwr_BPSK;
    INT_8       c11AcTxPwr_QPSK;
    INT_8       c11AcTxPwr_16QAM;
    INT_8       c11AcTxPwr_MCS5_MCS6;
    INT_8       c11AcTxPwr_MCS7;
    INT_8       c11AcTxPwr_MCS8;
    INT_8       c11AcTxPwr_MCS9;
    INT_8       c11AcTxPwrVht40_OFFSET;
    INT_8       c11AcTxPwrVht80_OFFSET;
    INT_8       c11AcTxPwrVht160_OFFSET;
#else 
    AC_PWR_SETTING_STRUCT  rAcPwr;
#endif
} CMD_TX_AC_PWR_T, *P_CMD_TX_AC_PWR_T;


typedef struct _CMD_RSSI_PATH_COMPASATION_T {
    INT_8       c2GRssiCompensation;
    INT_8       c5GRssiCompensation;
} CMD_RSSI_PATH_COMPASATION_T, *P_CMD_RSSI_PATH_COMPASATION_T;
typedef struct _CMD_5G_PWR_OFFSET_T {
    INT_8       cOffsetBand0;       
    INT_8       cOffsetBand1;       
    INT_8       cOffsetBand2;       
    INT_8       cOffsetBand3;       
    INT_8       cOffsetBand4;       
    INT_8       cOffsetBand5;       
    INT_8       cOffsetBand6;       
    INT_8       cOffsetBand7;       
} CMD_5G_PWR_OFFSET_T, *P_CMD_5G_PWR_OFFSET_T;

typedef struct _CMD_PWR_PARAM_T {
    UINT_32     au4Data[28];
    UINT_32     u4RefValue1;
    UINT_32     u4RefValue2;
} CMD_PWR_PARAM_T, *P_CMD_PWR_PARAM_T;


typedef struct _CMD_PHY_PARAM_T {
    UINT_8      aucData[144];           
} CMD_PHY_PARAM_T, *P_CMD_PHY_PARAM_T;

typedef struct _CMD_AUTO_POWER_PARAM_T {
    UINT_8      ucType;        
    UINT_8      ucBssIndex;
    UINT_8      aucReserved[2];
    UINT_8      aucLevelRcpiTh[3];
    UINT_8      aucReserved2[1];
    INT_8       aicLevelPowerOffset[3];     
    UINT_8      aucReserved3[1];
    UINT_8      aucReserved4[8];
} CMD_AUTO_POWER_PARAM_T, *P_CMD_AUTO_POWER_PARAM_T;


typedef struct _EVENT_CH_PRIVILEGE_T {
    UINT_8          ucBssIndex;
    UINT_8          ucTokenID;
    UINT_8          ucStatus;
    UINT_8          ucPrimaryChannel;
    UINT_8          ucRfSco;
    UINT_8          ucRfBand;
    UINT_8          ucRfChannelWidth;   
    UINT_8          ucRfCenterFreqSeg1; 
    UINT_8          ucRfCenterFreqSeg2; 
    UINT_8          ucReqType;
    UINT_8          aucReserved[2];
    UINT_32         u4GrantInterval;    
} EVENT_CH_PRIVILEGE_T, *P_EVENT_CH_PRIVILEGE_T;

typedef struct _EVENT_BSS_BEACON_TIMEOUT_T {
    UINT_8      ucBssIndex;
    UINT_8      ucReasonCode;
    UINT_8      aucReserved[2];
} EVENT_BSS_BEACON_TIMEOUT_T, *P_EVENT_BSS_BEACON_TIMEOUT_T;

typedef struct _EVENT_STA_AGING_TIMEOUT_T {
    UINT_8          ucStaRecIdx;
    UINT_8          aucReserved[3];
} EVENT_STA_AGING_TIMEOUT_T, *P_EVENT_STA_AGING_TIMEOUT_T;

typedef struct _EVENT_NOA_TIMING_T {
    UINT_8      ucIsInUse;              
    UINT_8      ucCount;                
    UINT_8      aucReserved[2];

    UINT_32     u4Duration;             
    UINT_32     u4Interval;             
    UINT_32     u4StartTime;            
} EVENT_NOA_TIMING_T, *P_EVENT_NOA_TIMING_T;

typedef struct _EVENT_UPDATE_NOA_PARAMS_T {
    UINT_8      ucBssIndex;
    UINT_8      aucReserved[2];
    UINT_8      ucEnableOppPS;
    UINT_16     u2CTWindow;

    UINT_8              ucNoAIndex;
    UINT_8              ucNoATimingCount; 
	EVENT_NOA_TIMING_T arEventNoaTiming[8 ];
} EVENT_UPDATE_NOA_PARAMS_T, *P_EVENT_UPDATE_NOA_PARAMS_T;

typedef struct _EVENT_AP_OBSS_STATUS_T {
    UINT_8      ucBssIndex;
    UINT_8      ucObssErpProtectMode;
    UINT_8      ucObssHtProtectMode;
    UINT_8      ucObssGfOperationMode;
    UINT_8      ucObssRifsOperationMode;
    UINT_8      ucObssBeaconForcedTo20M;
    UINT_8      aucReserved[2];
} EVENT_AP_OBSS_STATUS_T, *P_EVENT_AP_OBSS_STATUS_T;

typedef struct _EVENT_DEBUG_MSG_T {
    UINT_16  u2DebugMsgId;
    UINT_8   ucMsgType;
    UINT_8   ucFlags;  
    UINT_32  u4Value; 
    UINT_16  u2MsgSize;
    UINT_8   aucReserved0[2];
    UINT_8   aucMsg[1];
} EVENT_DEBUG_MSG_T, *P_EVENT_DEBUG_MSG_T;

typedef struct _CMD_EDGE_TXPWR_LIMIT_T {
    INT_8       cBandEdgeMaxPwrCCK;
    INT_8       cBandEdgeMaxPwrOFDM20;
    INT_8       cBandEdgeMaxPwrOFDM40;
    INT_8       cBandEdgeMaxPwrOFDM80;
} CMD_EDGE_TXPWR_LIMIT_T, *P_CMD_EDGE_TXPWR_LIMIT_T;

typedef struct _CMD_POWER_OFFSET_T {
	UINT_8 ucBand;		
    UINT_8       ucSubBandOffset[MAX_SUBBAND_NUM_5G];    
    UINT_8       aucReverse[3];
    
} CMD_POWER_OFFSET_T, *P_CMD_POWER_OFFSET_T;

typedef struct _CMD_NVRAM_SETTING_T {

    WIFI_CFG_PARAM_STRUCT   rNvramSettings;

} CMD_NVRAM_SETTING_T, *P_CMD_NVRAM_SETTING_T;

#if CFG_SUPPORT_TDLS
typedef struct _CMD_TDLS_CH_SW_T {
	BOOLEAN fgIsTDLSChSwProhibit;
} CMD_TDLS_CH_SW_T, *P_CMD_TDLS_CH_SW_T;
#endif

typedef struct _CMD_SET_DEVICE_MODE_T {
    UINT_16     u2ChipID;
    UINT_16     u2Mode;
} CMD_SET_DEVICE_MODE_T, *P_CMD_SET_DEVICE_MODE_T;


#if CFG_SUPPORT_RDD_TEST_MODE
typedef struct _CMD_RDD_CH_T {
    UINT_8      ucRddTestMode;
    UINT_8      ucRddShutCh;
    UINT_8      ucRddStartCh;
    UINT_8      ucRddStopCh;
    UINT_8      ucRddDfs;
    UINT_8      ucReserved;
    UINT_8      ucReserved1;
    UINT_8      ucReserved2;
} CMD_RDD_CH_T, *P_CMD_RDD_CH_T;

typedef struct _EVENT_RDD_STATUS_T {
    UINT_8  ucRddStatus;
    UINT_8  aucReserved[3];
} EVENT_RDD_STATUS_T, *P_EVENT_RDD_STATUS_T;
#endif

typedef struct _EVENT_ICAP_STATUS_T {
    UINT_8      ucRddStatus;
    UINT_8      aucReserved[3];
    UINT_32     u4StartAddress;
    UINT_32     u4IcapSieze;
} EVENT_ICAP_STATUS_T, *P_EVENT_ICAP_STATUS_T;

typedef struct _CMD_SET_TXPWR_CTRL_T {
	INT_8 c2GLegacyStaPwrOffset;	
    INT_8    c2GHotspotPwrOffset;
    INT_8    c2GP2pPwrOffset;
    INT_8    c2GBowPwrOffset;
	INT_8 c5GLegacyStaPwrOffset;	
    INT_8    c5GHotspotPwrOffset;
    INT_8    c5GP2pPwrOffset;
    INT_8    c5GBowPwrOffset;
    UINT_8  ucConcurrencePolicy;   
    INT_8    acReserved1[3];            

    
	INT_8 acTxPwrLimit2G[14];	
    INT_8    acTxPwrLimit5G[4];       
    INT_8    acReserved2[2];            
} CMD_SET_TXPWR_CTRL_T, *P_CMD_SET_TXPWR_CTRL_T;

typedef enum _ENUM_NLO_CIPHER_ALGORITHM {
    NLO_CIPHER_ALGO_NONE            = 0x00,
    NLO_CIPHER_ALGO_WEP40           = 0x01,
    NLO_CIPHER_ALGO_TKIP            = 0x02,
    NLO_CIPHER_ALGO_CCMP            = 0x04,
    NLO_CIPHER_ALGO_WEP104          = 0x05,
    NLO_CIPHER_ALGO_WPA_USE_GROUP   = 0x100,
    NLO_CIPHER_ALGO_RSN_USE_GROUP   = 0x100,
    NLO_CIPHER_ALGO_WEP             = 0x101,
} ENUM_NLO_CIPHER_ALGORITHM, *P_ENUM_NLO_CIPHER_ALGORITHM;

typedef enum _ENUM_NLO_AUTH_ALGORITHM {
    NLO_AUTH_ALGO_80211_OPEN        = 1,
    NLO_AUTH_ALGO_80211_SHARED_KEY  = 2,
    NLO_AUTH_ALGO_WPA               = 3,
    NLO_AUTH_ALGO_WPA_PSK           = 4,
    NLO_AUTH_ALGO_WPA_NONE          = 5,
    NLO_AUTH_ALGO_RSNA              = 6,
    NLO_AUTH_ALGO_RSNA_PSK          = 7,
} ENUM_NLO_AUTH_ALGORITHM, *P_ENUM_NLO_AUTH_ALGORITHM;

typedef struct _NLO_NETWORK {
    UINT_8      ucNumChannelHint[4];
    UINT_8      ucSSIDLength;
    UINT_8      ucCipherAlgo;
    UINT_16     u2AuthAlgo;
    UINT_8      aucSSID[32];
} NLO_NETWORK, *P_NLO_NETWORK;

typedef struct _CMD_NLO_REQ {
    UINT_8      ucSeqNum;
    UINT_8      ucBssIndex;
    UINT_8      fgStopAfterIndication;
    UINT_8      ucFastScanIteration;
    UINT_16     u2FastScanPeriod;
    UINT_16     u2SlowScanPeriod;
    UINT_8      ucEntryNum;
    UINT_8      ucFlag; 
    UINT_16     u2IELen;
    NLO_NETWORK arNetworkList[16];
    UINT_8      aucIE[0];
} CMD_NLO_REQ, *P_CMD_NLO_REQ;

typedef struct _CMD_NLO_CANCEL_T {
    UINT_8  ucSeqNum;
    UINT_8  ucBssIndex;
    UINT_8  aucReserved[2];
} CMD_NLO_CANCEL, *P_CMD_NLO_CANCEL;

typedef struct _EVENT_NLO_DONE_T {
    UINT_8      ucSeqNum;
    UINT_8      ucStatus;
    UINT_8      aucReserved[2];
} EVENT_NLO_DONE_T, *P_EVENT_NLO_DONE_T;

#if CFG_SUPPORT_BUILD_DATE_CODE
typedef struct _CMD_GET_BUILD_DATE_CODE {
    UINT_8  aucReserved[4];
} CMD_GET_BUILD_DATE_CODE, *P_CMD_GET_BUILD_DATE_CODE;

typedef struct _EVENT_BUILD_DATE_CODE {
    UINT_8      aucDateCode[16];
} EVENT_BUILD_DATE_CODE, *P_EVENT_BUILD_DATE_CODE;
#endif

typedef struct _CMD_GET_STA_STATISTICS_T {
    UINT_8  ucIndex;
    UINT_8  ucFlags;
    UINT_8  ucReadClear;
    UINT_8  aucReserved0[1];
    UINT_8  aucMacAddr[MAC_ADDR_LEN];
    UINT_8  aucReserved1[2];
    UINT_8  aucReserved2[16];
} CMD_GET_STA_STATISTICS_T, *P_CMD_GET_STA_STATISTICS_T;

typedef struct _EVENT_STA_STATISTICS_T {
    
	
	 
	
	
	

    
    UINT_8      ucVersion;
    UINT_8		aucReserved1[3];
    UINT_32     u4Flags; 

    UINT_8      ucStaRecIdx;
    UINT_8      ucNetworkTypeIndex;
    UINT_8      ucWTEntry;
    UINT_8		aucReserved4[1];

    UINT_8      ucMacAddr[MAC_ADDR_LEN];
    UINT_8      ucPer;          
    UINT_8      ucRcpi;

    UINT_32     u4PhyMode;          
	UINT_16 u2LinkSpeed;	
    UINT_8      ucLinkQuality;          
    UINT_8      ucLinkReserved;            

    UINT_32      u4TxCount;
    UINT_32      u4TxFailCount;
    UINT_32      u4TxLifeTimeoutCount;
    UINT_32      u4TxDoneAirTime;
    UINT_32      u4TransmitCount;       
    UINT_32      u4TransmitFailCount;   


    UINT_8      aucReserved[56];
} EVENT_STA_STATISTICS_T, *P_EVENT_STA_STATISTICS_T;
#if CFG_AUTO_CHANNEL_SEL_SUPPORT


typedef struct _CMD_GET_CHN_LOAD_T {
    UINT_8  	ucIndex;
    UINT_8  	ucFlags;
    UINT_8  	ucReadClear;
    UINT_8  	aucReserved0[1];
    UINT_8  	ucChannel;
    UINT_16 	u2ChannelLoad;
    UINT_8  	aucReserved1[1];
    UINT_8  	aucReserved2[16];
} CMD_GET_CHN_LOAD_T, *P_CMD_GET_CHN_LOAD_T;

typedef struct _EVENT_CHN_LOAD_T {
   
    UINT_8      ucVersion;
    UINT_8	aucReserved1[3];
    UINT_32     u4Flags; 
    UINT_8      ucChannel;
    UINT_16     u2ChannelLoad;
    UINT_8	aucReserved4[1];
    UINT_8      aucReserved[64];
} EVENT_CHN_LOAD_T, *P_EVENT_CHN_LOAD_T;
typedef struct _CMD_GET_LTE_SAFE_CHN_T {
    UINT_8  	ucIndex;
    UINT_8  	ucFlags;
    UINT_8  	aucReserved0[2];
    UINT_8  	aucReserved2[16];
} CMD_GET_LTE_SAFE_CHN_T, *P_CMD_GET_LTE_SAFE_CHN_T;

typedef struct _EVENT_LTE_MODE_T {
   
    UINT_8	ucVersion;
    UINT_8	aucReserved1[3];
    UINT_32	u4Flags; 
    LTE_SAFE_CH_INFO_T      rLteSafeChn;
} EVENT_LTE_MODE_T, *P_EVENT_LTE_MODE_T;
#endif

#if CFG_SUPPORT_SNIFFER
typedef struct _CMD_MONITOR_SET_INFO_T {
    UINT_8      ucEnable;
    UINT_8      ucBand;
    UINT_8      ucPriChannel;
    UINT_8      ucSco;
    UINT_8      ucChannelWidth;
    UINT_8      ucChannelS1;
    UINT_8      ucChannelS2;
    UINT_8      aucResv[9];
} CMD_MONITOR_SET_INFO_T, *P_CMD_MONITOR_SET_INFO_T;
#endif




VOID
nicCmdEventQueryMcrRead(IN P_ADAPTER_T prAdapter,
			IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID nicEventQueryMemDump(IN P_ADAPTER_T prAdapter, IN PUINT_8 pucEventBuf);


VOID
nicCmdEventQueryMemDump(IN P_ADAPTER_T prAdapter,
			IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQuerySwCtrlRead(IN P_ADAPTER_T prAdapter,
			   IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryChipConfig(IN P_ADAPTER_T prAdapter,
			   IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryRfTestATInfo(IN P_ADAPTER_T prAdapter,
			     IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventSetCommon(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventSetDisassociate(IN P_ADAPTER_T prAdapter,
			   IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventSetIpAddress(IN P_ADAPTER_T prAdapter,
			IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryLinkQuality(IN P_ADAPTER_T prAdapter,
			    IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryLinkSpeed(IN P_ADAPTER_T prAdapter,
			  IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryStatistics(IN P_ADAPTER_T prAdapter,
			   IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventEnterRfTest(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventLeaveRfTest(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryMcastAddr(IN P_ADAPTER_T prAdapter,
			  IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryEepromRead(IN P_ADAPTER_T prAdapter,
			   IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventSetMediaStreamMode(IN P_ADAPTER_T prAdapter,
			      IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventSetStopSchedScan(IN P_ADAPTER_T prAdapter,
			    IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryXmitOk(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryRecvOk(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryXmitError(IN P_ADAPTER_T prAdapter,
			  IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryRecvError(IN P_ADAPTER_T prAdapter,
			  IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryRecvNoBuffer(IN P_ADAPTER_T prAdapter,
			     IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryRecvCrcError(IN P_ADAPTER_T prAdapter,
			     IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryRecvErrorAlignment(IN P_ADAPTER_T prAdapter,
				   IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryXmitOneCollision(IN P_ADAPTER_T prAdapter,
				 IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryXmitMoreCollisions(IN P_ADAPTER_T prAdapter,
				   IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryXmitMaxCollisions(IN P_ADAPTER_T prAdapter,
				  IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID nicOidCmdTimeoutCommon(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo);

VOID nicCmdTimeoutCommon(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo);

VOID nicOidCmdEnterRFTestTimeout(IN P_ADAPTER_T prAdapter, IN P_CMD_INFO_T prCmdInfo);

#if CFG_SUPPORT_BUILD_DATE_CODE
VOID
nicCmdEventBuildDateCode(IN P_ADAPTER_T prAdapter,
			 IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);
#endif

VOID
nicCmdEventQueryStaStatistics(IN P_ADAPTER_T prAdapter,
			      IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

#if CFG_AUTO_CHANNEL_SEL_SUPPORT
VOID
nicCmdEventQueryChannelLoad(IN P_ADAPTER_T prAdapter,
			    IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);

VOID
nicCmdEventQueryLTESafeChn(IN P_ADAPTER_T prAdapter,
			   IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);
#endif


#if CFG_SUPPORT_BATCH_SCAN
VOID
nicCmdEventBatchScanResult(IN P_ADAPTER_T prAdapter,
			   IN P_CMD_INFO_T prCmdInfo, IN PUINT_8 pucEventBuf);
#endif


#endif 
