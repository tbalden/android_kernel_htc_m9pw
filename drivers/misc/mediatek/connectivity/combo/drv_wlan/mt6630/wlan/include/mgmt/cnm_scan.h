




#ifndef _CNM_SCAN_H
#define _CNM_SCAN_H



#define SCN_CHANNEL_DWELL_TIME_MIN_MSEC         12
#define SCN_CHANNEL_DWELL_TIME_EXT_MSEC         98

#define SCN_TOTAL_PROBEREQ_NUM_FOR_FULL         3
#define SCN_SPECIFIC_PROBEREQ_NUM_FOR_FULL      1

#define SCN_TOTAL_PROBEREQ_NUM_FOR_PARTIAL      2
#define SCN_SPECIFIC_PROBEREQ_NUM_FOR_PARTIAL   1


#define SCN_INTERLACED_CHANNEL_GROUPS_NUM       3	

#define SCN_PARTIAL_SCAN_NUM                    3

#define SCN_PARTIAL_SCAN_IDLE_MSEC              100

#define	MAXIMUM_OPERATION_CHANNEL_LIST	        32

#define SCN_P2P_FULL_SCAN_PARAM                 0
typedef enum _ENUM_SCN_REQ_SOURCE_T {
	SCN_REQ_SOURCE_HEM = 0,
	SCN_REQ_SOURCE_NET_FSM,
	SCN_REQ_SOURCE_ROAMING,	
	SCN_REQ_SOURCE_OBSS,	
	SCN_REQ_SOURCE_NUM
} ENUM_SCN_REQ_SOURCE_T, *P_ENUM_SCN_REQ_SOURCE_T;

typedef enum _ENUM_SCAN_PROFILE_T {
	SCAN_PROFILE_FULL = 0,
	SCAN_PROFILE_PARTIAL,
	SCAN_PROFILE_VOIP,
	SCAN_PROFILE_FULL_2G4,
	SCAN_PROFILE_NUM
} ENUM_SCAN_PROFILE_T, *P_ENUM_SCAN_PROFILE_T;





#if 0
VOID cnmScanInit(VOID);

VOID cnmScanRunEventScanRequest(IN P_MSG_HDR_T prMsgHdr);

BOOLEAN cnmScanRunEventScanAbort(IN P_MSG_HDR_T prMsgHdr);

VOID cnmScanProfileSelection(VOID);

VOID cnmScanProcessStart(VOID);

VOID cnmScanProcessStop(VOID);

VOID cnmScanRunEventReqAISAbsDone(IN P_MSG_HDR_T prMsgHdr);

VOID cnmScanRunEventCancelAISAbsDone(IN P_MSG_HDR_T prMsgHdr);

VOID cnmScanPartialScanTimeout(UINT_32 u4Param);

VOID cnmScanRunEventScnFsmComplete(IN P_MSG_HDR_T prMsgHdr);
#endif



#endif				
