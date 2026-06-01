#ifndef __DSPOTTER_SDK_API_CONST_H__
#define __DSPOTTER_SDK_API_CONST_H__


#define DSPOTTER_SUCCESS						(     0 )
#define DSPOTTER_ERR_SDKError					( -2000 )
#define DSPOTTER_ERR_LexiconError				( -3000 )
#define DSPOTTER_ERR_EngineError				( -5000 )


/************************************************************************/
// Recognition type                                                                
/************************************************************************/
#define DSPOTTER_RecogType_Unknown				(0)
#define DSPOTTER_RecogType_Passed				(1)
#define DSPOTTER_RecogType_NotGoodEnough		(2)
#define DSPOTTER_RecogType_MissStartSyllalbe	(3)
#define DSPOTTER_RecogType_MissEndSyllalbe		(4)

/************************************************************************/
// SD
/************************************************************************/

/************************************************************************/
// Error code                                                                  
/************************************************************************/

#define DSPOTTER_ERR_IllegalHandle				( DSPOTTER_ERR_SDKError -   1 )
#define DSPOTTER_ERR_IllegalParam				( DSPOTTER_ERR_SDKError -   2 )
#define DSPOTTER_ERR_LeaveNoMemory				( DSPOTTER_ERR_SDKError -   3 )
#define DSPOTTER_ERR_LoadDLLFailed				( DSPOTTER_ERR_SDKError -   4 )
#define DSPOTTER_ERR_LoadModelFailed			( DSPOTTER_ERR_SDKError -   5 )
#define DSPOTTER_ERR_GetFunctionFailed			( DSPOTTER_ERR_SDKError -   6 )
#define DSPOTTER_ERR_ParseEINFailed				( DSPOTTER_ERR_SDKError -   7 )
#define DSPOTTER_ERR_OpenFileFailed				( DSPOTTER_ERR_SDKError -   8 )
#define DSPOTTER_ERR_NeedMoreSample				( DSPOTTER_ERR_SDKError -   9 )
#define DSPOTTER_ERR_Timeout					( DSPOTTER_ERR_SDKError -  10 )
#define DSPOTTER_ERR_InitWTFFailed				( DSPOTTER_ERR_SDKError -  11 )
#define DSPOTTER_ERR_AddSampleFailed			( DSPOTTER_ERR_SDKError -  12 )
#define DSPOTTER_ERR_BuildUserCommandFailed	    ( DSPOTTER_ERR_SDKError -  13 )
#define DSPOTTER_ERR_MergeUserCommandFailed	    ( DSPOTTER_ERR_SDKError -  14 )
#define DSPOTTER_ERR_IllegalUserCommandFile     ( DSPOTTER_ERR_SDKError -  15 )
#define DSPOTTER_ERR_IllegalWaveFile			( DSPOTTER_ERR_SDKError -  16 )
#define DSPOTTER_ERR_BuildCommandFailed			( DSPOTTER_ERR_SDKError -  17 )
#define DSPOTTER_ERR_InitFixNRFailed			( DSPOTTER_ERR_SDKError -  18 )
#define DSPOTTER_ERR_EXCEED_NR_BUFFER_SIZE		( DSPOTTER_ERR_SDKError -  19 )
#define DSPOTTER_ERR_Rejected				    ( DSPOTTER_ERR_SDKError -  20 )
#define DSPOTTER_ERR_NoVoiceDetect		        ( DSPOTTER_ERR_SDKError -  21 )
#define DSPOTTER_ERR_Expired					( DSPOTTER_ERR_SDKError - 100 )
#define DSPOTTER_ERR_LicenseFailed				( DSPOTTER_ERR_SDKError - 200 )

#define DSPOTTER_ERR_CreateModelFailed			( DSPOTTER_ERR_SDKError - 500 )
#define DSPOTTER_ERR_WriteFailed				( DSPOTTER_ERR_SDKError - 501 )
#define DSPOTTER_ERR_NotEnoughStorage			( DSPOTTER_ERR_SDKError - 502 )
#define DSPOTTER_ERR_NoisyEnvironment			( DSPOTTER_ERR_SDKError - 503 )
#define DSPOTTER_ERR_VoiceTooShort		        ( DSPOTTER_ERR_SDKError - 504 )
#define DSPOTTER_ERR_VoiceTooLong		        ( DSPOTTER_ERR_SDKError - 505 )

#define DSPOTTER_ERR_AGCError					( DSPOTTER_ERR_SDKError - 605 )

#endif //__DSPOTTER_SDK_API_CONST_H__

