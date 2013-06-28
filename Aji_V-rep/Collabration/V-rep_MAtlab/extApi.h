// This file is part of the REMOTE API
// 
// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// The REMOTE API is licensed under the terms of GNU GPL:
// 
// -------------------------------------------------------------------
// The REMOTE API is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// The REMOTE API is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with the REMOTE API.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.3 on April 29th 2013

#ifndef _EXTAPI__
#define _EXTAPI__

#ifdef NON_MATLAB_PARSING /* when compiling, make sure NON_MATLAB_PARSING is always defined! */
	#include "extApiPlatform.h"
	#include "v_repConst.h"
#else 
	typedef char simxChar;
	typedef short simxShort;
	typedef unsigned short simxUShort;	
	typedef int simxInt;
	typedef unsigned int simxUInt;
	typedef float simxFloat;
	typedef void simxVoid;
	typedef double simxDouble;
#endif /* else NON_MATLAB_PARSING */

#ifdef _WIN32
	#define EXTAPI_DLLEXPORT extern __declspec(dllexport)
#endif
#if defined (__linux) || defined (__APPLE__)
	#define EXTAPI_DLLEXPORT extern
#endif

/* Use following to test endianness detection on little endian machines (endianness of the client is handled on the server side)
#define ENDIAN_TEST
*/

#ifdef NON_MATLAB_PARSING

/* Various other functions, not to be called directly */
simxVoid _waitUntilMessageArrived(simxInt clientID,simxInt* error);
simxChar* _setLastFetchedCmd(simxInt clientID,simxChar* cmdPtr,simxInt* error);

simxChar* _exec_null(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxChar options,simxInt* error);
simxChar* _exec_null_buffer(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxChar options,simxChar* buffer,simxInt bufferSize,simxInt* error);
simxChar* _exec_int(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxChar options,simxInt intValue,simxInt* error);
simxChar* _exec_intint(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxChar options,simxInt intValue1,simxInt intValue2,simxInt* error);
simxChar* _exec_string(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxChar options,const simxChar* stringValue,simxInt* error);
simxChar* _exec_int_int(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxChar options,simxInt intValue,simxInt intValue2,simxInt* error);
simxChar* _exec_intint_int(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxChar options,simxInt intValue1,simxInt intValue2,simxInt intValue3,simxInt* error);
simxChar* _exec_intint_buffer(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxChar options,simxInt intValue1,simxInt intValue2,simxChar* buffer,simxInt bufferSize,simxInt* error);
simxChar* _exec_int_float(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxChar options,simxInt intValue,simxFloat floatValue,simxInt* error);
simxChar* _exec_int_buffer(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxChar options,simxInt intValue,simxChar* buffer,simxInt bufferSize,simxInt* error);
simxChar* _exec_string_buffer(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxChar options,const simxChar* stringValue,simxChar* buffer,simxInt bufferSize,simxInt* error);

simxChar _readPureDataChar(simxChar* commandPointer,simxInt stringCnt,simxInt byteOffset);
simxInt _readPureDataInt(simxChar* commandPointer,simxInt stringCnt,simxInt byteOffset);
simxFloat _readPureDataFloat(simxChar* commandPointer,simxInt stringCnt,simxInt byteOffset);
simxInt _getCmdDataSize(simxChar* commandPointer);

simxChar* _getCommandPointer_(simxInt cmdRaw,const simxChar* commandBufferStart,simxInt commandBufferSize);
simxChar* _getCommandPointer_i(simxInt cmdRaw,simxInt intValue,const simxChar* commandBufferStart,simxInt commandBufferSize);
simxChar* _getCommandPointer_ii(simxInt cmdRaw,simxInt intValue1,simxInt intValue2,const simxChar* commandBufferStart,simxInt commandBufferSize);
simxChar* _getCommandPointer_s(simxInt cmdRaw,const simxChar* stringValue,const simxChar* commandBufferStart,simxInt commandBufferSize);

simxChar* _appendCommand_(simxInt cmd,simxChar options,simxUShort delayOrSplit,simxChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize);
simxChar* _appendCommand_null_buff(simxInt cmd,simxChar options,simxChar* buffer,simxInt bufferSize,simxUShort delayOrSplit,simxChar* destBuffer,simxInt* destBuffer_bufferSize,simxInt* destBuffer_dataSize);
simxChar* _appendCommand_i(simxInt cmd,simxChar options,simxInt intValue,simxUShort delayOrSplit,simxChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize);
simxChar* _appendCommand_ii(simxInt cmd,simxChar options,simxInt intValue1,simxInt intValue2,simxUShort delayOrSplit,simxChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize);
simxChar* _appendCommand_s(simxInt cmd,simxChar options,const simxChar* stringValue,simxUShort delayOrSplit,simxChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize);
simxChar* _appendCommand_i_i(simxInt cmd,simxChar options,simxInt intValue,simxInt intValue2,simxUShort delayOrSplit,simxChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize);
simxChar* _appendCommand_ii_i(simxInt cmd,simxChar options,simxInt intValue1,simxInt intValue2,simxInt intValue3,simxUShort delayOrSplit,simxChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize);
simxChar* _appendCommand_i_f(simxInt cmd,simxChar options,simxInt intValue,simxFloat floatValue,simxUShort delayOrSplit,simxChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize);
simxChar* _appendCommand_i_buff(simxInt cmd,simxChar options,simxInt intValue,simxChar* buffer,simxInt bufferSize,simxUShort delayOrSplit,simxChar* destBuffer,simxInt* destBuffer_bufferSize,simxInt* destBuffer_dataSize);
simxChar* _appendCommand_ii_buff(simxInt cmd,simxChar options,simxInt intValue1,simxInt intValue2,simxChar* buffer,simxInt bufferSize,simxUShort delayOrSplit,simxChar* destBuffer,simxInt* destBuffer_bufferSize,simxInt* destBuffer_dataSize);
simxChar* _appendCommand_s_buff(simxInt cmd,simxChar options,const simxChar* stringValue,simxChar* buffer,simxInt bufferSize,simxUShort delayOrSplit,simxChar* destBuffer,simxInt* destBuffer_bufferSize,simxInt* destBuffer_dataSize);

simxChar* _appendChunkToBuffer(const simxChar* chunk,simxInt chunkSize,simxChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize);
simxVoid _removeChunkFromBuffer(const simxChar* bufferStart,simxChar* chunkStart,simxInt chunkSize,simxInt* buffer_dataSize);

simxInt _removeCommandReply_null(simxInt clientID,simxInt cmdRaw);
simxInt _removeCommandReply_int(simxInt clientID,simxInt cmdRaw,simxInt intValue);
simxInt _removeCommandReply_intint(simxInt clientID,simxInt cmdRaw,simxInt intValue1,simxInt intValue2);
simxInt _removeCommandReply_string(simxInt clientID,simxInt cmdRaw,const char* stringValue);

simxChar* _getSameCommandPointer(const simxChar* cmdPtr,simxChar* cmdBuffer,simxInt cmdBufferSize);
SIMX_THREAD_RET_TYPE _communicationThread(simxVoid* p);

simxUShort _getCRC(const simxChar* data,simxInt length);

simxChar _sendMessage_socket(simxInt clientID,const simxChar* message,simxInt messageSize);
simxChar* _receiveReplyMessage_socket(simxInt clientID,simxInt* messageSize);
simxChar _sendSimplePacket_socket(simxInt clientID,const simxChar* packet,simxShort packetLength,simxShort packetsLeft);
simxInt _receiveSimplePacket_socket(simxInt clientID,simxChar* packet,simxShort* packetSize);

#endif /* NON_MATLAB_PARSING */


/* The remote API functions */
EXTAPI_DLLEXPORT simxInt simxGetJointPosition(simxInt clientID,simxInt jointHandle,simxFloat* position,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetJointPosition(simxInt clientID,simxInt jointHandle,simxFloat position,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetJointMatrix(simxInt clientID,simxInt jointHandle,simxFloat* matrix,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetSphericalJointMatrix(simxInt clientID,simxInt jointHandle,simxFloat* matrix,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetJointTargetVelocity(simxInt clientID,simxInt jointHandle,simxFloat targetVelocity,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetJointTargetPosition(simxInt clientID,simxInt jointHandle,simxFloat targetPosition,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxJointGetForce(simxInt clientID,simxInt jointHandle,simxFloat* force,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetJointForce(simxInt clientID,simxInt jointHandle,simxFloat force,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxReadForceSensor(simxInt clientID,simxInt forceSensorHandle,simxChar* state,simxFloat* forceVector,simxFloat* torqueVector,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxBreakForceSensor(simxInt clientID,simxInt forceSensorHandle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxReadVisionSensor(simxInt clientID,simxInt sensorHandle,simxChar* detectionState,simxFloat** auxValues,simxInt** auxValuesCount,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetObjectHandle(simxInt clientID,const simxChar* objectName,simxInt* handle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetVisionSensorImage(simxInt clientID,simxInt sensorHandle,simxInt* resolution,simxChar** image,simxChar options,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetVisionSensorImage(simxInt clientID,simxInt sensorHandle,simxChar* image,simxInt bufferSize,simxChar options,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetVisionSensorDepthBuffer(simxInt clientID,simxInt sensorHandle,simxInt* resolution,simxFloat** buffer,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetObjectChild(simxInt clientID,simxInt parentObjectHandle,simxInt childIndex,simxInt* childObjectHandle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetObjectParent(simxInt clientID,simxInt childObjectHandle,simxInt* parentObjectHandle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxReadProximitySensor(simxInt clientID,simxInt sensorHandle,simxChar* detectionState,simxFloat* detectedPoint,simxInt* detectedObjectHandle,simxFloat* detectedSurfaceNormalVector,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxLoadModel(simxInt clientID,const simxChar* modelPathAndName,simxChar options,simxInt* baseHandle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxLoadUI(simxInt clientID,const simxChar* uiPathAndName,simxChar options,simxInt* count,simxInt** uiHandles,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxLoadScene(simxInt clientID,const simxChar* scenePathAndName,simxChar options,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxStartSimulation(simxInt clientID,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxPauseSimulation(simxInt clientID,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxStopSimulation(simxInt clientID,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetUIHandle(simxInt clientID,const simxChar* uiName,simxInt* handle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetUISlider(simxInt clientID,simxInt uiHandle,simxInt uiButtonID,simxInt* position,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetUISlider(simxInt clientID,simxInt uiHandle,simxInt uiButtonID,simxInt position,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetUIEventButton(simxInt clientID,simxInt uiHandle,simxInt* uiEventButtonID,simxInt* auxValues,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetUIButtonProperty(simxInt clientID,simxInt uiHandle,simxInt uiButtonID,simxInt* prop,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetUIButtonProperty(simxInt clientID,simxInt uiHandle,simxInt uiButtonID,simxInt prop,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxAddStatusbarMessage(simxInt clientID,const simxChar* message,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxAuxiliaryConsoleOpen(simxInt clientID,const simxChar* title,simxInt maxLines,simxInt mode,simxInt* position,simxInt* size,simxFloat* textColor,simxFloat* backgroundColor,simxInt* consoleHandle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxAuxiliaryConsoleClose(simxInt clientID,simxInt consoleHandle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxAuxiliaryConsolePrint(simxInt clientID,simxInt consoleHandle,const simxChar* txt,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxAuxiliaryConsoleShow(simxInt clientID,simxInt consoleHandle,simxChar showState,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetObjectOrientation(simxInt clientID,simxInt objectHandle,simxInt relativeToObjectHandle,simxFloat* eulerAngles,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetObjectPosition(simxInt clientID,simxInt objectHandle,simxInt relativeToObjectHandle,simxFloat* position,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetObjectOrientation(simxInt clientID,simxInt objectHandle,simxInt relativeToObjectHandle,const simxFloat* eulerAngles,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetObjectPosition(simxInt clientID,simxInt objectHandle,simxInt relativeToObjectHandle,const simxFloat* position,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetObjectParent(simxInt clientID,simxInt objectHandle,simxInt parentObject,simxChar keepInPlace,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetUIButtonLabel(simxInt clientID,simxInt uiHandle,simxInt uiButtonID,const simxChar* upStateLabel,const simxChar* downStateLabel,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetLastErrors(simxInt clientID,simxInt* errorCnt,simxChar** errorStrings,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetArrayParameter(simxInt clientID,simxInt paramIdentifier,simxFloat* paramValues,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetArrayParameter(simxInt clientID,simxInt paramIdentifier,const simxFloat* paramValues,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetBooleanParameter(simxInt clientID,simxInt paramIdentifier,simxChar* paramValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetBooleanParameter(simxInt clientID,simxInt paramIdentifier,simxChar paramValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetIntegerParameter(simxInt clientID,simxInt paramIdentifier,simxInt* paramValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetIntegerParameter(simxInt clientID,simxInt paramIdentifier,simxInt paramValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetFloatingParameter(simxInt clientID,simxInt paramIdentifier,simxFloat* paramValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetFloatingParameter(simxInt clientID,simxInt paramIdentifier,simxFloat paramValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetStringParameter(simxInt clientID,simxInt paramIdentifier,simxChar** paramValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetCollisionHandle(simxInt clientID,const simxChar* collisionObjectName,simxInt* handle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetDistanceHandle(simxInt clientID,const simxChar* distanceObjectName,simxInt* handle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxReadCollision(simxInt clientID,simxInt collisionObjectHandle,simxChar* collisionState,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxReadDistance(simxInt clientID,simxInt distanceObjectHandle,simxFloat* minimumDistance,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxRemoveObject(simxInt clientID,simxInt objectHandle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxRemoveUI(simxInt clientID,simxInt uiHandle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxCloseScene(simxInt clientID,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetObjects(simxInt clientID,simxInt objectType,simxInt* objectCount,simxInt** objectHandles,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxDisplayDialog(simxInt clientID,const simxChar* titleText,const simxChar* mainText,simxInt dialogType,const simxChar* initialText,simxFloat* titleColors,simxFloat* dialogColors,simxInt* dialogHandle,simxInt* uiHandle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxEndDialog(simxInt clientID,simxInt dialogHandle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetDialogInput(simxInt clientID,simxInt dialogHandle,simxChar** inputText,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetDialogResult(simxInt clientID,simxInt dialogHandle,simxInt* result,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxCopyPasteObjects(simxInt clientID,const simxInt* objectHandles,simxInt objectCount,simxInt** newObjectHandles,simxInt* newObjectCount,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetObjectSelection(simxInt clientID,simxInt** objectHandles,simxInt* objectCount,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetObjectSelection(simxInt clientID,const simxInt* objectHandles,simxInt objectCount,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxClearFloatSignal(simxInt clientID,const simxChar* signalName,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxClearIntegerSignal(simxInt clientID,const simxChar* signalName,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxClearStringSignal(simxInt clientID,const simxChar* signalName,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetFloatSignal(simxInt clientID,const simxChar* signalName,simxFloat* signalValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetIntegerSignal(simxInt clientID,const simxChar* signalName,simxInt* signalValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetStringSignal(simxInt clientID,const simxChar* signalName,simxChar** signalValue,simxInt* signalLength,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetAndClearStringSignal(simxInt clientID,const simxChar* signalName,simxChar** signalValue,simxInt* signalLength,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetFloatSignal(simxInt clientID,const simxChar* signalName,simxFloat signalValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetIntegerSignal(simxInt clientID,const simxChar* signalName,simxInt signalValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetStringSignal(simxInt clientID,const simxChar* signalName,const simxChar* signalValue,simxInt signalLength,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxAppendStringSignal(simxInt clientID,const simxChar* signalName,const simxChar* signalValue,simxInt signalLength,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetObjectFloatParameter(simxInt clientID,simxInt objectHandle,simxInt parameterID,simxFloat* parameterValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetObjectFloatParameter(simxInt clientID,simxInt objectHandle,simxInt parameterID,simxFloat parameterValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetObjectIntParameter(simxInt clientID,simxInt objectHandle,simxInt parameterID,simxInt* parameterValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetObjectIntParameter(simxInt clientID,simxInt objectHandle,simxInt parameterID,simxInt parameterValue,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxGetModelProperty(simxInt clientID,simxInt objectHandle,simxInt* prop,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxSetModelProperty(simxInt clientID,simxInt objectHandle,simxInt prop,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxRMLPosition(simxInt clientID,simxInt dofs,simxDouble timeStep,simxInt flags,const simxDouble* currentPosVelAccel,const simxDouble* maxVelAccelJerk,const simxChar* selection,const simxDouble* targetPosVel,simxInt* rmlState,simxDouble* newPosVelAccel,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxRMLVelocity(simxInt clientID,simxInt dofs,simxDouble timeStep,simxInt flags,const simxDouble* currentPosVelAccel,const simxDouble* maxAccelJerk,const simxChar* selection,const simxDouble* targetVel,simxInt* rmlState,simxDouble* newPosVelAccel,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxCreateDummy(simxInt clientID,simxFloat size,const simxChar* colors,simxInt* objectHandle,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxQuery(simxInt clientID,const simxChar* signalName,const simxChar* signalValue,simxInt signalLength,const simxChar* retSignalName,simxChar** retSignalValue,simxInt* retSignalLength,simxInt timeOutInMs);



/* Very first function to call */
EXTAPI_DLLEXPORT simxInt simxStart(const simxChar* connectionAddress,simxInt connectionPort,simxChar waitUntilConnected,simxChar doNotReconnectOnceDisconnected,simxInt timeOutInMs,simxInt commThreadCycleInMs);

/* Very last function to call */
EXTAPI_DLLEXPORT simxVoid simxFinish(simxInt clientID);

/* The remote API helper functions */
EXTAPI_DLLEXPORT simxInt simxGetPingTime(simxInt clientID,simxInt* pingTime);
EXTAPI_DLLEXPORT simxInt simxGetLastCmdTime(simxInt clientID);
EXTAPI_DLLEXPORT simxInt simxSynchronousTrigger(simxInt clientID);
EXTAPI_DLLEXPORT simxInt simxSynchronous(simxInt clientID,simxChar enable);
EXTAPI_DLLEXPORT simxInt simxPauseCommunication(simxInt clientID,simxChar pause);
EXTAPI_DLLEXPORT simxInt simxGetInMessageInfo(simxInt clientID,simxInt infoType,simxInt* info);
EXTAPI_DLLEXPORT simxInt simxGetOutMessageInfo(simxInt clientID,simxInt infoType,simxInt* info);
EXTAPI_DLLEXPORT simxInt simxGetConnectionId(simxInt clientID);
EXTAPI_DLLEXPORT simxChar* simxCreateBuffer(simxInt bufferSize);
EXTAPI_DLLEXPORT simxVoid simxReleaseBuffer(simxChar* buffer);
EXTAPI_DLLEXPORT simxInt simxTransferFile(simxInt clientID,const simxChar* filePathAndName,const simxChar* fileName_serverSide,simxInt timeOut,simxInt operationMode);
EXTAPI_DLLEXPORT simxInt simxEraseFile(simxInt clientID,const simxChar* fileName_serverSide,simxInt operationMode);

#endif /* _EXTAPI__ */		
