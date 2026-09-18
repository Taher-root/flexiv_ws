#!/usr/bin/env python3
import json
import struct

API_PORT_ROBOD = 19200
API_PORT_STATE = 19204
API_PORT_CTRL = 19205
API_PORT_TASK = 19206
API_PORT_CONFIG = 19207
API_PORT_KERNEL = 19208
API_PORT_OTHER = 19210

robot_status_loc_req = 1004
robot_status_speed_req = 1005
robot_control_motion_req = 2010

PACK_HEAD_FMT_STR = '!BBHLH6s'
PACK_RSV_DATA = b'\x00\x00\x00\x00\x00\x00'

def packMsg(reqId, msgTyp, msg={}):
    msgLen = 0
    jsonStr = json.dumps(msg)
    if msg != {}:
        msgLen = len(jsonStr)
    rawMsg = struct.pack(PACK_HEAD_FMT_STR, 0x5A, 1, reqId, msgLen, msgTyp, PACK_RSV_DATA)
    if msg != {}:
        rawMsg += bytearray(jsonStr, 'ascii')
    return rawMsg

def unpackHead(data):
    result = struct.unpack(PACK_HEAD_FMT_STR, data)
    jsonLen = result[3]
    reqNum = result[4]
    return (jsonLen, reqNum)

def create_velocity_command(vx, vy, w):
    return {'vx': float(vx), 'vy': float(vy), 'w': float(w)}

def create_stop_command():
    return create_velocity_command(0.0, 0.0, 0.0)
