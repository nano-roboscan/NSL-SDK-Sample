#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import platform
import ctypes
from ctypes import (
    c_int, c_double, c_char_p, c_ubyte, c_bool, Structure, POINTER
)
import numpy as np


# Windows -> "nanolib.dll",  Linux -> "libnanolib.so", MAC -> "libnanolib.dylib"
DLL_PATHS = {
        "AARCH": "../nsl_lib/lib/aarch-7.5/shared/libnanolib.so", 
        "Linux": "../nsl_lib/lib/linux-7.5/shared/libnanolib.so", 
        "Windows": "../nsl_lib/lib/windows/shared/nanolib.dll", 
        "Darwin": "../nsl_lib/lib/mac-12.0.5/shared/libnanolib.dylib"
}

# 현재 운영 체제 확인
current_os = platform.system()
# ARM64 아키텍처일 경우 AARCH 경로 사용
if current_os == "Linux" and platform.machine().startswith("aarch"):
    current_os = "AARCH"

print("current_os = %s" % current_os)
    
dll_path = DLL_PATHS.get(current_os)
_nsl = None

print("current DLL Name : %s" % dll_path)

try:
    if dll_path:
        if os.path.exists(dll_path):
            _nsl = ctypes.CDLL(dll_path)
        else:
            sys.exit(":nanolib 동적라이브러리를 찾을 수 없습니다. dll/so 경로를 수정하세요.")
    else:
        print("Unsupported OS: %s" % current_os)
except Exception:
    sys.exit("::nanolib 동적라이브러리를 찾을 수 없습니다. dll/so 경로를 수정하세요.")
    
# Lidar Max Resolution
NSL_LIDAR_WIDTH  = 160
NSL_LIDAR_HEIGHT = 60

MAX_GRAYSCALE_VALUE		= 2897

#max ditance :: mm
MAX_DISTANCE_20MHZ		= 7500
MAX_DISTANCE_10MHZ		= 15000

# pixel data range
NSL_LIMIT_FOR_VALID_DATA = 16000
NSL_LOW_AMPLITUDE		 = 16001
NSL_ADC_OVERFLOW		 = 16002
NSL_SATURATION			 = 16003
NSL_BAD_PIXEL 			 = 16004
NSL_LOW_DCS				 = 16005
NSL_INTERFERENCE		 = 16007
NSL_EDGE_DETECTED		 = 16008

# Point Cloud Type
OUT_X                   = 0
OUT_Y                   = 1
OUT_Z                   = 2
MAX_OUT                 = 3

# FUNCTION_OPTIONS
FUNC_OFF    = 0
FUNC_ON     = 1

# HDR_OPTIONS
HDR_NONE_MODE       = 0
HDR_SPATIAL_MODE    = 1
HDR_TEMPORAL_MODE   = 2

# MODULATION_OPTIONS
MOD_10Mhz   = 0
MOD_20Mhz   = 1

# MODULATION_CH_OPTIONS
MOD_CH0 = 0
MOD_CH1 = 1
MOD_CH2 = 2
MOD_CH3 = 3
MOD_CH4 = 4
MOD_CH5 = 5
MOD_CH6 = 6
MOD_CH7 = 7
MOD_CH8 = 8
MOD_CH9 = 9
MOD_CH10 = 10
MOD_CH11 = 11
MOD_CH12 = 12
MOD_CH13 = 13
MOD_CH14 = 14
MOD_CH15 = 15

# FRAME_RATE_OPTIONS
FRAME_5FPS = 5
FRAME_10FPS = 10
FRAME_15FPS = 15
FRAME_20FPS = 20
FRAME_25FPS = 25
FRAME_30FPS = 30

# OPERATION_MODE_OPTIONS
NONE_MODE                   = 0
DISTANCE_MODE               = 1
DISTANCE_AMPLITUDE_MODE     = 2

# NSL_ERROR_TYPE  
NSL_SUCCESS = 0
NSL_INVALID_HANDLE = -1
NSL_NOT_OPENED = -2
NSL_NOT_READY = -3
NSL_IP_DUPLICATED = -4
NSL_HANDLE_OVERFLOW = -5
NSL_DISCONNECTED_SOCKET = -6
NSL_ANSWER_ERROR = -7
NSL_INVALID_PARAMETER = -8


# NslConfig structure in nanolib.h
class NslConfig(ctypes.Structure):
    _fields_ = [
        ("integrationTime3D", ctypes.c_int*4),
        ("integrationTimeGrayScale", ctypes.c_int),
        ("roiXMin", ctypes.c_int),
        ("roiXMax", ctypes.c_int),
        ("roiYMin", ctypes.c_int),
        ("roiYMax", ctypes.c_int),
        ("currentOffset", ctypes.c_int*2),
        ("minAmplitude", ctypes.c_int*4),
        ("lidarAngleV", ctypes.c_double),
        ("lidarAngleH", ctypes.c_double),
        ("operationModeOpt", ctypes.c_int),
        ("hdrOpt", ctypes.c_int),
        ("mod_frequencyOpt", ctypes.c_int),
        ("mod_channelOpt", ctypes.c_int),
        ("medianOpt", ctypes.c_int),
        ("gaussOpt", ctypes.c_int),
        ("temporalFactorValue", ctypes.c_int),
        ("temporalThresholdValue", ctypes.c_int),
        ("edgeThresholdValue", ctypes.c_int),
        ("interferenceDetectionLimitValue", ctypes.c_int),
        ("interferenceDetectionLastValueOpt", ctypes.c_int),
        ("edgeThresholdValue3D", ctypes.c_int),
        ("frameRateOpt", ctypes.c_int),
    ]

# NslVec3b structure in nanolib.h
class NslVec3b(ctypes.Structure):
    _fields_ = [
        ("b", c_ubyte),
        ("g", c_ubyte),
        ("r", c_ubyte),
    ]
    
    @property
    def __array_interface__(self):
        return {
            'descr': [('', np.uint8)],
            'shape': (3,),
            'typestr': np.dtype(np.uint8).str,
            'data': (ctypes.addressof(self), False),
        }
   
# NslPCD structure in nanolib.h
class NslPCD(Structure):
    _fields_ = [
        ("operationMode", c_int),
        ("temperature", c_double),
        ("width", c_int),
        ("height", c_int),
        ("roiXMin", c_int),
        ("roiYMin", c_int),
        ("binning_h", c_int),
        ("binning_v", c_int),
        ("amplitude", c_int * (NSL_LIDAR_HEIGHT  * NSL_LIDAR_WIDTH)),
        ("distance2D", c_int * (NSL_LIDAR_HEIGHT  * NSL_LIDAR_WIDTH)),
        ("distance3D", c_double * (MAX_OUT * NSL_LIDAR_HEIGHT  * NSL_LIDAR_WIDTH)),
    ]

    def __init__(self, *args, **kwargs):
        super(NslPCD, self).__init__(*args, **kwargs)

        self.amplitude_np = np.ctypeslib.as_array(self.amplitude).reshape(
            (NSL_LIDAR_HEIGHT, NSL_LIDAR_WIDTH)
        )
        self.distance2D_np = np.ctypeslib.as_array(self.distance2D).reshape(
            (NSL_LIDAR_HEIGHT, NSL_LIDAR_WIDTH)
        )
        self.distance3D_np = np.ctypeslib.as_array(self.distance3D).reshape(
            (MAX_OUT, NSL_LIDAR_HEIGHT, NSL_LIDAR_WIDTH)
        )
        
    # ---- numpy 변환 도우미 ----
    def np_distance2D(self):
        return self.distance2D_np
        
    def np_amplitude(self):
        return self.amplitude_np

    def np_distance3D(self):
        return self.distance3D_np

# Python Wrapper
class NanoLidar:
    def __init__(self, devName="/dev/ttyNsl2206", lidar_angleV = 0.0, lidar_angleH = 0.0, debug=FUNC_ON):
                
        _nsl.nsl_open.argtypes  = [c_char_p, ctypes.POINTER(NslConfig), c_int]
        _nsl.nsl_open.restype   = ctypes.c_int   # handle 반환

        _nsl.nsl_close.argtypes = []
        _nsl.nsl_close.restype  = c_int

        _nsl.nsl_streamingOn.argtypes  = [c_int, c_int]
        _nsl.nsl_streamingOn.restype   = c_int

        _nsl.nsl_streamingOff.argtypes = [c_int]
        _nsl.nsl_streamingOff.restype  = c_int

        _nsl.nsl_getPointCloudData.argtypes = [c_int, ctypes.POINTER(NslPCD), c_int]
        _nsl.nsl_getPointCloudData.restype  = c_int
        
        _nsl.nsl_setFrameRate.argtypes = [c_int, c_int]
        _nsl.nsl_setFrameRate.restype  = c_int
    
        _nsl.nsl_setMinAmplitude.argtypes = [c_int, c_int, c_int, c_int, c_int]
        _nsl.nsl_setMinAmplitude.restype  = c_int

        _nsl.nsl_setIntegrationTime.argtypes = [c_int, c_int, c_int, c_int, c_int, c_int]
        _nsl.nsl_setIntegrationTime.restype  = c_int

        _nsl.nsl_setHdrMode.argtypes = [c_int, c_int]
        _nsl.nsl_setHdrMode.restype  = c_int
        
        _nsl.nsl_setModulation.argtypes = [c_int, c_int, c_int]
        _nsl.nsl_setModulation.restype  = c_int
        
        _nsl.nsl_setFilter.argtypes = [c_int, c_int, c_int, c_int, c_int, c_int, c_int, c_int]
        _nsl.nsl_setFilter.restype  = c_int

        _nsl.nsl_set3DFilter.argtypes = [c_int, c_int]
        _nsl.nsl_set3DFilter.restype  = c_int

        _nsl.nsl_setRoi.argtypes = [c_int, c_int, c_int, c_int, c_int]
        _nsl.nsl_setRoi.restype  = c_int

        _nsl.nsl_saveConfiguration.argtypes = [c_int]
        _nsl.nsl_saveConfiguration.restype  = c_int

        _nsl.nsl_setColorRange.argtypes = [c_int, c_int, c_int]
        _nsl.nsl_setColorRange.restype  = c_int
        
        _nsl.nsl_getDistanceColor.argtypes = [c_int]
        _nsl.nsl_getDistanceColor.restype  = NslVec3b

        _nsl.nsl_getAmplitudeColor.argtypes = [c_int]
        _nsl.nsl_getAmplitudeColor.restype  = NslVec3b

        self.cfg = NslConfig()
        self.cfg.lidarAngleV = lidar_angleV #필수 인자
        self.cfg.lidarAngleH = lidar_angleH #필수 인자
        self.handle = _nsl.nsl_open(devName.encode("utf-8"), ctypes.byref(self.cfg), debug)
        if self.handle < 0:
            raise RuntimeError("nsl_open 실패")

        _nsl.nsl_setColorRange(MAX_DISTANCE_10MHZ, MAX_GRAYSCALE_VALUE, FUNC_ON)

        self.dist_color_lut = np.array([
            [_nsl.nsl_getDistanceColor(z).r / 255.0,
             _nsl.nsl_getDistanceColor(z).g / 255.0,
             _nsl.nsl_getDistanceColor(z).b / 255.0]
            for z in range(NSL_EDGE_DETECTED+1)
        ], dtype=np.float32)

        self.ampl_color_lut = np.array([
            [_nsl.nsl_getAmplitudeColor(z).r / 255.0,
             _nsl.nsl_getAmplitudeColor(z).g / 255.0,
             _nsl.nsl_getAmplitudeColor(z).b / 255.0]
            for z in range(NSL_EDGE_DETECTED+1)
        ], dtype=np.float32)
        
        print("[NanoLidar] Opened. handle=%d" % self.handle)
    
    # ---------------------- toString 계열 ----------------------
    def toString_FUNCTION_OPTIONS(self, c):
        mapping = {
            FUNC_OFF: "FUNC_OFF",
            FUNC_ON: "FUNC_ON",
        }
        return mapping.get(c, "Unknown->FUNC_ON")

    def toString_HDR_OPTIONS(self, c):
        mapping = {
            HDR_NONE_MODE: "HDR_NONE_MODE",
            HDR_SPATIAL_MODE: "HDR_SPATIAL_MODE",
            HDR_TEMPORAL_MODE: "HDR_TEMPORAL_MODE",
        }
        return mapping.get(c, "Unknown->HDR_NONE_MODE")

    def toString_MODULATION_OPTIONS(self, c):
        mapping = {
            MOD_10Mhz: "MOD_12Mhz",
            MOD_20Mhz: "MOD_24Mhz",
        }
        return mapping.get(c, "Unknown->MOD_12Mhz")

    def toString_MODULATION_CH_OPTIONS(self, c):
        mapping = dict((i, "MOD_CH{0}".format(i)) for i in range(16))
        return mapping.get(c, "Unknown->MOD_CH0")

    def toString_FRAME_RATE_OPTIONS(self, c):
        mapping = {
            FRAME_5FPS: "FRAME_5FPS",
            FRAME_10FPS: "FRAME_10FPS",
            FRAME_15FPS: "FRAME_15FPS",
            FRAME_20FPS: "FRAME_20FPS",
            FRAME_25FPS: "FRAME_25FPS",
            FRAME_30FPS: "FRAME_30FPS",
        }
        return mapping.get(c, "Unknown->FRAME_15FPS")

    def toString_OPERATION_MODE_OPTIONS(self, c):
        mapping = {
            NONE_MODE: "NONE_MODE",
            DISTANCE_MODE: "DISTANCE_MODE",
            DISTANCE_AMPLITUDE_MODE: "DISTANCE_AMPLITUDE_MODE",
        }
        return mapping.get(c, "Unknown->DISTANCE_AMPLITUDE_MODE")

    def toString_NSL_ERROR_TYPE(self, c):
        mapping = {
            NSL_SUCCESS: "NSL_SUCCESS",
            NSL_INVALID_HANDLE: "NSL_INVALID_HANDLE",
            NSL_NOT_OPENED: "NSL_NOT_OPENED",
            NSL_NOT_READY: "NSL_NOT_READY",
            NSL_IP_DUPLICATED: "NSL_IP_DUPLICATED",
            NSL_HANDLE_OVERFLOW: "NSL_HANDLE_OVERFLOW",
            NSL_DISCONNECTED_SOCKET: "NSL_DISCONNECTED_SOCKET",
            NSL_ANSWER_ERROR: "NSL_ANSWER_ERROR",
            NSL_INVALID_PARAMETER: "NSL_INVALID_PARAMETER",
        }
        return mapping.get(c, "Unknown")
        
    def printConfiguration(self):
        cfg = self.cfg
        print("------------------------------------------------------------------------")
        print("------------------------- Device configuration -------------------------")
        print("------------------------------------------------------------------------")

        print("HDR = {0}".format(self.toString_HDR_OPTIONS(cfg.hdrOpt)))

        print("int time = {0}.{1}.{2}.{3}.{4}".format(
            cfg.integrationTime3D[0],
            cfg.integrationTime3D[1],
            cfg.integrationTime3D[2],
            cfg.integrationTime3D[3],
            cfg.integrationTimeGrayScale))

        print("roi = {0},{1},{2},{3}".format(
            cfg.roiXMin, cfg.roiYMin, cfg.roiXMax, cfg.roiYMax))

        print("Modulation = {0}, ch = {1}".format(
            self.toString_MODULATION_OPTIONS(cfg.mod_frequencyOpt),
            self.toString_MODULATION_CH_OPTIONS(cfg.mod_channelOpt)))

        print("filter median = {0}, gauss = {1}, temporal factor = {2}, "
              "temporal threshold = {3}, edge threshold = {4}, "
              "interferenceLimit = {5}, used interference Last value = {6}".format(
                  self.toString_FUNCTION_OPTIONS(cfg.medianOpt),
                  self.toString_FUNCTION_OPTIONS(cfg.gaussOpt),
                  cfg.temporalFactorValue,
                  cfg.temporalThresholdValue,
                  cfg.edgeThresholdValue,
                  cfg.interferenceDetectionLimitValue,
                  self.toString_FUNCTION_OPTIONS(cfg.interferenceDetectionLastValueOpt)))

        print("frame rate = {0}".format(self.toString_FRAME_RATE_OPTIONS(cfg.frameRateOpt)))
        print("------------------------------------------------------------------------")

    def get_nsl_config(self):
        return self.cfg
            
    def start_stream(self, mode=DISTANCE_AMPLITUDE_MODE):
        #if( mode 
        ret = _nsl.nsl_streamingOn(self.handle, mode)
        if ret != NSL_SUCCESS:
            raise RuntimeError("nsl_streamingOn 실패 (mode={0}) ret = {1}".format(self.toString_OPERATION_MODE_OPTIONS(mode), self.toString_NSL_ERROR_TYPE(ret)))
        print("[NanoLidar] Streaming ON (mode={0}) ret = {1}".format(self.toString_OPERATION_MODE_OPTIONS(mode), self.toString_NSL_ERROR_TYPE(ret)))

    def stop_stream(self):
        ret = _nsl.nsl_streamingOff(self.handle)
        print("[NanoLidar] Streaming OFF")
        return ret

    def get_frame(self, frame, timeout_ms=1000):
        return _nsl.nsl_getPointCloudData(self.handle, ctypes.byref(frame), timeout_ms)
        
    def set_frame_rate(self, FRAME_RATE_OPTIONS):
        return _nsl.nsl_setFrameRate(self.handle, FRAME_RATE_OPTIONS) 

    def set_minimum_amplitude(self, amplitude0, amplitude1, amplitude2, amplitude3):
        return _nsl.nsl_setMinAmplitude(self.handle, amplitude0, amplitude1, amplitude2, amplitude3) 

    def set_intetration_time(self, intTime, intTimeHdr1, intTimeHdr2, intTimeHdr3, intTimeGray):
        return _nsl.nsl_setIntegrationTime(self.handle, intTime, intTimeHdr1, intTimeHdr2, intTimeHdr3, intTimeGray) 

    def set_hdr_mode(self, HDR_OPTIONS):
        return _nsl.nsl_setHdrMode(self.handle, HDR_OPTIONS) 

    def set_modulation(self, MODULATION_OPTIONS, MODULATION_CH_OPTIONS):
        return _nsl.nsl_setModulation(self.handle, MODULATION_OPTIONS, MODULATION_CH_OPTIONS) 

    def set_filters(self, FUNCTION_OPTIONS_Median, FUNCTION_OPTIONS_Gauss, temporalFactor, temporalThreshold, edgeThreshold, interferenceDetectionLimit, FUNCTION_OPTIONS_lastValue):
        return _nsl.nsl_setFilter(self.handle, FUNCTION_OPTIONS_Median, FUNCTION_OPTIONS_Gauss, temporalFactor, temporalThreshold, edgeThreshold, interferenceDetectionLimit, FUNCTION_OPTIONS_lastValue)

    def set_3d_filter(self, edgethreshold):
        return _nsl.nsl_set3DFilter(self.handle, edgethreshold) 

    def set_roi(self, minX, minY, maxX, maxY):
        return _nsl.nsl_setRoi(self.handle, minX, minY, maxX, maxY)
        
    def save_configuration(self):
        return _nsl.nsl_saveConfiguration(self.handle)

    def set_color_range(self, maxDistance, maxGrayscale, FUNCTION_OPTIONS_grayscale):
        ret = _nsl.nsl_setColorRange(maxDistance, maxGrayscale, FUNCTION_OPTIONS_grayscale)
        self.ampl_color_lut = np.array([
            [_nsl.nsl_getAmplitudeColor(z).r / 255.0,
             _nsl.nsl_getAmplitudeColor(z).g / 255.0,
             _nsl.nsl_getAmplitudeColor(z).b / 255.0]
            for z in range(NSL_EDGE_DETECTED+1)
        ], dtype=np.float32)
        return ret
    
    def get_distance_color(self, value):
        return _nsl.nsl_getDistanceColor(value)
        
    def get_amplitude_color(self, value):
        return _nsl.nsl_getAmplitudeColor(value)

    # ---------------- 벡터화 함수 ----------------
    def get_distance_color_array(self, value_array):
        """
        value_array: 2D NumPy 배열 (int)
        반환: 2D NumPy 3채널 BGR 이미지, dtype=uint8
        """

        clipped = np.clip(value_array, 0, NSL_EDGE_DETECTED)
        colors = (self.dist_color_lut[clipped] * 255).astype(np.uint8)   # (H, W, 3)
        out = colors[..., ::-1]
        return out

    def get_amplitude_color_array(self, value_array):
        """
        value_array: 2D NumPy 배열 (int)
        반환: 2D NumPy 3채널 BGR 이미지, dtype=uint8
        """
        clipped = np.clip(value_array, 0, NSL_EDGE_DETECTED)
        colors = (self.ampl_color_lut[clipped] * 255).astype(np.uint8)   # (H, W, 3)
        out = colors[..., ::-1]
        return out
        
    def close(self):
        _nsl.nsl_close()
        print("[NanoLidar] Closed")


