#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import platform
import ctypes
from ctypes import (
    c_int, c_double, c_char_p, c_ubyte, c_bool, Structure, POINTER, byref
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
NSL_LIDAR_TYPE_A_WIDTH  = 320
NSL_LIDAR_TYPE_A_HEIGHT = 240
NSL_LIDAR_TYPE_B_WIDTH  = 800
NSL_LIDAR_TYPE_B_HEIGHT = 600
NSL_RGB_IMAGE_WIDTH     = 1920
NSL_RGB_IMAGE_HEIGHT    = 1080

MAX_GRAYSCALE_VALUE		= 2897

#max ditance :: mm
MAX_DISTANCE_24MHZ		= 6250
MAX_DISTANCE_12MHZ		= 12500
MAX_DISTANCE_6MHZ		= 25000
MAX_DISTANCE_3MHZ		= 50000

# pixel data range
NSL_LIMIT_FOR_VALID_DATA = 64000
NSL_LOW_AMPLITUDE		 = 64001
NSL_ADC_OVERFLOW		 = 64002
NSL_SATURATION			 = 64003
NSL_BAD_PIXEL 			 = 64004
NSL_LOW_DCS				 = 64005
NSL_INTERFERENCE		 = 64007
NSL_EDGE_DETECTED		 = 64008

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

# UDP_SPEED_OPTIONS
NET_100Mbps     = 0
NET_1000Mbps    = 1

# DUALBEAM_MOD_OPTIONS
DB_OFF  = 0
DB_6MHZ = 1
DB_3MHZ = 2

# DUALBEAM_OPERATION_OPTIONS
DB_AVOIDANCE        = 0
DB_CORRECTION       = 1
DB_FULL_CORRECTION  = 2

# MODULATION_OPTIONS
MOD_12Mhz   = 0
MOD_24Mhz   = 1
MOD_6Mhz    = 2
MOD_3Mhz    = 3
MOD_1p5MHz  = 4

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

# LENS_TYPE
LENS_NF                 = 0
LENS_SF                 = 1
LENS_WF                 = 2

# LIDAR_TYPE_OPTIONS
TYPE_A = 0  # 320x240 lidar type 
TYPE_B = 1  # 800x600 lidar type

# OPERATION_MODE_OPTIONS
NONE_MODE                   = 0
DISTANCE_MODE               = 1
GRAYSCALE_MODE              = 2
DISTANCE_AMPLITUDE_MODE     = 3
DISTANCE_GRAYSCALE_MODE     = 4
RGB_MODE                    = 5
RGB_DISTANCE_MODE           = 6
RGB_DISTANCE_AMPLITUDE_MODE = 7
RGB_DISTANCE_GRAYSCALE_MODE = 8

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


# ImuData structure in nanolib.h
class ImuData(ctypes.Structure):
    _fields_ = [
        ("ax", ctypes.c_float),
        ("ay", ctypes.c_float),
        ("az", ctypes.c_float),
        ("gx", ctypes.c_float),
        ("gy", ctypes.c_float),
        ("gz", ctypes.c_float),
        ("qw", ctypes.c_float),
        ("qx", ctypes.c_float),
        ("qy", ctypes.c_float),
        ("qz", ctypes.c_float),
        ("roll", ctypes.c_float),
        ("pitch", ctypes.c_float),
        ("yaw", ctypes.c_float)
    ]    

# NslConfig structure in nanolib.h
class NslConfig(ctypes.Structure):
    _fields_ = [
        ("integrationTime3D", ctypes.c_int),
        ("integrationTime3DHdr1", ctypes.c_int),
        ("integrationTime3DHdr2", ctypes.c_int),
        ("integrationTimeGrayScale", ctypes.c_int),
        ("roiXMin", ctypes.c_int),
        ("roiXMax", ctypes.c_int),
        ("roiYMin", ctypes.c_int),
        ("roiYMax", ctypes.c_int),
        ("currentOffset", ctypes.c_int),
        ("minAmplitude", ctypes.c_int),
        ("firmware_release", ctypes.c_int),
        ("chipID", ctypes.c_int),
        ("waferID", ctypes.c_int),
        ("udpDataPort", ctypes.c_int),
        ("ledMask", ctypes.c_int),
        ("lidarAngle", ctypes.c_double),
        ("lidarType", ctypes.c_int),   # enum LIDAR_TYPE_OPTIONS
        ("lensType", ctypes.c_int),    # enum LENS_TYPE
        ("operationModeOpt", ctypes.c_int),
        ("hdrOpt", ctypes.c_int),
        ("mod_frequencyOpt", ctypes.c_int),
        ("mod_channelOpt", ctypes.c_int),
        ("mod_enabledAutoChannelOpt", ctypes.c_int),
        ("dbModOpt", ctypes.c_int),
        ("dbOpsOpt", ctypes.c_int),
        ("ver_binningOpt", ctypes.c_int),
        ("horiz_binningOpt", ctypes.c_int),
        ("overflowOpt", ctypes.c_int),
        ("saturationOpt", ctypes.c_int),
        ("drnuOpt", ctypes.c_int),
        ("temperatureOpt", ctypes.c_int),
        ("grayscaleOpt", ctypes.c_int),
        ("ambientlightOpt", ctypes.c_int),
        ("medianOpt", ctypes.c_int),
        ("gaussOpt", ctypes.c_int),
        ("temporalFactorValue", ctypes.c_int),
        ("temporalThresholdValue", ctypes.c_int),
        ("edgeThresholdValue", ctypes.c_int),
        ("interferenceDetectionLimitValue", ctypes.c_int),
        ("interferenceDetectionLastValueOpt", ctypes.c_int),
        ("edgeThresholdValue3D", ctypes.c_int),
        ("udpSpeedOpt", ctypes.c_int),
        ("frameRateOpt", ctypes.c_int),
        ("grayscaleIlluminationOpt", ctypes.c_int)
    ]
    
class NslAutoIntROI(ctypes.Structure):
    _fields_ = [
        ("x_start", c_int),
        ("y_start", c_int),
        ("x_end", c_int),
        ("y_end", c_int),
        ("max_overflow", c_int),
        ("min_intTime", c_int)
    ]
    
# NslVec3b structure in nanolib.h
class NslVec3b(ctypes.Structure):
    _fields_ = [
        ("b", c_ubyte),
        ("g", c_ubyte),
        ("r", c_ubyte)
    ]
    
    @property
    def __array_interface__(self):
        return {
            'descr': [('', np.uint8)],
            'shape': (3,),
            'typestr': np.dtype(np.uint8).str,
            'data': (ctypes.addressof(self), False)
        }
   
# NslPCD structure in nanolib.h
class NslPCD(Structure):
    _fields_ = [
        ("operationMode", c_int),
        ("lidarType", c_int),
        ("temperature", c_double),
        ("includeRgb", c_bool),
        ("includeLidar", c_bool),
        ("includeImu", c_bool),
        ("includeYml", c_bool),
        ("width", c_int),
        ("height", c_int),
        ("roiXMin", c_int),
        ("roiYMin", c_int),
        ("roiXMax", c_int),
        ("roiYMax", c_int),
        ("binning_h", c_int),
        ("binning_v", c_int),
        # 대용량 버퍼 (SDK 해상도 기준)
        ("amplitude", c_int * (NSL_LIDAR_TYPE_B_HEIGHT  * NSL_LIDAR_TYPE_B_WIDTH)),
        ("distance2D", c_int * (NSL_LIDAR_TYPE_B_HEIGHT  * NSL_LIDAR_TYPE_B_WIDTH)),
        ("distance3D", c_double * (MAX_OUT * NSL_LIDAR_TYPE_B_HEIGHT  * NSL_LIDAR_TYPE_B_WIDTH)),
        ("imuData", ImuData)
    ]

    def __init__(self, *args, **kwargs):
        super(NslPCD, self).__init__(*args, **kwargs)

        self.distance2D_np = np.ctypeslib.as_array(self.distance2D).reshape(
            (NSL_LIDAR_TYPE_B_HEIGHT, NSL_LIDAR_TYPE_B_WIDTH)
        )
        self.amplitude_np = np.ctypeslib.as_array(self.amplitude).reshape(
            (NSL_LIDAR_TYPE_B_HEIGHT, NSL_LIDAR_TYPE_B_WIDTH)
        )
        self.distance3D_np = np.ctypeslib.as_array(self.distance3D).reshape(
            (MAX_OUT, NSL_LIDAR_TYPE_B_HEIGHT, NSL_LIDAR_TYPE_B_WIDTH)
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
    def __init__(self, ip="192.168.0.220", lens_type = LENS_SF, lidar_angle = 0.0, debug=FUNC_ON):
                
        _nsl.nsl_open.argtypes  = [c_char_p, POINTER(NslConfig), c_int]
        _nsl.nsl_open.restype   = c_int   # handle 반환

        _nsl.nsl_close.argtypes = []
        _nsl.nsl_close.restype  = c_int

        _nsl.nsl_streamingOn.argtypes  = [c_int, c_int]
        _nsl.nsl_streamingOn.restype   = c_int

        _nsl.nsl_streamingOff.argtypes = [c_int]
        _nsl.nsl_streamingOff.restype  = c_int

        _nsl.nsl_getPointCloudData.argtypes = [c_int, POINTER(NslPCD), c_int]
        _nsl.nsl_getPointCloudData.restype  = c_int

        _nsl.nsl_getPointCloudRgbData.argtypes = [c_int, POINTER(NslPCD), POINTER(NslVec3b), c_int]
        _nsl.nsl_getPointCloudRgbData.restype  = c_int
        
        _nsl.nsl_setFrameRate.argtypes = [c_int, c_int]
        _nsl.nsl_setFrameRate.restype  = c_int
        
        _nsl.nsl_setUdpSpeed.argtypes = [c_int, c_int]
        _nsl.nsl_setUdpSpeed.restype  = c_int

        _nsl.nsl_setAdcOverflowSaturation.argtypes = [c_int, c_int, c_int]
        _nsl.nsl_setAdcOverflowSaturation.restype  = c_int

        _nsl.nsl_setMinAmplitude.argtypes = [c_int, c_int]
        _nsl.nsl_setMinAmplitude.restype  = c_int

        _nsl.nsl_setIntegrationTime.argtypes = [c_int, c_int, c_int, c_int, c_int]
        _nsl.nsl_setIntegrationTime.restype  = c_int

        _nsl.nsl_setHdrMode.argtypes = [c_int, c_int]
        _nsl.nsl_setHdrMode.restype  = c_int

        _nsl.nsl_setDualBeam.argtypes = [c_int, c_int, c_int]
        _nsl.nsl_setDualBeam.restype  = c_int
        
        _nsl.nsl_setModulation.argtypes = [c_int, c_int, c_int, c_int]
        _nsl.nsl_setModulation.restype  = c_int
        
        _nsl.nsl_setFilter.argtypes = [c_int, c_int, c_int, c_int, c_int, c_int, c_int, c_int]
        _nsl.nsl_setFilter.restype  = c_int

        _nsl.nsl_set3DFilter.argtypes = [c_int, c_int]
        _nsl.nsl_set3DFilter.restype  = c_int

        _nsl.nsl_setBinning.argtypes = [c_int, c_int, c_int]
        _nsl.nsl_setBinning.restype  = c_int

        _nsl.nsl_setRoi.argtypes = [c_int, c_int, c_int, c_int, c_int]
        _nsl.nsl_setRoi.restype  = c_int

        _nsl.nsl_setCorrection.argtypes = [c_int, c_int, c_int, c_int, c_int]
        _nsl.nsl_setCorrection.restype  = c_int

        _nsl.nsl_saveConfiguration.argtypes = [c_int]
        _nsl.nsl_saveConfiguration.restype  = c_int

        _nsl.nsl_setColorRange.argtypes = [c_int, c_int, c_int]
        _nsl.nsl_setColorRange.restype  = c_int
        
        _nsl.nsl_getDistanceColor.argtypes = [c_int]
        _nsl.nsl_getDistanceColor.restype  = NslVec3b

        _nsl.nsl_getAmplitudeColor.argtypes = [c_int]
        _nsl.nsl_getAmplitudeColor.restype  = NslVec3b
        
        _nsl.nsl_setAutoIntegrationTime.argtypes = [c_int, POINTER(NslAutoIntROI), c_int]
        _nsl.nsl_setAutoIntegrationTime.restype  = c_int

        _nsl.nsl_getAutoIntegrationTime.argtypes = [c_int, POINTER(c_int), POINTER(c_int), POINTER(c_int)]
        _nsl.nsl_getAutoIntegrationTime.restype  = c_int

        _nsl.nsl_getCurrentConfig.argtypes = [c_int, POINTER(NslConfig)]
        _nsl.nsl_getCurrentConfig.restype  = c_int
        
        _nsl.nsl_getDepthAtPixel.argtypes = [c_int, c_int, c_int, POINTER(NslPCD)]
        _nsl.nsl_getDepthAtPixel.restype  = c_double
        
        _nsl.nsl_getPixelAtDepth.argtypes = [c_int, c_int, c_int, POINTER(NslVec3b)]
        _nsl.nsl_getPixelAtDepth.restype  = NslVec3b
        
        self.rgb = (NslVec3b * (NSL_RGB_IMAGE_HEIGHT * NSL_RGB_IMAGE_WIDTH))()
        self.rgb_np = np.ctypeslib.as_array(self.rgb).view(np.uint8).reshape(
            (NSL_RGB_IMAGE_HEIGHT, NSL_RGB_IMAGE_WIDTH, 3)
        )
        
        self.cfg = NslConfig()
        self.cfg.lensType = lens_type     #필수 인자
        self.cfg.lidarAngle = lidar_angle #필수 인자
        self.handle = _nsl.nsl_open(ip.encode("utf-8"), byref(self.cfg), debug)
        if self.handle < 0:
            raise RuntimeError("nsl_open 실패: ip = {}".format(ip))

        _nsl.nsl_setColorRange(MAX_DISTANCE_12MHZ, MAX_GRAYSCALE_VALUE, FUNC_ON)

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

    def toString_UDP_SPEED_OPTIONS(self, c):
        mapping = {
            NET_100Mbps: "NET_100Mbps",
            NET_1000Mbps: "NET_1000Mbps",
        }
        return mapping.get(c, "Unknown->NET_1000Mbps")

    def toString_DUALBEAM_MOD_OPTIONS(self, c):
        mapping = {
            DB_OFF: "DB_OFF",
            DB_6MHZ: "DB_6MHZ",
            DB_3MHZ: "DB_3MHZ",
        }
        return mapping.get(c, "Unknown->DB_3MHZ")

    def toString_DUALBEAM_OPERATION_OPTIONS(self, c):
        mapping = {
            DB_AVOIDANCE: "DB_AVOIDANCE",
            DB_CORRECTION: "DB_CORRECTION",
            DB_FULL_CORRECTION: "DB_FULL_CORRECTION",
        }
        return mapping.get(c, "Unknown->DB_AVOIDANCE")

    def toString_MODULATION_OPTIONS(self, c):
        mapping = {
            MOD_12Mhz: "MOD_12Mhz",
            MOD_24Mhz: "MOD_24Mhz",
            MOD_6Mhz: "MOD_6Mhz",
            MOD_3Mhz: "MOD_3Mhz",
            MOD_1p5MHz: "MOD_1.5Mhz",
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

    def toString_LIDAR_TYPE_OPTIONS(self, c):
        mapping = {
            TYPE_A: "TYPE_A",
            TYPE_B: "TYPE_B",
        }
        return mapping.get(c, "Unknown")

    def toString_OPERATION_MODE_OPTIONS(self, c):
        mapping = {
            NONE_MODE: "NONE_MODE",
            DISTANCE_MODE: "DISTANCE_MODE",
            GRAYSCALE_MODE: "GRAYSCALE_MODE",
            DISTANCE_AMPLITUDE_MODE: "DISTANCE_AMPLITUDE_MODE",
            DISTANCE_GRAYSCALE_MODE: "DISTANCE_GRAYSCALE_MODE",
            RGB_MODE: "RGB_MODE",
            RGB_DISTANCE_MODE: "RGB_DISTANCE_MODE",
            RGB_DISTANCE_AMPLITUDE_MODE: "RGB_DISTANCE_AMPLITUDE_MODE",
            RGB_DISTANCE_GRAYSCALE_MODE: "RGB_DISTANCE_GRAYSCALE_MODE",
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
        ret = _nsl.nsl_getCurrentConfig(self.handle, byref(self.cfg))
        print("------------------------------------------------------------------------")
        print("------------------------- Device configuration -------------------------")
        print("------------------------------------------------------------------------")

        firmware_major = (self.cfg.firmware_release >> 16) & 0xFFFF
        firmware_minor = self.cfg.firmware_release & 0xFFFF
        print("firmware version = {0}.{1}".format(firmware_major, firmware_minor))

        print("waferID = {0}".format(self.cfg.waferID))
        print("chipID = {0}".format(self.cfg.chipID))
        print("UDP RX port = {0}".format(self.cfg.udpDataPort))

        print("HDR = {0}".format(self.toString_HDR_OPTIONS(self.cfg.hdrOpt)))

        print("int time = {0}.{1}.{2}.{3}".format(
            self.cfg.integrationTime3D,
            self.cfg.integrationTime3DHdr1,
            self.cfg.integrationTime3DHdr2,
            self.cfg.integrationTimeGrayScale))
            
        print("minimum amplitude = {0}".format(self.cfg.minAmplitude))

        print("roi = {0},{1},{2},{3}".format(
            self.cfg.roiXMin, self.cfg.roiYMin, self.cfg.roiXMax, self.cfg.roiYMax))

        print("Modulation = {0}, ch = {1}, autoChannel = {2}".format(
            self.toString_MODULATION_OPTIONS(self.cfg.mod_frequencyOpt),
            self.toString_MODULATION_CH_OPTIONS(self.cfg.mod_channelOpt),
            self.toString_FUNCTION_OPTIONS(self.cfg.mod_enabledAutoChannelOpt)))

        print("dual beam = {0}, option = {1}".format(
            self.toString_DUALBEAM_MOD_OPTIONS(self.cfg.dbModOpt),
            self.toString_DUALBEAM_OPERATION_OPTIONS(self.cfg.dbOpsOpt)))

        print("Binning vertical = {0}, horizontal = {1}".format(
            self.toString_FUNCTION_OPTIONS(self.cfg.ver_binningOpt),
            self.toString_FUNCTION_OPTIONS(self.cfg.horiz_binningOpt)))

        print("adc overflow = {0}, saturation = {1}".format(
            self.toString_FUNCTION_OPTIONS(self.cfg.overflowOpt),
            self.toString_FUNCTION_OPTIONS(self.cfg.saturationOpt)))

        print("Compensation drnu = {0}, temperature = {1}, grayscale = {2}, ambient = {3}".format(
            self.toString_FUNCTION_OPTIONS(self.cfg.drnuOpt),
            self.toString_FUNCTION_OPTIONS(self.cfg.temperatureOpt),
            self.toString_FUNCTION_OPTIONS(self.cfg.grayscaleOpt),
            self.toString_FUNCTION_OPTIONS(self.cfg.ambientlightOpt)))

        print("filter median = {0}, gauss = {1}, temporal factor = {2}, "
              "temporal threshold = {3}, edge threshold = {4}, "
              "interferenceLimit = {5}, used interference Last value = {6}".format(
                  self.toString_FUNCTION_OPTIONS(self.cfg.medianOpt),
                  self.toString_FUNCTION_OPTIONS(self.cfg.gaussOpt),
                  self.cfg.temporalFactorValue,
                  self.cfg.temporalThresholdValue,
                  self.cfg.edgeThresholdValue,
                  self.cfg.interferenceDetectionLimitValue,
                  self.toString_FUNCTION_OPTIONS(self.cfg.interferenceDetectionLastValueOpt)))

        print("3D edge filter theshold = {0}".format(self.cfg.edgeThresholdValue3D))


        print("UDP speed = {0}".format(self.toString_UDP_SPEED_OPTIONS(self.cfg.udpSpeedOpt)))
        print("frame rate = {0}".format(self.toString_FRAME_RATE_OPTIONS(self.cfg.frameRateOpt)))
        print("------------------------------------------------------------------------")
    
    def get_rgb_np(self):
        return self.rgb_np
        
    def is_rgb_command(self):
        return self.cfg.operationModeOpt in (
            RGB_MODE,
            RGB_DISTANCE_MODE,
            RGB_DISTANCE_AMPLITUDE_MODE,
            RGB_DISTANCE_GRAYSCALE_MODE,
        )
        
    def get_nsl_config(self):
        return self.cfg
            
    def start_stream(self, mode=DISTANCE_AMPLITUDE_MODE):
        #if( mode 
        ret = _nsl.nsl_streamingOn(self.handle, mode)
        if ret != NSL_SUCCESS:
            raise RuntimeError("nsl_streamingOn 실패 (mode={0}) ret = {1}".format(self.toString_OPERATION_MODE_OPTIONS(mode), self.toString_NSL_ERROR_TYPE(ret)))
        self.cfg.operationModeOpt = mode
        print("[NanoLidar] Streaming ON (mode={0}) ret = {1}".format(self.toString_OPERATION_MODE_OPTIONS(mode), self.toString_NSL_ERROR_TYPE(ret)))

    def stop_stream(self):
        ret = _nsl.nsl_streamingOff(self.handle)
        self.cfg.operationModeOpt = 0
        print("[NanoLidar] Streaming OFF")
        return ret

    def get_frame(self, frame, timeout_ms=1000):
        if self.is_rgb_command():
            return _nsl.nsl_getPointCloudRgbData(self.handle, byref(frame), self.rgb, timeout_ms)
            
        return _nsl.nsl_getPointCloudData(self.handle, byref(frame), timeout_ms)
        
    def set_frame_rate(self, FRAME_RATE_OPTIONS):
        return _nsl.nsl_setFrameRate(self.handle, FRAME_RATE_OPTIONS) 

    def set_udp_speed(self, UDP_SPEED_OPTIONS):
        return _nsl.nsl_setUdpSpeed(self.handle, UDP_SPEED_OPTIONS) 
        
    def set_adcOverflow(self, FUNCTION_OPTIONS_overflow, FUNCTION_OPTIONS_saturation):
        return _nsl.nsl_setAdcOverflowSaturation(self.handle, FUNCTION_OPTIONS_overflow, FUNCTION_OPTIONS_saturation) 

    def set_minimum_amplitude(self, amplitude):
        return _nsl.nsl_setMinAmplitude(self.handle, amplitude) 

    def set_intetration_time(self, intTime, intTimeHdr1, intTimeHdr2, intTimeGray):
        return _nsl.nsl_setIntegrationTime(self.handle, intTime, intTimeHdr1, intTimeHdr2, intTimeGray) 

    def set_auto_integration_time(self, roi, FUNCTION_OPTIONS_onoff):
        return _nsl.nsl_setAutoIntegrationTime(self.handle, byref(roi), FUNCTION_OPTIONS_onoff)
    
    def get_auto_integration_time(self, onoff, currentIntTime, overflowCnt):
        return _nsl.nsl_getAutoIntegrationTime(self.handle, byref(onoff), byref(currentIntTime), byref(overflowCnt))
     
    def set_hdr_mode(self, HDR_OPTIONS):
        return _nsl.nsl_setHdrMode(self.handle, HDR_OPTIONS) 

    def set_dual_beam(self, DUALBEAM_MOD_OPTIONS, DUALBEAM_OPERATION_OPTIONS):
        return _nsl.nsl_setDualBeam(self.handle, DUALBEAM_MOD_OPTIONS, DUALBEAM_OPERATION_OPTIONS) 

    def set_modulation(self, MODULATION_OPTIONS, MODULATION_CH_OPTIONS, FUNCTION_OPTIONS_auto):
        return _nsl.nsl_setModulation(self.handle, MODULATION_OPTIONS, MODULATION_CH_OPTIONS, FUNCTION_OPTIONS_auto) 

    def set_filters(self, FUNCTION_OPTIONS_Median, FUNCTION_OPTIONS_Gauss, temporalFactor, temporalThreshold, edgeThreshold, interferenceDetectionLimit, FUNCTION_OPTIONS_lastValue):
        return _nsl.nsl_setFilter(self.handle, FUNCTION_OPTIONS_Median, FUNCTION_OPTIONS_Gauss, temporalFactor, temporalThreshold, edgeThreshold, interferenceDetectionLimit, FUNCTION_OPTIONS_lastValue)

    def set_3d_filter(self, edgethreshold):
        return _nsl.nsl_set3DFilter(self.handle, edgethreshold) 

    def set_binning(self, FUNCTION_OPTIONS_vertical, FUNCTION_OPTIONS_horizontal):
        return _nsl.nsl_setBinning(self.handle, FUNCTION_OPTIONS_vertical, FUNCTION_OPTIONS_horizontal) 

    def set_roi(self, minX, minY, maxX, maxY):
        return _nsl.nsl_setRoi(self.handle, minX, minY, maxX, maxY)
                
    def set_conrrection(self, FUNCTION_OPTIONS_drnu, FUNCTION_OPTIONS_temperature, FUNCTION_OPTIONS_grayscale, FUNCTION_OPTIONS_ambient):
        return _nsl.nsl_setCorrection(self.handle, FUNCTION_OPTIONS_drnu, FUNCTION_OPTIONS_temperature, FUNCTION_OPTIONS_grayscale, FUNCTION_OPTIONS_ambient)

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
        
    def get_pixel_at_depth(self, depthX, depthY):
        return _nsl.nsl_getPixelAtDepth(self.handle, depthX, depthY, self.rgb)

    def get_depth_at_pixel(self, rgbX, rgbY, frame):
        return _nsl.nsl_getDepthAtPixel(self.handle, rgbX, rgbY, byref(frame))

    # ---------------- 벡터화 함수 ----------------
    def get_distance_color_array(self, value_array):
        """
        value_array: 2D NumPy 배열 (int)
        반환: 2D NumPy 3채널 BGR 이미지, dtype=uint8
        """
        value_array = np.asarray(value_array).copy()
        clipped = np.clip(value_array, 0, NSL_EDGE_DETECTED)
        colors = (self.dist_color_lut[clipped] * 255).astype(np.uint8)   # (H, W, 3)
        out = colors[..., ::-1]
        return np.ascontiguousarray(out)

    def get_amplitude_color_array(self, value_array):
        """
        value_array: 2D NumPy 배열 (int)
        반환: 2D NumPy 3채널 BGR 이미지, dtype=uint8
        """
        value_array = np.asarray(value_array).copy()
        clipped = np.clip(value_array, 0, NSL_EDGE_DETECTED)
        colors = (self.ampl_color_lut[clipped] * 255).astype(np.uint8)   # (H, W, 3)
        out = colors[..., ::-1]
        return np.ascontiguousarray(out)
        
    def close(self):
        _nsl.nsl_close()
        print("[NanoLidar] Closed")


