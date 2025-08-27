"""
NanoLiDAR Python Port (ctypes + numpy + OpenCV/Open3D)
- C API: nanolib.dll / libnanolib.so 를 ctypes로 직접 호출
- Frame 데이터: numpy 변환
- 시각화: OpenCV(2D), Open3D(3D; 선택)
"""


import os
import sys
import ctypes
from ctypes import (
    c_int, c_double, c_char_p, c_ubyte, c_bool, Structure, POINTER
)
import cv2
import numpy as np
import time

# ----------------------------------
# 0) 라이브러리 로드 (경로 수정)
# ----------------------------------
# Windows -> "nanolib.dll",  Linux -> "./libnanolib.so"
DLL_PATHS = ["./nsl_lib/lib/linux/shared/libnanolib.so", "./nsl_lib/lib/windows/shared/nanolib.dll", "/usr/local/lib/libnanolib.so"]
_nsl = None
for p in DLL_PATHS:
    if os.path.exists(p):
        _nsl = ctypes.CDLL(p)
        break
if _nsl is None:
    # 마지막 시도: 시스템 경로에서
    try:
        print("not found DLL_PATHS")
        _nsl = ctypes.CDLL("nanolib.dll")
    except Exception:
        try:
            print("not found windows dll")
            _nsl = ctypes.CDLL("libnanolib.so")
        except Exception as e:
            raise RuntimeError(
                "nanolib 동적라이브러리를 찾을 수 없습니다. DLL/so 경로를 수정하세요."
            ) from e


#enum in nanolib.h
FUNC_OFF, FUNC_ON = 0, 1

# pixel data range
NSL_LIMIT_FOR_VALID_DATA = 64000
NSL_LOW_AMPLITUDE		 = 64001
NSL_ADC_OVERFLOW		 = 64002
NSL_SATURATION			 = 64003
NSL_BAD_PIXEL 			 = 64004
NSL_LOW_DCS				 = 64005
NSL_INTERFERENCE		 = 64007
NSL_EDGE_DETECTED		 = 64008

# Streaming mode
DISTANCE_MODE = 1
GRAYSCALE_MODE = 2
DISTANCE_AMPLITUDE_MODE = 3
DISTANCE_GRAYSCALE_MODE = 4
RGB_MODE = 5
RGB_DISTANCE_MODE = 6
RGB_DISTANCE_AMPLITUDE_MODE = 7
RGB_DISTANCE_GRAYSCALE_MODE = 8

NSL_LIDAR_TYPE_A_WIDTH  = 320
NSL_LIDAR_TYPE_A_HEIGHT = 240
NSL_LIDAR_TYPE_B_WIDTH  = 800
NSL_LIDAR_TYPE_B_HEIGHT = 600
NSL_RGB_IMAGE_WIDTH     = 1920
NSL_RGB_IMAGE_HEIGHT    = 1080

OUT_X                   = 0
OUT_Y                   = 1
OUT_Z                   = 2
MAX_OUT                 = 3

LENS_NF                 = 0
LENS_SF                 = 1
LENS_WF                 = 2

  
NSL_SUCCESS = 0  # 보통 0 성공. SDK와 다르면 수정

# -------------------------------------- class -------------------------------------------------#
# --- viewerInfo ---
class ViewerInfo:
    def __init__(self):
        self.mouseX = -1
        self.mouseY = -1
        self.drawframeCount = 0
        self.temperature = 0.0
        self.fps = 0
        self.ipAddress = "192.168.0.220"
        self.start_time = time.time()
        # ----------------------------------
        # area settings
        self.area_left               = -800
        self.area_right              = 1500
        self.area_top                = -500
        self.area_bottom             = 500
        self.area_start              = 0
        self.area_end                = 3000
        self.area_inCount            = 0
        self.area_enable             = True
        
    def updateFps(self):
        self.fps += 1
        elapsed = time.time() - self.start_time
                
        if elapsed > 1:
            self.drawframeCount = int(self.fps / elapsed)
            self.fps = 0
            self.start_time = time.time()
            return self.fps

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
        ("grayscaleIlluminationOpt", ctypes.c_int),
    ]

# NslVec3b structure in nanolib.h
class NslVec3b(ctypes.Structure):
    _fields_ = [
        ("b", c_ubyte),
        ("g", c_ubyte),
        ("r", c_ubyte),
    ]
   
# NslPCD structure in nanolib.h
class NslPCD(Structure):
    _fields_ = [
        ("operationMode", c_int),
        ("lidarType", c_int),
        ("temperature", c_double),
        ("includeRgb", c_bool),
        ("includeLidar", c_bool),
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
        ("rgb", NslVec3b * (NSL_RGB_IMAGE_HEIGHT * NSL_RGB_IMAGE_WIDTH)),
    ]

    # ---- numpy 변환 도우미 ----
    def np_distance2D(self):
        arr = np.ctypeslib.as_array(self.distance2D)
        return arr.reshape((NSL_LIDAR_TYPE_B_HEIGHT, NSL_LIDAR_TYPE_B_WIDTH))
        
    def np_amplitude(self):
        arr = np.ctypeslib.as_array(self.amplitude)
        return arr.reshape((NSL_LIDAR_TYPE_B_HEIGHT, NSL_LIDAR_TYPE_B_WIDTH))

    def np_distance3D(self):
        arr = np.ctypeslib.as_array(self.distance3D)
        return arr.reshape((MAX_OUT, NSL_LIDAR_TYPE_B_HEIGHT, NSL_LIDAR_TYPE_B_WIDTH))

    def np_rgb(self):
        arr = np.ctypeslib.as_array(self.rgb)
        arr = arr.view(np.uint8).reshape((NSL_RGB_IMAGE_HEIGHT, NSL_RGB_IMAGE_WIDTH, 3))
        return arr

# Python Wrapper
class NanoLidar:
    def __init__(self, ip="192.168.0.220", debug=FUNC_ON):
                
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

        _nsl.nsl_getDistanceColor.argtypes = [c_int]
        _nsl.nsl_getDistanceColor.restype  = NslVec3b

        _nsl.nsl_getAmplitudeColor.argtypes = [c_int]
        _nsl.nsl_getAmplitudeColor.restype  = NslVec3b

        self.cfg = NslConfig()
        self.cfg.lensType = LENS_SF     #필수 인자
        self.cfg.lidarAngle = 0         #필수 인자
        self.handle = _nsl.nsl_open(ip.encode("utf-8"), ctypes.byref(self.cfg), debug)
        if self.handle < 0:
            raise RuntimeError("nsl_open 실패")
        print(f"[NanoLidar] Opened. handle={self.handle}")

    def start_stream(self, mode=DISTANCE_AMPLITUDE_MODE):
        ret = _nsl.nsl_streamingOn(self.handle, mode)
        if ret != NSL_SUCCESS:
            raise RuntimeError(f"nsl_streamingOn 실패 (ret={ret}, mode={mode})")
        print(f"[NanoLidar] Streaming ON (mode={mode})")

    def stop_stream(self):
        _nsl.nsl_streamingOff(self.handle)
        print("[NanoLidar] Streaming OFF")

    def get_frame(self, timeout_ms=1000) -> NslPCD | None:
        frame = NslPCD()
        ret = _nsl.nsl_getPointCloudData(self.handle, ctypes.byref(frame), timeout_ms)
        if ret == NSL_SUCCESS:
            return frame
        return None
        
    def get_distance_color(self, value):
        return _nsl.nsl_getDistanceColor(value)
        
    def get_amplitude_color(self, value):
        return _nsl.nsl_getAmplitudeColor(value)
        
    def close(self):
        _nsl.nsl_close()
        print("[NanoLidar] Closed")

# -------------------------------------- global -------------------------------------------------#

viewerInfo = ViewerInfo()
winName = "Python example"  

# -------------------------------------- function -------------------------------------------------#
# --- mouseCallbackCV ---
def mouseCallbackCV(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONUP:
        viewerInfo.mouseX = x
        viewerInfo.mouseY = y

# --- addDistanceInfo ---
def addDistanceInfo(distMat, frame, lidarWidth, lidarHeight, scaleSize):
    height, width = frame.height, frame.width
    viewer_xpos, viewer_ypos = viewerInfo.mouseX, viewerInfo.mouseY
    textSize = 0.8
    xMin, yMin = frame.roiXMin, frame.roiYMin

    xpos = viewer_xpos // scaleSize if viewer_xpos >= 0 else -1
    ypos = viewer_ypos // scaleSize if viewer_ypos >= 0 else -1

    infoImage = np.full((80, distMat.shape[1], 3), 255, dtype=np.uint8)

    if ypos >= yMin and ypos < lidarHeight and xpos >= 0:
        # 십자선 표시
        cv2.line(distMat, (viewer_xpos-13, viewer_ypos), (viewer_xpos+13, viewer_ypos), (255,255,0), 2)
        cv2.line(distMat, (viewer_xpos, viewer_ypos-15), (viewer_xpos, viewer_ypos+15), (255,255,0), 2)

        if xpos >= lidarWidth:
            xpos -= lidarWidth

        distance2D = frame.np_distance2D()[ypos,xpos]
        distance3D = frame.np_distance3D()[OUT_Z,ypos,xpos]

        # 값 해석 (간단화)
        if int(distance3D) > NSL_LIMIT_FOR_VALID_DATA:
            if( int(distance3D) == NSL_ADC_OVERFLOW ):
                dist2D_caption = f"X:{xpos},Y:{ypos} ADC_OVERFLOW"
            elif( int(distance3D) == NSL_SATURATION ):
                dist2D_caption = f"X:{xpos},Y:{ypos} SATURATION"
            elif( int(distance3D) == NSL_BAD_PIXEL ):
                dist2D_caption = f"X:{xpos},Y:{ypos} BAD_PIXEL"
            elif( int(distance3D) == NSL_INTERFERENCE ):
                dist2D_caption = f"X:{xpos},Y:{ypos} INTERFERENCE"
            elif( int(distance3D) == NSL_EDGE_DETECTED ):
                dist2D_caption = f"X:{xpos},Y:{ypos} EDGE_DETECTED"
            else:
                dist2D_caption = f"X:{xpos},Y:{ypos} LOW_AMPLITUDE"
                
            dist3D_caption = ""
        else:
            if frame.operationMode == DISTANCE_AMPLITUDE_MODE or frame.operationMode == RGB_DISTANCE_AMPLITUDE_MODE:
                amp = frame.np_amplitude()[ypos][xpos]
                dist2D_caption = f"2D X:{xpos} Y:{ypos} {distance2D}mm/{amp}lsb"
            else:
                dist2D_caption = f"2D X:{xpos} Y:{ypos} {distance2D}mm"

            dist3D_caption = (
                f"3D X:{frame.np_distance3D()[OUT_X,ypos,xpos]:.1f}mm "
                f"Y:{frame.np_distance3D()[OUT_Y,ypos,xpos]:.1f}mm "
                f"Z:{distance3D:.1f}mm"
            )

        info_caption = f"{width}x{height} < {viewerInfo.drawframeCount}fps> {viewerInfo.temperature:.2f}'C"

        cv2.putText(infoImage, info_caption, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, textSize, (0,0,0), 1, cv2.LINE_AA)
        cv2.putText(infoImage, dist2D_caption, (10, 46), cv2.FONT_HERSHEY_SIMPLEX, textSize, (0,0,0), 1, cv2.LINE_AA)
        cv2.putText(infoImage, dist3D_caption, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, textSize, (0,0,0), 1, cv2.LINE_AA)

    else:
        info_caption = f"{width}x{height} <{viewerInfo.drawframeCount}fps> {viewerInfo.temperature:.2f}'C"
        cv2.putText(infoImage, info_caption, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, textSize, (0,0,0), 1, cv2.LINE_AA)

    distMat = np.vstack((distMat, infoImage))
    return distMat


def visualize_loop(ip="192.168.0.220", mode=DISTANCE_AMPLITUDE_MODE, with_open3d=True):
    import cv2

    # Open3D는 선택
    use_o3d = False
    lidar = NanoLidar(ip)
    try:
        lidar.start_stream(mode)
        o3d_vis = None
        pcl_cloud = None
        cloud_points = None
        cloud_colors = None
        sphere = None
        
        if with_open3d:
            try:
                import open3d as o3d
                use_o3d = True

                o3d_vis = o3d.visualization.Visualizer()
                o3d_vis.create_window(window_name="PointCloud (Open3D)", width=960, height=720, visible=True)
                cloud_points = np.zeros((NSL_LIDAR_TYPE_A_HEIGHT*NSL_LIDAR_TYPE_A_WIDTH, 3), dtype=np.float32)
                cloud_colors = np.zeros_like(cloud_points)
                pcl_cloud = o3d.geometry.PointCloud()
                # 초기 빈 포인트클라우드 추가
                o3d_vis.get_render_option().point_size = 1.0
                o3d_vis.add_geometry(pcl_cloud)

                
                if viewerInfo.area_enable:

                    xmin = viewerInfo.area_left / 1000.0
                    xmax = viewerInfo.area_right / 1000.0
                    ymin = -viewerInfo.area_top / 1000.0
                    ymax = -viewerInfo.area_bottom / 1000.0
                    zmin = -viewerInfo.area_start / 1000.0
                    zmax = -viewerInfo.area_end / 1000.0

                    aabb = o3d.geometry.AxisAlignedBoundingBox(
                        min_bound=(xmin, ymin, zmin),
                        max_bound=(xmax, ymax, zmax)
                    )

                    # 4) LineSet으로 변환 후 색상 지정
                    box_lines = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(aabb)
                    box_lines.paint_uniform_color([1, 0, 0])  # 빨강
                    o3d_vis.add_geometry(box_lines)

                print("open3D를 지원 합니다.")
            except Exception:
                print("[WARN] open3d 미설치: 3D 시각화를 건너뜁니다.")
                use_o3d = False
                

        print("[INFO] ESC 종료, 'c' 컬러맵 토글, 'r' RGB 창 토글")
        first_frame = True

        while True:
            frame = lidar.get_frame(timeout_ms=1000)
            if frame is None:
                # 프레임 미수신 시 잠깐 대기
                time.sleep(0.005)
                continue
            viewerInfo.temperature = frame.temperature
            # --------- 2D Distance / Amplitude ----------
            # uint8 3채널 BGR 이미지
            imageDistance  = np.zeros((NSL_LIDAR_TYPE_A_HEIGHT, NSL_LIDAR_TYPE_A_WIDTH, 3), dtype=np.uint8)
            imageAmplitude = np.zeros_like(imageDistance)
            rgb = np.zeros((NSL_RGB_IMAGE_HEIGHT, NSL_RGB_IMAGE_WIDTH, 3), dtype=np.uint8)

            dist2d = frame.np_distance2D()
            amplitude    = frame.np_amplitude()
            dist3d = frame.np_distance3D()
            
            index = 0
            viewerInfo.area_inCount = 0
            xMin = frame.roiXMin
            yMin = frame.roiYMin
                        
            for y in range(frame.height):
                for x in range(frame.width):
                    # 2D 이미지 색상
                    col_dist  = lidar.get_distance_color(dist2d[y+yMin, x+xMin])
                    col_amp = lidar.get_amplitude_color(amplitude[y+yMin, x+xMin])
                    imageDistance[y+yMin, x+xMin] = [col_dist.b, col_dist.g, col_dist.r]
                    imageAmplitude[y+yMin, x+xMin] = [col_amp.b, col_amp.g, col_amp.r]

                    if use_o3d:
                        z_val = dist3d[OUT_Z, y+yMin, x+xMin]
                                                    
                        if z_val < NSL_LIMIT_FOR_VALID_DATA:
                            # 3D 좌표 (m 단위)
                            cloud_points[index, OUT_X] = dist3d[OUT_X, y+yMin, x+xMin] / 1000.0
                            cloud_points[index, OUT_Y] = -dist3d[OUT_Y, y+yMin, x+xMin] / 1000.0
                            cloud_points[index, OUT_Z] = -z_val / 1000.0

                            # 색상
                            if viewerInfo.area_enable:
                                x_val = dist3d[OUT_X, y+yMin, x+xMin]
                                y_val = dist3d[OUT_Y, y+yMin, x+xMin]

                                if (viewerInfo.area_left <= x_val <= viewerInfo.area_right and
                                    viewerInfo.area_top <= y_val <= viewerInfo.area_bottom and
                                    viewerInfo.area_start <= z_val <= viewerInfo.area_end):
                                    color3D = lidar.get_distance_color(int(z_val))
                                    cloud_colors[index] = [color3D.r/255.0, color3D.g/255.0, color3D.b/255.0]
                                    viewerInfo.area_inCount += 1
                                else:
                                    cloud_colors[index] = [196/255.0]*3
                            else:
                                color3D = lidar.get_distance_color(int(z_val))
                                cloud_colors[index] = [color3D.r/255.0, color3D.g/255.0, color3D.b/255.0]
                        
                        index += 1


            # --------- RGB ----------
            if frame.includeLidar:
                if use_o3d:
                    pcl_cloud.points = o3d.utility.Vector3dVector(cloud_points[:index])
                    pcl_cloud.colors = o3d.utility.Vector3dVector(cloud_colors[:index])
                    o3d_vis.update_geometry(pcl_cloud)
                    o3d_vis.poll_events()
                    o3d_vis.update_renderer()
                    if first_frame:
                        first_frame = False
                        o3d_vis.reset_view_point(True)

                # 합쳐서 보기
                vis2d = np.hstack([imageDistance, imageAmplitude])
                if frame.includeRgb:
                    rgb = frame.np_rgb()
                    small = cv2.resize(rgb, (640, 240))
                    vis2d = np.vstack([vis2d, small])
                    
                vis2d = addDistanceInfo(vis2d, frame, 320, 240, 1)
                cv2.imshow(winName, vis2d)
                
            elif frame.includeRgb:
                rgb = frame.np_rgb()

                small = cv2.resize(rgb, (640, 480))
                cv2.imshow(winName, small)

            # ---------- 키 처리 ----------
            k = cv2.waitKey(1) & 0xFF
            if k == 27:  # ESC
                break
                
            viewerInfo.updateFps()

    finally:
        try:
            lidar.stop_stream()
            lidar.close()
        except Exception:
            pass
            lidar.close()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
  

# excute
if __name__ == "__main__":
    """
    사용법:
      python main.py 필요시 IP/모드/3D 사용 여부를 수정하세요.
    """
    
    cv2.namedWindow(winName)
    cv2.setMouseCallback(winName, mouseCallbackCV)

    visualize_loop(
        ip=viewerInfo.ipAddress,
        mode=RGB_DISTANCE_MODE,  # DISTANCE_AMPLITUDE_MODE, RGB_DISTANCE_MODE
        with_open3d=False         # 3D 뷰 필요 없으면 False or True
    )
