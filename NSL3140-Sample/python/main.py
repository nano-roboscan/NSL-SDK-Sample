"""
NanoLiDAR Python Port (ctypes + numpy + OpenCV/Open3D)
- C API: nanolib.dll / libnanolib.so 를 ctypes로 직접 호출
- Frame 데이터: numpy 변환
- 시각화: OpenCV(2D), Open3D(3D; 선택)
"""



#import sys
import ctypes
from ctypes import (
    c_int, c_double, c_char_p, c_ubyte, c_bool, Structure, POINTER
)
import cv2
import time
import numpy as np
import interface


# -------------------------------------- class -------------------------------------------------#
# --- viewerInfo ---
class ViewerInfo:
    def __init__(self):
        self.mouseX = -1
        self.mouseY = -1
        self.drawframeCount = 0
        self.temperature = 0.0
        self.fps = 0
        self.lensType = interface.LENS_SF
        self.lidarAngle = 0        
        self.ipAddress = "192.168.0.220"
        #self.ipAddress = "\\\\.\\COM12"
        self.operationMode = interface.DISTANCE_AMPLITUDE_MODE
        self.usedOpen3d = True
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
            print(f"fps : {self.drawframeCount} temp = {self.temperature:.2f} in-Count = {self.area_inCount}")
            return self.drawframeCount

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
        distance3D = frame.np_distance3D()[interface.OUT_Z,ypos,xpos]

        # 값 해석 (간단화)
        if int(distance3D) > interface.NSL_LIMIT_FOR_VALID_DATA:
            if( int(distance3D) == interface.NSL_ADC_OVERFLOW ):
                dist2D_caption = f"X:{xpos},Y:{ypos} ADC_OVERFLOW"
            elif( int(distance3D) == interface.NSL_SATURATION ):
                dist2D_caption = f"X:{xpos},Y:{ypos} SATURATION"
            elif( int(distance3D) == interface.NSL_BAD_PIXEL ):
                dist2D_caption = f"X:{xpos},Y:{ypos} BAD_PIXEL"
            elif( int(distance3D) == interface.NSL_INTERFERENCE ):
                dist2D_caption = f"X:{xpos},Y:{ypos} INTERFERENCE"
            elif( int(distance3D) == interface.NSL_EDGE_DETECTED ):
                dist2D_caption = f"X:{xpos},Y:{ypos} EDGE_DETECTED"
            else:
                dist2D_caption = f"X:{xpos},Y:{ypos} LOW_AMPLITUDE"
                
            dist3D_caption = ""
        else:
            if frame.operationMode == interface.DISTANCE_AMPLITUDE_MODE or frame.operationMode == interface.RGB_DISTANCE_AMPLITUDE_MODE:
                amp = frame.np_amplitude()[ypos][xpos]
                dist2D_caption = f"2D X:{xpos} Y:{ypos} {distance2D}mm/{amp}lsb"
            else:
                dist2D_caption = f"2D X:{xpos} Y:{ypos} {distance2D}mm"

            dist3D_caption = (
                f"3D X:{frame.np_distance3D()[interface.OUT_X,ypos,xpos]:.1f}mm "
                f"Y:{frame.np_distance3D()[interface.OUT_Y,ypos,xpos]:.1f}mm "
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


def visualize_loop():

    cv2.namedWindow(winName)
    cv2.setMouseCallback(winName, mouseCallbackCV)

    # Open3D는 선택
    use_o3d = False
    o3d_vis = None
    pcl_cloud = None
    cloud_points = None
    cloud_colors = None
    color_lut = None
    
    if viewerInfo.usedOpen3d:
        try:
            import open3d as o3d
            use_o3d = True

            o3d_vis = o3d.visualization.Visualizer()
            o3d_vis.create_window(window_name="PointCloud (Open3D)", width=960, height=720, visible=True)

            cloud_points = np.zeros((interface.NSL_LIDAR_TYPE_A_HEIGHT*interface.NSL_LIDAR_TYPE_A_WIDTH, 3), dtype=np.float32)
            cloud_colors = np.zeros_like(cloud_points)
            pcl_cloud = o3d.geometry.PointCloud()

            o3d_vis.get_render_option().point_size = 1.0
            o3d_vis.add_geometry(pcl_cloud)

            if viewerInfo.area_enable:

                xmin = viewerInfo.area_left / 1000.0
                xmax = viewerInfo.area_right / 1000.0
                ymax = -viewerInfo.area_top / 1000.0
                ymin = -viewerInfo.area_bottom / 1000.0
                zmax = -viewerInfo.area_start / 1000.0
                zmin = -viewerInfo.area_end / 1000.0

                aabb = o3d.geometry.AxisAlignedBoundingBox(
                    min_bound=(xmin, ymin, zmin),
                    max_bound=(xmax, ymax, zmax)
                )

                box_lines = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(aabb)
                box_lines.paint_uniform_color([1, 0, 0])  # 빨강
                o3d_vis.add_geometry(box_lines)

            print("open3D를 지원 합니다.")
        except Exception:
            print("[WARN] open3d 미설치: 3D 시각화를 건너뜁니다.")
            use_o3d = False
            
    
    lidar = interface.NanoLidar(viewerInfo.ipAddress, viewerInfo.lensType, viewerInfo.lidarAngle)
    lidar.set_filters(interface.FUNC_ON, interface.FUNC_ON, 300, 200, 100, 0, interface.FUNC_OFF)
    lidar.set_3d_filter(100)
    lidar.set_frame_rate(interface.FRAME_30FPS)
#    lidar.set_color_range(interface.MAX_DISTANCE_12MHZ, interface.MAX_GRAYSCALE_VALUE, interface.FUNC_OFF)
    
    color_3d_lut = np.array([
        [lidar.get_distance_color(z).r / 255.0,
         lidar.get_distance_color(z).g / 255.0,
         lidar.get_distance_color(z).b / 255.0]
        for z in range(interface.NSL_LIMIT_FOR_VALID_DATA)
    ], dtype=np.float32)
    
    try:
        lidar.start_stream(viewerInfo.operationMode)

        print("[INFO] ESC or 'q' 종료")

        draw_2d = True
        first_frame = True

        imageDistance  = np.zeros((interface.NSL_LIDAR_TYPE_A_HEIGHT, interface.NSL_LIDAR_TYPE_A_WIDTH, 3), dtype=np.uint8)
        imageAmplitude = np.zeros_like(imageDistance)
        rgb = np.zeros((interface.NSL_RGB_IMAGE_HEIGHT, interface.NSL_RGB_IMAGE_WIDTH, 3), dtype=np.uint8)

        frame = interface.NslPCD()

        
        while True:
            # ---------- 키 처리 ----------
            k = cv2.waitKey(10) & 0xFF
            if k == 27 or k == ord('q') or k == ord('Q'):  # ESC, q, Q
                break
            elif k == ord('d') or k == ord('D'):
                draw_2d = not draw_2d
                print("draw_2d = ", draw_2d)
                
            ret = lidar.get_frame(frame, timeout_ms=1000)
            if ret != interface.NSL_SUCCESS:
                continue
                
            viewerInfo.area_inCount = 0
            viewerInfo.temperature  = frame.temperature            

            xMin, yMin  = frame.roiXMin, frame.roiYMin
            dist2d      = frame.np_distance2D()
            amplitude   = frame.np_amplitude()
            dist3d      = frame.np_distance3D()
            
            # --------- 2D Distance / Amplitude ----------
            # uint8 3채널 BGR 이미지
                        
            if frame.includeLidar:
                
                if draw_2d:
                    # ROI 영역 선택
                    dist_roi = dist2d[0:interface.NSL_LIDAR_TYPE_A_HEIGHT, 0:interface.NSL_LIDAR_TYPE_A_WIDTH]
                    amp_roi = amplitude[0:interface.NSL_LIDAR_TYPE_A_HEIGHT, 0:interface.NSL_LIDAR_TYPE_A_WIDTH]

                     # 2D 이미지 벡터화 처리
                    imageDistance[0:interface.NSL_LIDAR_TYPE_A_HEIGHT, 0:interface.NSL_LIDAR_TYPE_A_WIDTH] = lidar.get_distance_color_array(dist_roi)
                    imageAmplitude[0:interface.NSL_LIDAR_TYPE_A_HEIGHT, 0:interface.NSL_LIDAR_TYPE_A_WIDTH] = lidar.get_amplitude_color_array(amp_roi)

                    # 합쳐서 보기
                    vis2d = np.hstack([imageDistance, imageAmplitude])
                    if frame.includeRgb:
                        rgb = frame.np_rgb()
                        small = cv2.resize(rgb, (640, 240))
                        vis2d = np.vstack([vis2d, small])
                        
                    vis2d = addDistanceInfo(vis2d, frame, 320, 240, 1)
                    cv2.imshow(winName, vis2d)
    
                
                if use_o3d:
                    x_roi = dist3d[interface.OUT_X, 0:interface.NSL_LIDAR_TYPE_A_HEIGHT, 0:interface.NSL_LIDAR_TYPE_A_WIDTH]
                    y_roi = dist3d[interface.OUT_Y, 0:interface.NSL_LIDAR_TYPE_A_HEIGHT, 0:interface.NSL_LIDAR_TYPE_A_WIDTH]
                    z_roi = dist3d[interface.OUT_Z, 0:interface.NSL_LIDAR_TYPE_A_HEIGHT, 0:interface.NSL_LIDAR_TYPE_A_WIDTH]
                    
                    cloud_points_np = np.zeros((interface.NSL_LIDAR_TYPE_A_HEIGHT*interface.NSL_LIDAR_TYPE_A_WIDTH, 3), dtype=np.float64)
                    cloud_colors_np = np.full((interface.NSL_LIDAR_TYPE_A_HEIGHT*interface.NSL_LIDAR_TYPE_A_WIDTH, 3), 196/255.0, dtype=np.float64)
                    
                    valid_mask = (z_roi < interface.NSL_LIMIT_FOR_VALID_DATA).flatten()
                    valid_indices = np.flatnonzero(valid_mask)
                    
                    x_valid = (x_roi.flatten())[valid_mask]
                    y_valid = (y_roi.flatten())[valid_mask]
                    z_valid = (z_roi.flatten())[valid_mask]
                    
                    cloud_points_np[valid_mask, 0] = x_valid / 1000.0
                    cloud_points_np[valid_mask, 1] = -y_valid / 1000.0
                    cloud_points_np[valid_mask, 2] = -z_valid / 1000.0

                    if viewerInfo.area_enable:

                        area_mask = (
                            (x_valid >= viewerInfo.area_left) &
                            (x_valid <= viewerInfo.area_right) &
                            (y_valid >= viewerInfo.area_top) &
                            (y_valid <= viewerInfo.area_bottom) &
                            (z_valid >= viewerInfo.area_start) &
                            (z_valid <= viewerInfo.area_end)
                        )

                        # 영역 안 포인트 색상
                        if np.any(area_mask):
                            area_indices = valid_indices[area_mask]
                            z_vals_int = np.clip(z_valid[area_mask].astype(int), 0, interface.NSL_LIMIT_FOR_VALID_DATA - 1)
                            cloud_colors_np[area_indices, :] = color_3d_lut[z_vals_int]
                                                
                            viewerInfo.area_inCount = np.sum(area_mask)
                        else:
                            viewerInfo.area_inCount = 0
                    else:
                        z_vals_int = np.clip(z_valid.astype(int), 0, interface.NSL_LIMIT_FOR_VALID_DATA - 1)
                        cloud_colors_np[valid_indices, :] = color_3d_lut[z_vals_int]
                    
                    if first_frame:
                        pcl_cloud.points = o3d.utility.Vector3dVector(cloud_points_np)
                        pcl_cloud.colors = o3d.utility.Vector3dVector(cloud_colors_np)
                        o3d_vis.add_geometry(pcl_cloud)
                        first_frame = False
                        o3d_vis.reset_view_point(True)
                    else:
                        np.asarray(pcl_cloud.points)[:] = cloud_points_np
                        np.asarray(pcl_cloud.colors)[:] = cloud_colors_np

                    o3d_vis.update_geometry(pcl_cloud)
                    o3d_vis.poll_events()
                    o3d_vis.update_renderer()   

            elif frame.includeRgb:
                rgb = frame.np_rgb()

                small = cv2.resize(rgb, (640, 480))
                cv2.imshow(winName, small)
                
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
    
    visualize_loop()
