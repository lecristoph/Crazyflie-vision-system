import cv2
from cv2 import aruco
import numpy as np
import transforms3d as trans3d
import math


def undistort(img, cMatrix, k_dist):
    h,  w = img.shape[:2]   # Resolution of image, e.g. 480, 640
    [newcameramtx, roi] = cv2.getOptimalNewCameraMatrix(cMatrix, k_dist, (w, h), 1, (w, h))
    # undistort
    mapx, mapy = cv2.initUndistortRectifyMap(cMatrix,k_dist,None,newcameramtx,(w,h),5)
    dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
    return dst


# Draw rectangle on image
def drawRec(img, corners, color=(26, 238, 62), thickness=2):
    cv2.line(img, (int(corners[0, 0]), int(corners[0, 1])),
                  (int(corners[1, 0]), int(corners[1, 1])),
                  (color), thickness)
    cv2.line(img, (int(corners[1, 0]), int(corners[1, 1])),
                  (int(corners[2, 0]), int(corners[2, 1])),
                  (color), thickness)
    cv2.line(img, (int(corners[2, 0]), int(corners[2, 1])),
                  (int(corners[3, 0]), int(corners[3, 1])),
                  (color), thickness)
    cv2.line(img, (int(corners[3, 0]), int(corners[3, 1])),
                  (int(corners[0, 0]), int(corners[0, 1])),
                  (color), thickness)
    return(img)


def createAruco():
    # Set up the ArUco dictionary and paramters
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    # Create ArUco detector with default parameters
    arucoParams = aruco.DetectorParameters_create()
    arucoParams.minMarkerPerimeterRate = 0.05
    arucoParams.maxMarkerPerimeterRate = 1.5
    return(aruco_dict, arucoParams)


# Detects the ArUco Marker and returns id, trans/rot-vecs and image
def ArUcoDetect(img, cMatrix, k_dist, m_len, aruco_dict, arucoParams):
    # Convert to gray (required by detector)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Detect aruco
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img_gray, aruco_dict, parameters=arucoParams)

    if isinstance(ids, object):  # if aruco marker was detected4
        # Estimate pose of single marker
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, m_len, cMatrix, k_dist)
        # Draw border around detected marker
        imgWithAruco = aruco.drawDetectedMarkers(img, corners, ids, (0, 0, 255))
    else:   # if aruco marker is NOT detected
        imgWithAruco = img  # assign imRemapped_color to imgWithAruco directly

    return corners, rvec, tvec, ids, imgWithAruco


def angleFromAruco(rvec, tvec, ids):
    if ids is not None:
        R = cv2.Rodrigues(rvec, tvec)[0]    # Retreive rotation matrix
        R = np.matrix(R)
        # Calculate Euler angles from rotation matrix
        EulerAngles = trans3d.euler.mat2euler(R, axes='syxz')
        return EulerAngles
    else:
        pass


def rotate(v):
    if v is not None:
        v = np.array([v[0], v[1], v[2]])
        # grab ang from vector
        psi = v[0]*math.pi/180
        theta = -v[1]*math.pi/180
        phi = -v[2]*math.pi/180
        # Define rotation matrix around x and y
        Rxy = np.matrix([[math.cos(theta), -math.sin(theta), 0],
                        [-math.sin(phi), math.cos(phi), 0],
                        [0, 0, 1]])
        #Rxy = np.matrix([[1, 0, 0],
        #                 [0, 1, 0],
        #                 [0, 0, 1]])
        # Define 180 degree rotation matrix
        Rxpi = np.matrix([[-1, 0, 0],
                          [0, -1, 0],
                          [0, 0, -1]])
        # Compensate for rotation in 3d and 4th quadrant
        if v[2] >= -45 and v[2] <= 45:
            v_new = Rxy.T @ v @ Rxpi.T
        else:
            v_new = Rxy.T @ v
        if -v[2] >= 45 and -v[2] <= 135:
            v_new = np.matrix([v[1], v[0], v[2]]) @ Rxpi.T
        elif -v[2] <= -45 and -v[2] >= -135:
            v_new = np.matrix([v[1], v[0], v[2]])
        return(v_new)
    else:
        return(v)
