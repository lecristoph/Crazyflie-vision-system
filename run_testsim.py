import logging
import time
import numpy as np
import cv2
import yaml
import math
import sys
import os
import traceback
from simple_pid import PID
from filterpy.kalman import KalmanFilter

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

import arucoCam as arc
from plotData import plotData

err_dict = {
    "1": "Could not reshape. Multiple markers detected ?",
    "2": "No Crazyflies found, cannot run test",
    "3": "Something went wrong. Closing webcam.",
    "4": "Battery level is low. landing...",
    "5": "Unknown error. Check traceback:"
}


def getData():
    with SyncLogger(scf, lg_stab) as logger:
        for log_entry in logger:
            data = log_entry[1]
            y = data.get('stabilizer.yaw')
            vbat = data.get('pm.vbat')
            roll = data.get('stabilizer.roll')
            pitch = data.get('stabilizer.pitch')
            time.sleep(0.02)
            return(y, vbat, roll, pitch)


def getCFparams(scf, available):
    try:
        yaw_CF, vbat, roll, pitch = np.asarray(getData()).tolist()
    except TypeError:
        print("Error: ", sys.exc_info()[0])
        print(TypeError)
        traceback.print_exc()

    if np.size(vbat) != 0:
        return(yaw_CF, vbat, roll, pitch)
    else:
        return(0, 0)


def angCor(alpha):
            if alpha > 0:
                alpha = 180 - alpha
            else:
                alpha = abs(alpha) - 180
            return(alpha)


def getMarkerAngles(cam):
        # Read image frame from camera
        [ret, img] = cam.read()  # Take snapshot from camera
        # Undistort the frame
        img = arc.undistort(img, cMatrix, k_dist)
        # Detect ArUco marker
        # corners = [x1, y1; x2, y2; ..., x4, y4] where x1,y1 = top left corner
        corners, rvec, tvec, ids, imgWithAruco = arc.ArUcoDetect(img, cMatrix,
                                                                 k_dist, m_len,
                                                                 aruco_dict,
                                                                 arucoParams)
        # Get angles between camera and marker with ID=2
        if ids is not None and id2find in ids:
            idx_r, idx_c = np.where(ids == id2find)
            tvec = tvec[idx_r, :]
            rvec = rvec[idx_r, :]
            # draw rectangle around marker with ID=2
            corners = np.asarray(corners)
            corners = corners[idx_r, :]
            corners = np.reshape(corners, [4, 2])
            imgWithAruco = arc.drawRec(imgWithAruco, corners)
            EulerAngles = arc.angleFromAruco(rvec, tvec, ids)
        else:
            EulerAngles = None
        if EulerAngles is not None:
            psi = EulerAngles[0]*180/math.pi  # yaw
            theta = EulerAngles[1]*180/math.pi  # pitch
            phi = EulerAngles[2]*180/math.pi  # roll
            # Corrent for rotation
            psi = angCor(psi)
            EulerAngles = np.array([psi, theta, phi])
            EulerAngles_rot = arc.rotate(EulerAngles)
            alpha = EulerAngles_rot[0, 0]  # yaw
            return(alpha, imgWithAruco, tvec)
        else:
            return(None, imgWithAruco, None)


class CF:
    def __init__(self, scf, available):
        # get yaw-angle and battery level of crazyflie
        self.psi, self.vbat, self.roll, self.pitch = getCFparams(scf, available)
        self.scf = scf
        self.available = available
        self.mc = MotionCommander(scf)
        self.psi_limit = 0.7    # Don't cmd an angle less than this [deg]
        self.des_angle = 0  # Set to zero initially

    def update(self, des_angle, eul, turnRate):
        if 'psi' in eul and abs(des_angle) > self.psi_limit:
            self.des_angle = des_angle
            if des_angle > 0:
                pass
            else:
                pass
        # Compute current angle (yaw) of CF
        self.psi, self.vbat, self.roll, self.pitch = getCFparams(self.scf, self.available)
        return(self.psi, self.vbat, self.roll, self.pitch)

    # Move cf left or right
    def update_x(self, dist):
        if dist is not None and dist != 0:
            pass
        elif dist is not None and dist == 0 and self.des_angle == 0:
            pass

    def takeoff(self):
        self.mc.take_off()

    def land(self):
        self.mc.land()

    def move_distance(self, x, y, z):
        '''
        Move in a straight line, {CF frame}.
        positive X is forward [cm]
        positive Y is left [cm]
        positive Z is up [cm]
        '''
        velocity = 0.08
        x = x/100
        y = y/100
        z = z/100

        distance = math.sqrt(x**2 + y**2 + z**2)

        if x != 0:
            flight_time = distance / velocity
            velocity_x = velocity * x / distance
        else:
            velocity_x = 0
        if y != 0:
            velocity_y = velocity * y / distance
        else:
            velocity_y = 0
        if z != 0:
            velocity_z = velocity * z / distance
        else:
            velocity_z = 0

        if x != 0:
            time.sleep(flight_time)
        return(False)


if __name__ == '__main__':
    # Define radio interface
    URI = 'radio://0/80/2M'
    # Initialize camera
    cam = cv2.VideoCapture(1)
    # Name of plot figure output
    figname = "experimentData.png"
    # Title of plot
    figtitle = "Battery level"
    # Name of dumpfile
    dumpfile_name = "testData1.yaml"
    # Do we want live display or not ?
    dispLiveVid = True
    # Marker side length [cm]
    m_len = 19.0
    # Marker ID to find:
    id2find = 24
    # Init write VideoWriter
    FILE_OUTPUT = 'output.avi'
    # Checks and deletes the output file
    # You cant have a existing file or it will through an error
    try:
        if os.path.isfile(FILE_OUTPUT):
            os.remove(FILE_OUTPUT)
    except PermissionError:
        print("Error: ", sys.exc_info()[0])
        traceback.print_exc()

    # Get width of camera capture
    width = (int(cam.get(3)))
    # Get height of camera capture
    height = (int(cam.get(4)))
    # Define some text parameters
    font = cv2.FONT_HERSHEY_SIMPLEX
    s1 = (10, int(height-40))
    s2 = (10, int(height-60))
    s3 = (10, int(height-80))
    s4 = (10, int(height-100))
    s5 = (10, int(height-120))
    s6 = (10, int(height-140))
    fontScale = 0.6
    fontColor = (255, 255, 255)  # white
    lineType = 2
    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(FILE_OUTPUT, fourcc, 8.0, ((width), (height)))
    # Open camera calibration data
    with open('calibration.yaml') as f:
        calParams = yaml.load(f)
    # Load camera matrix and distortion coeffs
    cMatrix = calParams.get('camera_matrix')
    k_dist = calParams.get('dist_coeff')
    # Convert to numpy arrays
    cMatrix = np.array(cMatrix)
    k_dist = np.array(k_dist)
    # keep track of values for plotting
    setpoint = np.array([])
    yaw_cf = np.array([])
    roll_cf = np.array([])
    pitch_cf = np.array([])
    ar_time = np.array([])
    ar_bat = np.array([])

    aruco_dict, arucoParams = arc.createAruco()

    # Init Kalman filter
    x = 8   # n.o. elements in state vector
    z = 4   # n.o dimensions in state vector
    f = KalmanFilter(dim_x=x, dim_z=z)  # Yaw filter
    # Uncertainty factor
    unc_f = 0.04
    # Define Kalman filter state estimate vector
    f.x = np.zeros((x, 1), dtype='float')
    # state transition matrix
    f.F = (np.eye(x))
    f.F[0, 1] = 1.0
    f.F[2, 3] = 1.0
    f.F[4, 5] = 1.0
    f.F[6, 7] = 1.0
    # Measurement function
    f.H = np.zeros((z, x), dtype='float')
    # Define elements to which we perform calculations
    f.H[0, 0] = 1.0
    f.H[1, 2] = 1.0
    f.H[2, 4] = 1.0
    f.H[3, 6] = 1.0
    # state uncertainty
    f.R *= unc_f
    # process noise
    f.Q *= unc_f*0.1

    low_battery = 3.0  # Don't take of if battery level < low_battery [V]
    nf_max = 8  # Define maximum number of estimated frames in a row
    nf = nf_max + 1  # Define current number of estimated frames
    alpha = 0.91  # Inertial factor for velocity degregation
    samples_max = 10   # Define sample heap size for moving avg filter
    zs = np.array([])   # Array for moving average filter (yaw)
    ds = np.array([])   # Array for moving average filter (transl_z)
    xs = np.array([])   # --||-- (transl_X = right/left in img)
    ys = np.array([])   # --||-- (tranls_Y = height)
    arb = np.array([])  # Array for moving avg filter (battery level)
    r = np.array([0])  # Array for storing location and velocity estimates(yaw)
    r2 = np.array([0])  # --||-- (transl_z)
    r3 = np.array([0])  # --||-- (transl_x)
    r4 = np.array([0])  # --||-- (transl_y)

    # PID Gains
    Kp_psi = 1.40  # 1.5
    Kd_psi = 0.15  # 0.345
    Ki_psi = 0.55  # 0.62
    Kp_x = 7.24  # 7.24
    Kd_x = 0.46  # 0.46
    Ki_x = 6.9  # 0.13.89
    turnRate = 360/4    # 360/4.5 is default value

    # Only output errors from the logging framework
    logging.basicConfig(level=logging.ERROR)
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) == 0:
        print(err_dict["2"])
    else:
        cf = Crazyflie(rw_cache='./cache')
        # Add logging config to logger
        lg_stab = LogConfig(name='Stabilizer', period_in_ms=12)
        lg_stab.add_variable('pm.vbat', 'float')
        lg_stab.add_variable('stabilizer.yaw', 'float')
        lg_stab.add_variable('stabilizer.roll', 'float')
        lg_stab.add_variable('stabilizer.pitch', 'float')
        with SyncCrazyflie(URI) as scf:
            # Define motion commander to crazyflie as mc:
            cf2 = CF(scf, available)
            cf2_psi = cf2.psi    # Get current yaw-angle of cf
            cf2_bat = cf2.vbat  # Get current battery level of cf
            cf2_phi = cf2.roll
            cf2_theta = cf2.pitch
            marker_psi = 0  # Set marker angle to zero initially

            pid = PID(0, 0, 0, setpoint=marker_psi)  # Initialize PID
            pid_x = PID(0, 0, 0, setpoint=0)  # init PID for roll
            # Define pid parameters
            pid.tunings = (Kp_psi, Kd_psi, Ki_psi)
            pid_x.tunings = (Kp_x, Kd_x, Ki_x)
            pid_x.sample_time = 0.05    # update pid every 50 ms
            pid.output_limits = (-60, 60)
            pid_x.output_limits = (-20, 20)
            pid.proportional_on_measurment = False
            pid_x.proportional_on_measurment = False

            if cf2_bat >= low_battery:
                # cf2.takeoff()    # CF takes off from ground
                print("Taking off ! Battery level: " + str(cf2_bat))
                time.sleep(1.5)   # Let the CF hover for 1.5s

            t_init = time.time()
            endTime = t_init + 30000   # Let the CF do its thing for some time

            # Add values to array for logging/plotting
            setpoint = np.append(setpoint, marker_psi)  # Commanded yaw
            yaw_cf = np.append(yaw_cf, cf2.psi)  # Actual yaw of CF
            roll_cf = np.append(roll_cf, cf2.roll)
            pitch_cf = np.append(pitch_cf, cf2.pitch)
            ar_time = np.append(ar_time, 0)    # Time vector
            ar_bat = np.append(ar_bat, cf2_bat)

            A = scf.is_link_open()
            while time.time() < endTime and cf2_bat >= low_battery and A:
                # Get angle of marker
                try:
                    # Get marker yaw-angle, img and translation vector(xyz)
                    marker_psi, img, t_vec = getMarkerAngles(cam)

                    # Get translational distance in Z-dir, marker->lens
                    if t_vec is not None:
                        try:
                            t_vec = np.asarray(t_vec).tolist()
                            t_vec = np.reshape(t_vec, [1, 3])
                            dist_x = t_vec[0, 0]
                            dist_y = t_vec[0, 1]
                            dist_z = t_vec[0, 2]
                        except ValueError:
                            print(err_dict["1"])
                    else:
                        dist_z = None
                        dist_x = None
                        dist_y = None

                    # Moving average filter
                    if marker_psi is not None:
                        zs = np.append(zs, marker_psi)
                        ds = np.append(ds, dist_z)
                        xs = np.append(xs, dist_x)
                        ys = np.append(ys, dist_y)
                        marker_psi = np.average(zs)
                        dist_z = np.average(ds)
                        dist_x = np.average(xs)
                        dist_y = np.average(ys)
                        if np.size(zs) >= samples_max:
                            zs = np.delete(zs, 0)
                            ds = np.delete(ds, 0)
                            xs = np.delete(xs, 0)
                            ys = np.delete(ys, 0)

                    # perform kalman filtering
                    if marker_psi is not None:
                        f.update(np.array([marker_psi,
                                           dist_z, dist_x, dist_y]))
                        f.predict()
                    else:
                        f.update(marker_psi)
                        f.predict()

                    # Set inertial degregation to velocity for each iteration
                    if marker_psi is None and nf < nf_max:
                        # f.x holds the estimated values
                        f.x[1, 0] *= alpha
                        f.x[3, 0] *= alpha
                        f.x[5, 0] *= alpha
                        f.x[7, 0] *= alpha
                        yaw_cmd = f.x[0, 0]  # commanded yaw
                        transl_z_cmd = round(f.x[2, 0], 2)  # comm transl_z
                        transl_x_cmd = round(f.x[4, 0], 2)  # comm tranls_x
                        transl_y_cmd = round(f.x[6, 0], 2)  # comm transl_y
                        nf += 1
                    elif marker_psi is not None and marker_psi != 0:
                        nf = 0
                        yaw_cmd = marker_psi  # commanded yaw
                        transl_z_cmd = round(dist_z, 2)   # commanded transl_z
                        transl_x_cmd = round(dist_x, 2)   # commanded transl_x
                        transl_y_cmd = round(dist_y, 2)   # commanded transl_y
                    else:
                        yaw_cmd = 0
                        transl_z_cmd = 0
                        transl_x_cmd = 0
                        transl_y_cmd = 0

                except (cv2.error, TypeError, AttributeError):
                    print(err_dict["3"])
                    print("Error: ", sys.exc_info()[0])
                    cam.release()
                    cv2.destroyAllWindows()
                    # cf2.land()  # Land the CF2
                    print(TypeError)
                    traceback.print_exc()
                    break

                dt = time.time() - t_init  # compute elapsed time

                # print text on image
                cv2.putText(img, "t: "+str(round(dt, 2)),
                            s1,
                            font,
                            fontScale,
                            fontColor,
                            lineType)
                cv2.putText(img, "psi: "+str(round(yaw_cmd, 2)),
                            s2,
                            font,
                            fontScale,
                            fontColor,
                            lineType)
                cv2.putText(img, "Z: "+str(transl_z_cmd),
                            s3,
                            font,
                            fontScale,
                            fontColor,
                            lineType)
                cv2.putText(img, "X: "+str(transl_x_cmd),
                            s4,
                            font,
                            fontScale,
                            fontColor,
                            lineType)
                cv2.putText(img, "Battery: "+str(round(cf2_bat, 2)),
                            s5,
                            font,
                            fontScale,
                            fontColor,
                            lineType)

                if dispLiveVid:
                    cv2.imshow('image', img)    # display current frame

                    if cv2.waitKey(25) & 0xFF == ord('q'):
                        # cf2.land()
                        cv2.destroyAllWindows()
                        break

                out.write(img)  # write img frame to video file
                # Send command signal to CF2 and update cf2_psi
                # 'psi' = yaw
                # 'theta' = pitch
                # 'phi' = roll
                # 'z' = forwards/backwards
                #  ** Send signal to CF + get battery level and yaw-angle
                try:
                    if yaw_cmd is not None:
                        cval_psi = pid(yaw_cmd)
                        cf2_psi, cf2_bat, cf2_phi, cf2_theta = cf2.update(-cval_psi,
                                                      'psi', turnRate)
                    if transl_x_cmd is not None:
                        cval_x = pid_x(transl_x_cmd)
                        cf2.update_x(round(-cval_x, 1))
                    time.sleep(0.03)
                except (UnboundLocalError, TypeError, ZeroDivisionError):
                    print(err_dict["5"])
                    print("Error: ", sys.exc_info()[0])
                    cam.release()
                    cv2.destroyAllWindows()
                    # cf2.land()
                    traceback.print_exc()
                    scf.close_link()

                # Moving avg filter ,battery:
                if cf2_bat is not None:
                    arb = np.append(arb, cf2_bat)
                    cf2_bat = np.average(arb)
                    if np.size(arb) >= samples_max:
                        arb = np.delete(arb, 0)

                # Do some logging of data for plotting
                setpoint = np.append(setpoint, marker_psi)  # Commanded yaw
                yaw_cf = np.append(yaw_cf, cf2_psi)  # Actual yaw of CF
                roll_cf = np.append(roll_cf, cf2_phi)
                pitch_cf = np.append(pitch_cf, cf2_theta)
                ar_time = np.append(ar_time, dt)    # Time vector
                r = np.append(r, f.x[0, 0])  # Estimated yaw
                r2 = np.append(r2, f.x[2, 0])  # Estimated z distance
                r3 = np.append(r3, f.x[4, 0])   # Estimated x distance
                r4 = np.append(r4, f.x[6, 0])   # Estimated y distance
                ar_bat = np.append(ar_bat, cf2_bat)  # Battery level

                A = scf.is_link_open()  # Check that link is still open
            # ****** END OF MAIN LOOP ********* #
            if cf2_bat <= low_battery:
                print(err_dict["4"])
                print("Battery level: " + str(cf2_bat))
                # cf2.land()
            else:
                print("Test completed. landing...")
                # Test sending a signal to CF: Move forward [cm]:
                # arguments: Forward(+), left(+), up(+)
                # if values are negative; opposite direction
                # cf2.move_distance(round(transl_z_cmd-15), 0, 0)
                # Land the CF2
                time.sleep(0.1)
                # cf2.land()
                print("Landed.")

# Important to close webcam connection when everything is done
if 'scf' in locals():
    scf.close_link()    # Close connection to CF
out.release()
cam.release()
cv2.destroyAllWindows()
if len(available) != 0:
    # Save data to yaml-file for analysis
    pid_settings = np.array([Kp_psi, Kd_psi, Ki_psi, turnRate])
    data = {'timestamp': np.asarray(ar_time).tolist(),
            'battery_level': np.asarray(ar_bat).tolist(),
            'yaw_cf': np.asarray(yaw_cf).tolist(),
            'roll_cf': np.asarray(roll_cf).tolist(),
            'pitch_cf': np.asarray(pitch_cf).tolist(),
            'yaw_commanded': np.asarray(setpoint).tolist(),
            'Kalman_yaw_estimation': np.asarray(r).tolist(),
            'Kalman_z_dist_estimation': np.asarray(r2).tolist(),
            'Kalman_x_dist_estimation': np.asarray(r3).tolist(),
            'Kalman_y_dist_estimation': np.asarray(r4).tolist(),
            'PID_settings': np.asarray(pid_settings).tolist()}

    # Save data to file
    with open(dumpfile_name, "w") as f:
            yaml.dump(data, f)

    # Plot results
    plotData(dumpfile_name, figname, figtitle)
