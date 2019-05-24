import numpy as np
import yaml
import matplotlib.pyplot as plt


def plotData(filename, figname, figtitle):

    params = {'legend.fontsize': 'small',
              'figure.figsize': (15, 8),
              'axes.labelsize': 'small',
              'axes.titlesize': 'medium',
              'xtick.labelsize': 'small',
              'ytick.labelsize': 'small'}

    # Open camera test data file
    with open(filename) as f:
        testData = yaml.load(f)
    with open(filename) as fn:
        testData2 = yaml.load(fn)

    # Load camera matrix and distortion coeffs
    PID_settings = testData.get('PID_settings')
    time_array = testData.get('timestamp')
    time_array2 = testData2.get('timestamp')
    cf_yaw = testData.get('yaw_cf')
    cf_roll = testData.get('roll_cf')
    cf_pitch = testData.get('pitch_cf')
    cmd_yaw = testData.get('yaw_commanded')
    kalmanYaw = testData.get('Kalman_yaw_estimation')
    KalmanX = testData.get('Kalman_x_dist_estimation')
    blevel = testData.get('voltage_level')
    temp = testData.get('temperature')
    pwm = testData.get('pwm')
    bleveln = testData2.get('battery_level')

    # Convert to numpy arrays
    time_array = np.array(time_array)
    time_array2 = np.array(time_array2)
    PID_settings = np.array(PID_settings)
    cf_yaw = np.array(cf_yaw)
    cf_roll = np.array(cf_roll)
    cf_pitch = np.array(cf_pitch)
    cmd_yaw = np.array(cmd_yaw)
    blevel = np.array(blevel)
    bleveln = np.array(bleveln)
    new = np.array([])

    for n in range(np.size(cmd_yaw)):
        if cmd_yaw[n] is not None:
            new = np.append(new, cmd_yaw[n])
        else:
            new = np.append(new, kalmanYaw[n])

    error = cf_yaw - new

    # Calc percentage of charge
    blevel[0] = blevel[1]
    bat_range = max(blevel) - min(blevel)
    corrstart = blevel - min(blevel)
    bsoc = (corrstart*100)/bat_range

    bleveln[0] = bleveln[1]
    bat_rangen = max(bleveln) - min(bleveln)
    corrstart = bleveln - min(bleveln)
    bsocn = (corrstart*100)/bat_rangen

    cf_roll[0] = cf_roll[1]

    pwm = (np.array(pwm) / 65535)*100
    pwm[0] = pwm[1]

    def plMinMax(valuearray, timearray):
        # calc max/min for yaw
        y_max = np.argmax(valuearray)
        xmax = timearray[y_max]
        ymax = round(valuearray[y_max], 2)
        y_min = np.argmin(valuearray)
        xmin = timearray[y_min]
        ymin = round(valuearray[y_min], 2)
        markers_on = [y_min, y_max]
        return(y_max, xmax, ymax, y_min, xmin, ymin, markers_on)

    y_max, xmax, ymax, y_min, xmin, ymin, markers_on = plMinMax(temp, time_array)
    # Plot data
    plt.rcParams.update(params)
    f, axarr = plt.subplots(3, sharex=True)
    axarr[0].plot(time_array, blevel, '-g', label='Voltage level')
    # axarr[0].plot(time_array2, bsocn, '-b', label='Without camera module')
    axarr[0].set_title('Battery voltage level: With camera module (turned off)')
    axarr[0].set_xlabel('Time [s]')
    axarr[0].set_ylabel('Volage level [V]')
    axarr[0].legend(loc='best')
    axarr[0].grid(True)

    axarr[1].plot(time_array, temp, '-bD', markevery=markers_on, label="Measured on-board temperature")
    axarr[1].annotate(str(ymin), xy=(xmin+2.9, ymin))
    axarr[1].annotate(str(ymax), xy=(xmax+1.0, ymax-0.3))
    axarr[1].set_title('Temperature')
    axarr[1].set_xlabel('Time [s]')
    axarr[1].set_ylabel('Temperature [degree Celsius]')
    axarr[1].legend(loc='best')
    axarr[1].grid(True)

    axarr[2].plot(time_array, pwm, '-r', label="PWM")
    axarr[2].set_title('PWM signal output to motors [%]')
    axarr[2].set_xlabel('Time [s]')
    axarr[2].set_ylabel('PWM to motors (percentage of maximum)')
    axarr[2].legend(loc='best')
    axarr[2].grid(True)

    plt.savefig(figname)
    plt.show()


if __name__ == '__main__':
    figtitle = 'Stationary position on ground'
    filename = 'batteryleveltest_withcamera_on.yaml'
    figname = 'batterytest2.png'
    plotData(filename, figname, figtitle)
