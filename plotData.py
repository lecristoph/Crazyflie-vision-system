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

    # Load camera matrix and distortion coeffs
    PID_settings = testData.get('PID_settings')
    time_array = testData.get('timestamp')
    cf_yaw = testData.get('yaw_cf')
    cmd_yaw = testData.get('yaw_commanded')
    kalmanYaw = testData.get('Kalman_yaw_estimation')
    KalmanX = testData.get('Kalman_x_dist_estimation')
    blevel = testData.get('battery_level')

    # Convert to numpy arrays
    time_array = np.array(time_array)
    PID_settings = np.array(PID_settings)
    cf_yaw = np.array(cf_yaw)
    cmd_yaw = np.array(cmd_yaw)
    blevel = np.array(blevel)
    new = np.array([])

    for n in range(np.size(cmd_yaw)):
        if cmd_yaw[n] is not None:
            new = np.append(new, cmd_yaw[n])
        else:
            new = np.append(new, kalmanYaw[n])

    error = cf_yaw - new

    # Plot data
    plt.rcParams.update(params)
    f, axarr = plt.subplots(2, sharex=True)
    axarr[0].plot(time_array, cf_yaw, '-b', label='crazyflie measured yaw')
    axarr[0].plot(time_array, error, '-m', label="Commanded yaw error")
    axarr[0].plot(time_array, cmd_yaw, '-r', linestyle=':', label='measured marker yaw')
    axarr[0].plot(time_array, kalmanYaw, '-c', linestyle=':', label='Kalman estimated yaw')
    axarr[0].set_title(figtitle)
    axarr[0].set_xlabel('Time [s]')
    axarr[0].set_ylabel('Angle [degrees]')
    axarr[0].legend(loc='best')
    axarr[0].grid(True)

    axarr[1].plot(time_array, KalmanX, '-r', label="Error in X")
    axarr[1].set_title('Estimated error in image X-direction')
    axarr[1].set_xlabel('Time [s]')
    axarr[1].set_ylabel('Distance [cm]')
    axarr[1].grid(True)

    plt.savefig(figname)
    plt.show()


if __name__ == '__main__':
    figtitle = 'test'
    filename = 'testData1.yaml'
    figname = 'experimentData.png'
    plotData(filename, figname, figtitle)
