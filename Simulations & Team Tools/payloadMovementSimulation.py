import numpy as np
import matplotlib.pyplot as plt
import math

Kp = 9
Kd = 10
forwardSpeed = 1
dT = 1
simTime = 10


def clamp(val, max, min):
    if val < min: return min
    elif val > max: return max
    return val

def wrapTheta(theta):
    """
    wrap theta to [-pi,pi]
    """
    if theta > math.pi:
       theta -= 2*math.pi
    elif theta < -math.pi:
        theta += 2*math.pi
    
    return theta

def getAngularAccel(PWM):
    """
    parameters:
    PWM(int): PWM to run fans at

    return:
    alpha(float) : angular acceleration *is this in radians/s^2?
    """
    # Constants
    x = PWM  # PWM value
    # Calculate angular acceleration based on PWM value
    if x < 0:
        alpha = 10.8 + 0.986 * x + 0.0454 * x**2 - 3.06e-05 * x**3
    else:
        alpha = 0.0378 - 2.33 * x - 0.0159 * x**2 - 2.77e-04 * x**3
    return alpha

def getThetaError(currentPos, currentTheta, goalPos):
    """
    parameters: 
    currentPos([double, double]): current x,y position 
    currentTheta(double, [-pi,pi)): current direction facing
    goalPos([double, double]): goal position

    return:
    theta(double, [-pi,pi]): angle to goal from current position
    """

    [x0,y0] = currentPos
    [xg,yg] = goalPos
    delta_x = xg-x0
    delta_y = yg-y0
    goalTheta = math.atan2(delta_y,delta_x) #[-pi,pi]

    #identify if rotation need to happen CCW, or CW
    thetaError = wrapTheta((goalTheta -currentTheta)% (2*math.pi))
    return thetaError 

def controller(currentPos, currentTheta, goalPos, currentOmega):
    thetaError = getThetaError(currentPos, currentTheta, goalPos)
    angVelError = currentOmega

    PWM = Kp*thetaError + Kd*angVelError #simple PD, may need to change given PWM to angular accel relationship is nonlinear

    PWM = clamp(PWM, 255, -255) #restict PWM to possible output
    return PWM

def testThetaError():
    """
    test theta error caluclations over various angles
    """
    # Testing and plotting theta errors
    currentPos = [1, 1]
    goalPos = [0.5, 0.5]
    angleRange = np.arange(-180, 181, 15)  # Angles in degrees
    theta_errors = []

    for angle in angleRange:
        currentTheta = math.radians(angle) # Convert degrees to radians
        thetaError = math.degrees(getThetaError(currentPos, currentTheta, goalPos))
        theta_errors.append(thetaError)

    # Convert results to a 2xn array
    theta_data = np.array([angleRange, theta_errors])

    # Print theta error with respect to angle
    for i in range(theta_data.shape[1]):
        print(f"Angle: {theta_data[0, i]} degrees, Theta Error: {theta_data[1, i]:.2f} radians")

    # Plot theta error
    plt.plot(theta_data[0], theta_data[1], marker="o")
    plt.title("Theta Error vs Angle")
    plt.xlabel("Angle (degrees)")
    plt.ylabel("Theta Error (degrees)")
    plt.xticks(angleRange)
    plt.yticks(np.arange(-180,181,45))
    plt.grid(True)
    plt.show()

def simulate(startPos, goalPos, theta0, omega0, forwardSpeed, simTime, dT):

    # Initialize arrays
    positions = np.array([[startPos[0]], [startPos[1]]])  # 2xN matrix for positions
    thetas = np.array([theta0])  # Array for theta values
    omegas = np.array([omega0])  # Array for angular velocity
    alphas = np.array([0])  # Array for angular acceleration
    PWMs = np.array([0])  # Array for PWM outputs

    for i in range(int(simTime / dT)):  # Simulate for a fixed number of steps
        # Compute controller output
        outputPWM = controller([positions[0, i], positions[1, i]], thetas[i], goalPos, omegas[i])
        PWMs = np.append(PWMs, outputPWM)

        # Calculate angular acceleration
        currentAlpha = getAngularAccel(outputPWM)
        alphas = np.append(alphas, currentAlpha)

        # Update angular velocity and angle
        currentOmega = omegas[i] + currentAlpha * dT
        omegas = np.append(omegas, currentOmega)

        currentTheta = wrapTheta(thetas[i] + currentOmega * dT)
        thetas = np.append(thetas, currentTheta)

        # Update position
        currentX = positions[0, i] + math.cos(currentTheta) * forwardSpeed * dT
        currentY = positions[1, i] + math.sin(currentTheta) * forwardSpeed * dT
        new_position = np.array([[currentX], [currentY]])  # New position as a column vector
        positions = np.hstack((positions, new_position))  # Append new position


    # Plot the trajectory
    plt.figure(figsize=(6, 6))
    plt.plot(positions[0, :], positions[1, :], marker='o', label='Path')
    plt.scatter(startPos[0], startPos[1], color='black', marker='x', s=100, label='Start Position')
    plt.scatter(goalPos[0], goalPos[1], color='red', marker='x', s=100, label='Goal Position')  # Red X for goalPos
    plt.title("Payload Position")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.grid(True)
    plt.legend()
    plt.show()

simulate([0,0], [10,0], 0, 0, forwardSpeed, simTime, dT)
