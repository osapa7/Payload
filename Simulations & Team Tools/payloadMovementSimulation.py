import numpy as np
import matplotlib.pyplot as plt
import math
import csv

Kp = 5
Kd = 1
forwardSpeed = 1
dT = 1
simTime = 10
goalNearness = -1 # how much of original distance from goal need to be reduced for sim to break(enter negative value to remove this clause)


def clamp(val, max, min):
    if val < min: return min
    elif val > max: return max
    return val

def wrapTheta(theta):
    """
    wrap theta to [-pi,pi]
    """
    theta = theta % (2*math.pi)
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
    x = -PWM  # PWM value
    # Calculate angular acceleration based on PWM value
    if x < 0:
        alpha = 10.8 + 0.986 * x + 0.0454 * x**2 - 3.06e-05 * x**3
    #    alpha = math.pi/4 #test line
    elif x == 0:
        alpha = 0
    else:
        alpha = 0.0378 - 2.33 * x - 0.0159 * x**2 - 2.77e-04 * x**3
    #    alpha = -math.pi/4 #test line
    return alpha*0.05

def plotAngularAccel():
    # Generate PWM values from -255 to 255
    PWM_values = np.arange(-255, 256)
    # Compute angular acceleration for each PWM value
    angular_accel_values = [getAngularAccel(PWM) for PWM in PWM_values]

    # Plot the results
    plt.figure(figsize=(10, 6))
    plt.plot(PWM_values, angular_accel_values, label='Angular Acceleration')
    plt.axhline(0, color='black', linewidth=0.8, linestyle='--', label='Zero Acceleration')
    plt.title('Angular Acceleration vs PWM', fontsize=16)
    plt.xlabel('PWM', fontsize=14)
    plt.ylabel('Angular Acceleration (rad/s²)', fontsize=14)
    plt.grid(True)
    plt.legend(fontsize=12)
    plt.show()

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
    if abs(thetaError) < math.radians(45):
        return 0, thetaError
    angVelError = currentOmega

    PWM = Kp*thetaError + Kd*angVelError #simple PD, may need to change given PWM to angular accel relationship is nonlinear

    PWM = clamp(PWM, 100, -100) #restict PWM to possible output
    return PWM, thetaError


def distanceFromGoal(posA,posB):
    dist = math.sqrt((posA[0] - posB[0])**2 +(posA[1] - posB[1])**2)
    return dist

def simulate(startPos, goalPos, theta0, omega0, forwardSpeed, simTime, dT):

    # Initialize arrays
    positions = np.array([[startPos[0]], [startPos[1]]])  # 2xN matrix for positions
    thetas = np.array([theta0])  # Array for theta values
    omegas = np.array([omega0])  # Array for angular velocity
    alphas = np.array([0])  # Array for angular acceleration
    PWMs = np.array([0])  # Array for PWM outputs
    thetaErrors = np.array([0])
    distFromGoal0 = distanceFromGoal(startPos,goalPos)
    distFromGoals = np.array([distFromGoal0])
    print(f"Step 0:")
    print(f"  X Position: {startPos[0]:.4f}, Y Position: {startPos[1]:.4f}")
    print(f"  Theta: {theta0:.4f} rad ({math.degrees(theta0):.2f} deg)")
    print(f"  Omega: {omega0:.4f} rad/s, Alpha: 0 rad/s²")
    print(f"  Theta Error: {getThetaError(startPos, theta0, goalPos):.4f} rad ({math.degrees(getThetaError(startPos, theta0, goalPos)):.2f} deg)")
    print(f"  PWM Output: {0:.2f}")
    print(f"  Distance from Goal: {distFromGoal0:.4f}")
    print("-" * 40)

    for i in range(int(simTime / dT)):  # Simulate for a fixed number of steps
        # Compute controller output
        outputPWM, thetaError = controller([positions[0, i], positions[1, i]], thetas[i], goalPos, omegas[i])
        PWMs = np.append(PWMs, outputPWM)
        thetaErrors = np.append(thetaErrors, thetaError)

        # Calculate angular acceleration
        currentAlpha = getAngularAccel(outputPWM)
        alphas = np.append(alphas, currentAlpha)

        # Update angular velocity and angle
        currentOmega = omegas[i] + currentAlpha * dT
        omegas = np.append(omegas, currentOmega)

        currentTheta = wrapTheta(thetas[i] + currentOmega * dT)
        # if currentAlpha != 0:
        #     currentTheta = wrapTheta(thetas[i] + math.pi*0.25*currentAlpha/abs(currentAlpha)) #test line, removing momentum
        # else: currentTheta = thetas[i]
        thetas = np.append(thetas, currentTheta)

        # Update position
        currentX = positions[0, i] + math.cos(currentTheta) * forwardSpeed * dT
        currentY = positions[1, i] + math.sin(currentTheta) * forwardSpeed * dT
        new_position = np.array([[currentX], [currentY]])  # New position as a column vector
        positions = np.hstack((positions, new_position))  # Append new position
        distFromGoal = distanceFromGoal([currentX,currentY], goalPos)
        distFromGoals = np.append(distFromGoals, distFromGoal)
        if distFromGoal < (distFromGoal0*goalNearness): break
        print(f"Step {i + 1}:")
        print(f"  X Position: {currentX:.4f}, Y Position: {currentY:.4f}")
        print(f"  Theta: {currentTheta:.4f} rad ({math.degrees(currentTheta):.2f} deg)")
        print(f"  Omega: {currentOmega:.4f} rad/s, Alpha: {currentAlpha:.4f} rad/s²")
        print(f"  Theta Error: {thetaError:.4f} rad ({math.degrees(thetaError):.2f} deg)")
        print(f"  PWM Output: {outputPWM:.5f}")
        print(f"  Distance from Goal: {distFromGoal:.4f}")
        print("-" * 40)
    thetaErrors = np.delete(thetaErrors, 0) #remove thetaError from initalization as 0
    PWMs = np.delete(PWMs, 0) #remove thetaError from initalization as 0
    
    # Plot the trajectory
    # Plot trajectory with arrows representing theta
    plt.figure(figsize=(6, 6))
    plt.plot(positions[0, :], positions[1, :], marker='o', label='Path', color="blue")
    plt.scatter(startPos[0], startPos[1], color='black', marker='x', s=100, label='Start Position')
    plt.scatter(goalPos[0], goalPos[1], color='red', marker='x', s=100, label='Goal Position')

    # Add arrows to indicate orientation
    arrow_scale = 0.2  # Scale for the arrow length
    for i in range(0, positions.shape[1], max(1, int(len(thetas) / 20))):  # Reduce arrow count for clarity
        dx = arrow_scale * math.cos(thetas[i])  # X component of arrow
        dy = arrow_scale * math.sin(thetas[i])  # Y component of arrow
        plt.arrow(positions[0, i], positions[1, i], dx, dy,
                head_width=0.05, head_length=0.1, fc='green', ec='green')

    # Plot settings
    plt.title("Payload Position with Orientation")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.grid(True)
    plt.legend()
    plt.axis('equal')  # Ensure equal scaling for x and y axes
    plt.show()

    fig, axs = plt.subplots(3, 1, figsize=(8, 12))  # Create subplots with 3 rows and 1 column
    axs[0].plot(np.degrees(thetaErrors), label="Theta Error", color="blue")
    axs[0].set_title("Theta Error vs Time")
    axs[0].set_xlabel("Time")
    axs[0].set_ylabel("Theta Error (deg)")
    axs[0].grid(True)

    axs[1].plot(np.degrees(thetas), label="Theta", color="green")
    axs[1].set_title("Theta vs Time")
    axs[1].set_xlabel("Time")
    axs[1].set_ylabel("Theta (deg)")
    axs[1].grid(True)

    axs[2].plot(PWMs, label="PWM Output", color="red")
    axs[2].set_title("PWM Output vs Time")
    axs[2].set_xlabel("Time")
    axs[2].set_ylabel("PWM")
    axs[2].grid(True)

    # Adjust spacing between plots
    plt.tight_layout()

    # Show the figure
    plt.show()

    import csv

    # Write simulation data to a CSV file
    # Write simulation data to a CSV file
    csv_filename = "simulation_results.csv"
    with open(csv_filename, mode="w", newline="") as csvfile:
        csvwriter = csv.writer(csvfile)

        # Write header
        csvwriter.writerow([
            "Time Step", "X Position", "Y Position", "Theta (rad)", "Theta (deg)", 
            "Omega (rad/s)", "Alpha (rad/s²)", "Theta Error (rad)", "Theta Error (deg)", 
            "PWM Output", "Distance from Goal"
        ])

        # Write data
        for i in range(len(thetas)):
            csvwriter.writerow([
                i * dT,                   # Time step
                positions[0, i],          # X position
                positions[1, i],          # Y position
                thetas[i],                # Theta in radians
                math.degrees(thetas[i]),  # Theta in degrees
                omegas[i],                # Angular velocity
                alphas[i],                # Angular acceleration
                thetaErrors[i] if i < len(thetaErrors) else 0,  # Theta error in radians
                math.degrees(thetaErrors[i]) if i < len(thetaErrors) else 0,  # Theta error in degrees
                PWMs[i] if i < len(PWMs) else 0,                  # PWM output
                distFromGoals[i]          # Distance from goal
            ])

    print(f"Simulation results written to {csv_filename}")



simulate([0,0], [5,5], 0, 0, forwardSpeed, simTime, dT)
