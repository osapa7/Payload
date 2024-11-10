import random
import math
import matplotlib.pyplot as plt
import numpy as np
import copy

# Define constants
K = 2  # Motor constant
KP = 8.96  # Proportional gain for PD controller
KD = 16  # Derivative gain for PD controller
VS = 5  # Vehicle speed
DT = 0.01
RAND = 10/DT

target = (0,5)

# Function to generate wind vector in polar form
def generate_wind(a, b, c, d, e, f, wind_mag):
  wind_mag_change = (a * random.uniform(-1, 1) + b) / DT
  wind_mag += wind_mag_change * DT
  wind_mag = max(c, min(wind_mag, d))  # Limit magnitude
  wind_angle = e + (f - e) * random.uniform(0, 1)  # Limit angle
  wind_angle = wind_angle % 360  # Wrap around to 0-360 degrees
  return wind_mag, wind_angle

# Function to calculate goal angle
def calculate_goal_angle(pos, heading, corrected):
  global target

  # distance from current to target
  dx = target[0] - pos[0][-1]
  dy = target[1] - pos[1][-1]

  return_angle = math.degrees(math.atan2(dy, dx)) % 360

  # wind correction logic
  if corrected:
    wind = np.array([0,0])
    if len(pos[0]) > 1:
      wind = np.array([pos[0][-1]-pos[0][-2], pos[1][-1]-pos[1][-2]]) - VS*DT*np.array([math.cos(math.radians(heading[-1])), math.sin(math.radians(heading[-1]))])
      wind /= DT
    
    direction = np.array([dx, dy])
    direction_norm = np.linalg.norm(direction)

    wind_todirection = direction * np.dot(wind, direction) / direction_norm**2
    wind_cross = wind - wind_todirection

    h_prime = direction/direction_norm * np.sqrt(VS**2 - np.linalg.norm(wind_cross)**2)

    goal_vec = h_prime - wind_cross

    goal_angle = math.degrees(math.atan2(goal_vec[1], goal_vec[0])) % 360

    return_angle = goal_angle

  return return_angle

# Function simulates PD controller
def system(pos, heading, past_error, corrected):
  # Calculate goal angle
  goal_angle = calculate_goal_angle(pos, heading, corrected)

  # Calculate error terms
  angle_error = goal_angle - heading[-1]
  derror = (angle_error - past_error)/DT

  # Calculate control signal
  pwm = KP * angle_error + KD * derror

  return pwm, angle_error


# Updates heading and angular velocity
def update_state(ang_vel, index, pwm):
  accel = K * pwm
  accel += RAND * random.uniform(-1, 1)
  ang_vel[index] += accel * DT

  return ang_vel[index] * DT

# updates the heading and position
def move(pos, heading, total_time, ang_vel, xvel, yvel, past_error, completed, wind_mag, wind_angle, corrected):
  global target

  # iterates through each starting point
  for i in range(len(pos)):
    if i not in completed:
      total_time[i] += 1
      # Get system output
      pwm, past_error[i] = system(pos[i], heading[i], past_error[i], corrected)

      # Update state
      heading[i].append(heading[i][-1] + update_state(ang_vel, i, pwm))

      xvel[i].append(VS * math.cos(math.radians(heading[i][-1])) + wind_mag * math.cos(math.radians(wind_angle)))
      yvel[i].append(VS * math.sin(math.radians(heading[i][-1])) + wind_mag * math.sin(math.radians(wind_angle)))
      pos[i][0].append(pos[i][0][-1] + xvel[i][-1] * DT)
      pos[i][1].append(pos[i][1][-1] + yvel[i][-1] * DT)

      if math.sqrt((pos[i][0][-1]-target[0])**2 + (pos[i][1][-1]-target[1])**2) < 0.1:
        completed.append(i)

# Simulation loop
def simulate(pos, steps, rand_mag, offset, wind_mag_lower, wind_mag_upper, ang_lower, ang_upper):
  # Initialize state
  wind_mag = 0

  # not wind-corrected state
  heading = [[0] for _ in pos]
  xvel = [[0] for _ in pos]
  yvel = [[0] for _ in pos]
  total_time = [1] * len(pos)
  ang_vel = [0] * len(pos)
  past_error = [0] * len(pos)
  completed = []

  # wind-corrected state
  posw = copy.deepcopy(pos)
  headingw = copy.deepcopy(heading)
  xvelw = copy.deepcopy(xvel)
  yvelw = copy.deepcopy(yvel)
  total_timew = [1] * len(pos)
  ang_velw = [0] * len(pos)
  past_errorw = [0] * len(pos)
  completedw = []
      
  # Simulation loop
  for _ in range(int(steps)):
    # Generate wind
    wind_mag, wind_angle = generate_wind(rand_mag, offset, wind_mag_lower, wind_mag_upper, ang_lower, ang_upper, wind_mag)
    
    # the non-wind-corrected simulation
    move(pos, heading, total_time, ang_vel, xvel, yvel, past_error, completed, wind_mag, wind_angle, False)

    # wind-corrected simulation
    move(posw, headingw, total_timew, ang_velw, xvelw, yvelw, past_errorw, completedw, wind_mag, wind_angle, True)
      
  # displays all graphs for each starting point
  for i in range(len(pos)):
    plt.figure()
    plt.title(f"Position of Starting Point ({pos[i][0][0]}, {pos[i][1][0]})")
    plt.plot(pos[i][0], pos[i][1], ls='-', color='pink', label='non-corrected')
    plt.plot(posw[i][0], posw[i][1], ls='-', color='red', label='wind corrected')
    plt.legend()

    plt.figure()
    plt.subplot(2, 1, 1) 
    plt.title("X Velocity")
    plt.plot(np.arange(0, (total_time[i])*DT, DT), xvel[i], ls='-', color='orange', label='non-corrected')
    plt.plot(np.arange(0, (total_timew[i])*DT, DT), xvelw[i], ls='-', color='brown', label='wind corrected')
    plt.legend()

    plt.subplot(2, 1, 2) 
    plt.title("Y Velocity")
    plt.plot(np.arange(0, (total_time[i])*DT, DT), yvel[i], ls='-', color='olive', label='non-corrected')
    plt.plot(np.arange(0, (total_timew[i])*DT, DT), yvelw[i], ls='-', color='green', label='wind corrected')
    plt.tight_layout()
    plt.legend()

    plt.figure()
    plt.title(f"Heading of Starting Point ({pos[i][0][0]}, {pos[i][1][0]})")
    plt.plot(np.arange(0,(total_time[i])*DT,DT), heading[i], ls = '-', color='cyan', label='non-corrected')
    plt.plot(np.arange(0,(total_timew[i])*DT,DT), headingw[i], ls = '-', color='blue', label='wind corrected')
    plt.legend()
    plt.show()


# wind variables
rand_mag = 0.1
offset = 0
w_min = 0
w_max = 3
angle_min = 90
angle_max = 180

duration = 10

# possible starting positions
pos = [([-3],[-10]), ([0],[0]), ([6],[6])]

simulate(pos, duration/DT, rand_mag, offset, w_min, w_max, angle_min, angle_max)
