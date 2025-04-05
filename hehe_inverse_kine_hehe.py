import numpy as np
import matplotlib.pyplot as plt

# Initialize variables
# x and y are inputs representing the position of Wall-E's hands relative to the base of actuator 2 (in inches)
# x = np.linspace(20, 40, 100)  # x-position input (in inches)
t = np.linspace(0,10,1000)
x = 10 * np.sin(2 * np.pi * t / 5)  # x-position input (in inches)
y = -15  # y-position input (in inches)

# The program should find L1 and L2 to satisfy these requirements
r = 10  # r is the distance from the base of actuator 1 to the base of actuator 2 (in inches)
x0 = r * np.cos(np.deg2rad(80))  # x0 is the horizontal component of r, assuming its angle is 80 degrees (in inches)
y0 = r * np.sin(np.deg2rad(80))  # y0 is the vertical component of r, assuming its angle is 80 degrees (in inches)
L2 = 2  # L2 is the length of the link that connects the end of actuator 1 and the base of actuator 2 (in inches)

L3 = np.sqrt(x**2 + y**2)  # L3 is the distance from the base of actuator 2 to Wall-E's hands, or the end of actuator 2 (in inches)
theta3 = np.rad2deg(np.arctan(y / x))  # theta3 is the angle of actuator 2 relative to horizontal. Returns between -90 degrees and 90 degrees
theta2 = 200 - theta3  # theta2 is the angle of the link connecting actuator 1 and 2

L1 = np.sqrt(r**2 - 2 * L2 * (x0 * np.cos(np.deg2rad(theta2)) + y0 * np.sin(np.deg2rad(theta2))) + L2**2)  # L1 is the length of actuator 1 (in inches)

plt.figure()
plt.plot(t,L3, label = 'length of shoulder actuator')
plt.plot(t, L1, label = 'length of forearm actuator')
plt.xlabel('time')
plt.ylabel('length')
plt.legend()
plt.show()
'''
# Print results for verification (optional)
print(f"x: {x}")
print(f"y: {y}")
print(f"x0: {x0}")
print(f"y0: {y0}")
print(f"L2: {L2}")
print(f"L3: {L3}")
print(f"theta3: {theta3}")
print(f"theta2: {theta2}")
print(f"L1: {L1}")
'''
