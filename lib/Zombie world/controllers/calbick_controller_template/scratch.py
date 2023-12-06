import matplotlib.pyplot as plt
import numpy as np

#%%
# Enable interactive mode
plt.ion()

# Create the initial plot
fig, ax = plt.subplots()
ax.set_xlim(-10, 10)  # Adjust these limits based on your expected data range
ax.set_ylim(-10, 10)
ax.set_xlabel('X Coordinate')
ax.set_ylabel('Y Coordinate')

# Constants
total_degrees = 360
total_indices = 512
deg_per_index = total_degrees / total_indices

# Degree range (60 degrees span)
start_deg = 0
end_deg   = 360

deg60 = round(60 // deg_per_index)

# Convert degrees to indices
start_index = round(start_deg / deg_per_index)
end_index = round(end_deg / deg_per_index)

allx = []
ally = []

for i in range(len(lidar_values)):
    if lidar_values[i] != float('inf') and lidar_values[i] != 0.0:
        theta, gps_xy = map_lidar(map, i, lidar_values[i])
        allx.extend([x, gps_xy[0], np.nan])
        ally.extend([y, gps_xy[1], np.nan])



# Clear previous lines
ax.clear()
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_xlabel('X Coordinate')
ax.set_ylabel('Y Coordinate')

imrange = range(start, end)

start  = 0
end    = 43
imrange = range(start, end)

# Plotting logic
xsv = []
ysv = []
for i in imrange:
    if lidar_values[i] != float('inf') and lidar_values[i] != 0.0:
        theta, gps_xy = map_lidar(map, i, lidar_values[i])
        xsv.extend([x, gps_xy[0], np.nan])
        ysv.extend([y, gps_xy[1], np.nan])

# Clear previous lines
ax.clear()
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_xlabel('X Coordinate')
ax.set_ylabel('Y Coordinate')

imrange = range(start, end)

start  = 512-43
end    = 513
imrange = range(start, end)

# Plotting logic
xsv = []
ysv = []
for i in imrange:
    if lidar_values[i] != float('inf') and lidar_values[i] != 0.0:
        theta, gps_xy = map_lidar(map, i, lidar_values[i])
        xsv.extend([x, gps_xy[0], np.nan])
        ysv.extend([y, gps_xy[1], np.nan])

ax.plot(allx, ally, color='green')
ax.plot(xsv, ysv, color='blue')
ax.set_title(f'Index: {i} (Degree: {i*deg_per_index})')

# Update the plot
fig.canvas.draw()
fig.canvas.flush_events()
plt.pause(0.1)  # Pause to update the plot

plt.ioff()  # Turn off interactive mode
plt.show()

