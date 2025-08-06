import matplotlib.pyplot as plt
import numpy as np

# no-imu-no-pda, imu-no-pda, no-imu-pda, imu-pda
# Sample data
averages = [0.083, 0.0231, 0.0622, 0.0016]

# Series labels
labels = ['no-imu-no-upscaling', 'imu-no-upscaling', 'no-imu-upscaling', 'imu-upscaling']

# X locations for the groups
x = np.arange(len(labels))

# Create the bar plot for averages
plt.bar(x, averages, capsize=5, color='lightblue', label='Average')


# Add labels and title
plt.xticks(x, labels, rotation=45)
plt.ylabel('[m]')

# Show the plot
plt.tight_layout()
plt.show()
