# import pandas as pd
# import matplotlib.pyplot as plt

# # Load CSV
# df = pd.read_csv("ground_truth_curve_2_4.csv")

# # Plot XY
# plt.figure(figsize=(8, 6))
# plt.plot(df['x'], df['y'], marker='o', markersize=2)
# plt.xlabel('X [m]')
# plt.ylabel('Y [m]')
# plt.title('2D Trajectory from TF Logger')
# plt.grid(True)
# plt.axis('equal')
# plt.show()



import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv("speed_linear_1.6.csv")

# Convert Series to NumPy arrays
x = df['x'].to_numpy()
y = df['y'].to_numpy()

# Plot XY
plt.figure(figsize=(8, 6))
plt.plot(x, y, marker='o', markersize=2)
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('2D Trajectory from TF Logger')
plt.grid(True)
plt.axis('equal')
plt.show()
