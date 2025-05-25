import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load your CSV file
df = pd.read_csv(r"C:\Users\jerem\Documents\CansatGithub\LENA\Ground_staton\data.csv", sep="\t")

# Extract measured and reference temperature data
measured_temp = df['DHT11_Temp'].values[200:]
reference_temp = df['DS18B20_Temp'].values[200:]

print(len(measured_temp))

# Perform linear regression
p = np.polyfit(measured_temp, reference_temp, 1)
a, b = p

# Compute the fitted values
fitted = np.polyval(p, measured_temp)

# Calculate R^2
ss_res = np.sum((reference_temp - fitted) ** 2)
ss_tot = np.sum((reference_temp - np.mean(reference_temp)) ** 2)
r_squared = 1 - (ss_res / ss_tot)

# Plot
plt.figure(figsize=(8, 6))
plt.scatter(measured_temp, reference_temp, color='b', marker='o', label='Calibration Data', s=0.4)
plt.plot(measured_temp, fitted, 'r-', label='Linear Fit')
plt.xlabel('Measured Temperature (DHT11)')
plt.ylabel('Reference Temperature (DS18B20)')
plt.title('DHT11 Calibration Using DS18B20 as Reference')
plt.legend(loc='best')
plt.grid(True)

# Annotate R^2 as a text box
textstr = f'$R^2 = {r_squared:.4f}$\n$y = {a:.4f}x + {b:.4f}$'
plt.gca().text(0.05, 0.95, textstr, transform=plt.gca().transAxes,
               fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='white', edgecolor='black'))

plt.show()