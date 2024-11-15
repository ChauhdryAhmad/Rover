import numpy as np
import matplotlib.pyplot as plt

# Define the function based on the fitted equation
def fitted_curve(x):
    return -2.449667700795079e-05 * x**3 + 0.07483786812947929 * x**2 - 76.28786215573622 * x + 25978.577127861463

# Generate x values (for example, from 0 to 500, or adjust range as needed)
x_values = np.linspace(0, 1000, 5)  # 1000 points for a smooth curve

# Calculate y values based on the fitted curve function
y_values = fitted_curve(x_values)

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(x_values, y_values, label='Fitted Curve', color='blue')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Plot of Fitted Curve')
plt.legend()
plt.grid(True)
plt.show()

