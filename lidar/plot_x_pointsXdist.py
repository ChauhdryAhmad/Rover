import numpy as np
import matplotlib.pyplot as plt

# Data
x_coordinates = np.array([
    # dist = 25 
    1031,
    1043,
    1006,
    1087,
    1048,
    1050,
    1047,
    1045,
    # dist = 50
    964,
    962,
    925,
    930,
    941,
    940,
    941,
    942,
    # dist = 75
    886,
    905,
    932,
    903,
    895,
    898,
    895,
    897,
    # dist = 100
    881,
    886,
    886,
    888,
    881,
    882,
    880,
    878,
    # dist = 125 
    874,
    872,
    872,
    873,
    871,
    871,
    871,
    871
])
distances = np.array([
    # dist = 25 
    25,
    25,
    25,
    25,
    25,
    25,
    25,
    25,
    # dist = 50
    50,
    50,
    50,
    50,
    50,
    50,
    50,
    50,
    # dist = 75
    75,
    75,
    75,
    75,
    75,
    75,
    75,
    75,
    # dist = 100
    100,
    100,
    100,
    100,
    100,
    100,
    100,
    100,
    # dist = 125 
    125,
    125,
    125,
    125,
    125,
    125,
    125,
    125
])

# Fit a polynomial curve (let's start with a second-degree polynomial)
coefficients = np.polyfit(x_coordinates, distances, 3) #4th degree suits rn
polynomial = np.poly1d(coefficients)

# Generate x values for plotting the fitted curve
x_fit = np.linspace(min(x_coordinates), max(x_coordinates), 500)
y_fit = polynomial(x_fit)

# Plot the data points
plt.scatter(x_coordinates, distances, color='red', label='Data Points')

# Plot the fitted curve
plt.plot(x_fit, y_fit, color='blue', label='Fitted Curve')

# Add labels and title
plt.xlabel('X Coordinates')
plt.ylabel('Distances')
plt.title('Curve Fitting Example')
plt.legend()

# Show the plot
plt.show()

# Print the equation of the curve
print("Equation of the fitted curve:")
equation = f"d = {coefficients[0]}*x^3 + {coefficients[1]}*x^2 + {coefficients[2]}*x + {coefficients[3]}"
print(equation)
