import numpy as np
import matplotlib.pyplot as plt

# Data
y_coordinates = np.array([
    # dist = 25 
    199,
    198,
    196,
    198,
    207,
    215,
    211,
    215,
    # dist = 50
    351,
    351,
    350,
    350,
    361,
    356,
    359,
    358,
    # dist = 75
    411,
    411,
    411,
    410,
    415,
    415,
    417,
    415,
    # dist = 100
    429,
    429,
    430,
    429,
    429,
    430,
    431,
    431,
    # dist = 125 
    444,
    443,
    443,
    444,
    449,
    446,
    447,
    448
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
coefficients = np.polyfit(y_coordinates, distances, 3) #4th degree suits rn
polynomial = np.poly1d(coefficients)

# Generate x values for plotting the fitted curve
x_fit = np.linspace(min(y_coordinates), max(y_coordinates), 500)
y_fit = polynomial(x_fit)

# Plot the data points
plt.scatter(y_coordinates, distances, color='red', label='Data Points')

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
equation = f"d = {coefficients[0]}*y^3 + {coefficients[1]}*y^2 + {coefficients[2]}*y + {coefficients[3]}"
print(equation)
