'''
Equation of the fitted curve acc to x coordinates :
y = -2.449667700795079e-05*x^3 + 0.07483786812947929*x^2 + -76.28786215573622*x + 25978.577127861463


Equation of the fitted curve acc to y coordinates :
y = 2.8138839924614784e-05*x^3 + -0.025410657787418665*x^2 + 7.595435158317794*x + -705.4154291595778

'''

import numpy as np

def predictDistanceAccTo_x_Cordinates(x):
    return (-2.449667700795079e-05 * x**3 
            + 0.07483786812947929 * x**2 
            - 76.28786215573622 * x 
            + 25978.577127861463)

def predictDistanceAccTo_y_Cordinates(x):
    return (2.8138839924614784e-05 * x**3 
            - 0.025410657787418665 * x**2 
            + 7.595435158317794 * x 
            - 705.4154291595778)

x_values = np.array([
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
y_values = np.array([
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
x_d_values = predictDistanceAccTo_x_Cordinates(x_values)
y_d_values = predictDistanceAccTo_y_Cordinates(y_values)

print("\nComputed d values according to x :")
print(x_d_values)


print("\nComputed d values according to y :")
print(y_d_values)

