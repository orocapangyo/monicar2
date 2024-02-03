import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def calcPwm(x, a, b, ):
    return a * np.array(x) + b

xdata=[0.027, 0.043, 0.060,  0.077,  0.095, 0.112, 0.130, 0.148]
ydata=[40,    60,    80,    100,     120,   140,   160,   180]

popt, pcov = curve_fit(calcPwm, xdata, ydata)

print(popt[0], popt[1])

print("Orig :",ydata)
print("Fited:",calcPwm(xdata, *popt).astype(int))

#Find min -> max fitted value
xdata=[0.03, 0.05, 0.07, 0.1, 0.12, 0.14, 0.16]
print("Fited min, max:",calcPwm(xdata, *popt).astype(int))
