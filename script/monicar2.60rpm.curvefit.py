import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def calcPwm(x, a, b, ):
    return a * np.array(x) + b

xdata=[0.04, 0.08, 0.1, 0.12, 0.14, 0.16, 0.2]
ydata=[55,   100,   122,   145,   167,   190, 235]

popt, pcov = curve_fit(calcPwm, xdata, ydata)

print(popt[0], popt[1])

print("Orig :",ydata)
print("Fited:",calcPwm(xdata, *popt).astype(int))

#0.02, 0.03 calculate
xdata=[0.02, 0.03, 0.04, 0.08, 0.1, 0.12, 0.14, 0.16, 0.2]
print("Fited:",calcPwm(xdata, *popt).astype(int))

