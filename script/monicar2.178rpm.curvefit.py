import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def calcPwm(x, a, b, ):
    return a * np.array(x) + b

xdata=[0.064, 0.088,   0.114,  0.118, 0.138, 0.166]
ydata= [30,     40,     50,   60,     70,   80]

popt, pcov = curve_fit(calcPwm, xdata, ydata)

print(popt[0], popt[1])

print("Orig :",ydata)
print("Fitted:",calcPwm(xdata, *popt).astype(int))

#Find min -> max fitted value
xdata=[0.04, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16, 0.18]
print("Fited min, max:",calcPwm(xdata, *popt).astype(int))
