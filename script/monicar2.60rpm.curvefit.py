import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def calcPwm(x, a, b, ):
    return a * np.array(x) + b

xldata=[0.027, 0.043, 0.060,  0.077,  0.095, 0.112, 0.130, 0.148]
xrdata=[0.027, 0.043, 0.060,  0.077,  0.095, 0.112, 0.130, 0.148]
ydata=[40,    60,    80,    100,     120,   140,   160,   180]

#Fit with Vright
popt, pcov = curve_fit(calcPwm, xrdata, ydata)

print(popt[0], popt[1])
#print("Orig :",ydata)
#rint("Fitted:",calcPwm(xrdata, *popt).astype(int))

#Find min -> max fitted value
xdata=[0.03, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16]
pwmR = calcPwm(xdata, *popt).astype(int)
print("Fited min, max:", pwmR)

#Fit with Vleft
popt, pcov = curve_fit(calcPwm, xldata, ydata)
pwmL = calcPwm(xdata, *popt).astype(int)
print("Fited min, max:", pwmL)

print("Kbias:", pwmL - pwmR)

