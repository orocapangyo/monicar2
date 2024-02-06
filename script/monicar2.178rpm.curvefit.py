import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def calcPwm(x, a, b, ):
    return a * np.array(x) + b

xldata=[0.062, 0.085,   0.111,  0.135, 0.160, 0.185]
xrdata=[0.058, 0.081,   0.104,  0.129, 0.151, 0.173]
ydata= [30,     40,     50,     60,     70,   80]

#Fit with Vright
popt, pcov = curve_fit(calcPwm, xrdata, ydata)

print(popt[0], popt[1])
#print("Orig :",ydata)
#rint("Fitted:",calcPwm(xrdata, *popt).astype(int))

#Find min -> max fitted value
xdata=[0.04, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16, 0.18]
pwmR = calcPwm(xdata, *popt).astype(int)
print("Fited min, max:", pwmR)

#Fit with Vleft
popt, pcov = curve_fit(calcPwm, xldata, ydata)
pwmL = calcPwm(xdata, *popt).astype(int)
print("Fited min, max:", pwmL)

print("Kbias:", pwmL - pwmR)
