import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def calcPwm(x, a, b, ):
    return a * np.array(x) + b

xdata=[0.04, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16, 0.18]
ydata=[40,    60,    80,  95, 117,  135,  150,  170]

popt, pcov = curve_fit(calcPwm, xdata, ydata)

print(popt[0], popt[1])

print("Orig :",ydata)
print("Fitted:",calcPwm(xdata, *popt).astype(int))

#if want to check another velocity
xdata=[0.03, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16]
print("Min,Max:",calcPwm(xdata, *popt).astype(int))
