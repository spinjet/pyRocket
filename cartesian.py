import numpy as np
from scipy.integrate import odeint
from matplotlib import pyplot as plt

v0 = 100 #m/s Initial speed
th0 =80 * (np.pi / 180) #deg Initial flight path angle
h0 = 0 #m launching altitude
s0 = 0 #m launching position


Mass = 0.5 #kg Projectile Mass
g = 9.81 # m/s^2 Gravity Acceleration
Cd = 0.01 # Drag Coefficient
S = 0.5 # m^2 Frontal Area
rho = 1.225 # Air Density kg/m^3


def idealBallistic(V, Th, M, h0 = 0):
    vy0 = V * np.sin(Th)
    vx0 = V * np.cos(Th)

    tFlight = vy0 / g + np.sqrt( (2 * h0 + ( vy0 ** 2 / g) ) / g )
    hMax = h0 + (vy0 ** 2) / (2 * g)
    sMax = vx0 * ( vy0 / g + np.sqrt(( 2 * h0 + ( vy0 ** 2 / g) ) / g ))


    return (tFlight, hMax, sMax)

def plotIdealBallistic(V, Th, M, h0=0):
    T, H, S = idealBallistic(V, Th, M)

    Ndiscr = T / 0.5
    t = np.linspace(0,T,Ndiscr)
    x = V * np.cos(Th) * t
    y = h0 + V * np.sin(Th) * t - 0.5 * g * t ** 2

    plt.plot(x,y,'r')
    
def ddtV(V, Th):
    return (- Drag(rho, V) / Mass ) -  (g * np.sin(Th))

def ddtTh(V, Th):
    return -g * np.cos(Th) / V

def Drag(rho, V):
    return 0.5 * rho * S * V**2 * S * Cd

def ddtSys(Z, t):
    V, Th = Z
    return [ddtV(V, Th), ddtTh(V, Th)]

t = np.linspace(0, 50, 10000)

Z0 = [v0, th0]
Z = odeint(ddtSys, Z0, t)
v, th = Z.T

vx = v * np.cos(th)
vy = v * np.sin(th)


h = np.zeros(len(t))
h[0] = h0
for i in range(0,len(t)-1):
    h[i+1] = h[i] + vy[i] * (t[i+1] - t[i])


h = np.array(list(filter(lambda x : x >= 0, h)))
t = t[0:len(h)]


s = np.zeros(len(t))
s[0] = s0
for i in range(0,len(t)-1):
    s[i+1] = s[i] + vx[i] * (t[i+1] - t[i])


# vy = list(filter(lambda x : x >= 0, vy))
# vx = vx[0:len(vy)]
# t = t[0:len(vy)]


outText = """***TRAJECTORY RESULTS***

Input Data:

Initial Velocity            {v0:<5} m/s
Initial Flight Path Angle   {th0:<5.2f} deg
Projectile Mass             {m0:<5} kg
Drag Coefficient            {cd:<5} 
Reference Front Area        {S:<5} m^2

Output Data:

Maximum Altitude        {hmax:>7.3f} m
Travelled Distance      {sfin:>7.3f} m
Time of Flight          {tfin:>7.3f} s
""".format(
    v0=v0,
    th0=(th0*(180 / np.pi)),
    m0=Mass,
    cd=Cd,
    S=S,
    hmax=h.max(0),
    sfin=s[-1],
    tfin=t[-1]
)

print(outText)

_, hmax, smax = idealBallistic(v0, th0, Mass)

scaleL = hmax if hmax > smax else smax

fig = plt.figure(1)
ax = fig.add_subplot(1,1,1)

'''
ax.set_xticks(np.arange(0,1000,100))
ax.set_yticks(np.arange(0,1000,100))
'''
ax.grid(b=True)
ax.set_ylim([0, scaleL*1.05])
ax.set_xlim([0, scaleL*1.05])


plotIdealBallistic(v0, th0, Mass)
plt.plot(s, h)
plt.title('Trajectory\nTime of Flight {:.3f} s'.format(t[-1]))
plt.xlabel('Distance [m]'), plt.ylabel('Altitude [m]')
plt.legend(['Ideal','Real'])

plt.show()
