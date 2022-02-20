import math
from matplotlib import pyplot as plt
m = 342.0
J = 1.13
r = 0.33
g = 9.81
v0 = 27.78
Tbmax = 1200.0
#dry asphalt
c1 = 1.2801
c2 = 23.99
c3 = 0.52
c4 = 0.03
Tp = 0.01
Ts = 10.0
S = [v0*Tp]
slip = [0.0]
V = [v0]
omega = [v0/r]

# withoutABS funcion simulating vehicle braking without ABS system
# recives as follow:
# m - mass of vehicle
# J -
# r - radius of wheel
# g - gravitational acceleration
# v0 - initial velocity of vehicle
# c1,c2,c3,c4 - parameters of road
# Tp - czas próbkowania
# Ts - time of simulation
# With controler system there are also:
# Kp - proportional gain
# Ki - integral gain
# Kd - derivative gain
# FUNCTION RETURNS LIST OF ELEMENTS SUCH AS:
# [0] - velocity of vehicle over time (list)
# [1] - breaking distance over time (list)
# [2] - breaking time (float)
# [3] - wheel slip over time (list)
# [4] - wheel anugular speed over time (list)
def withoutABS (m, J, r, g, v0, Tbmax, c1, c2, c3, c4, Tp, Ts):
    S = [v0 * Tp]
    slip = [0]
    V = [v0]
    omega = [v0 / r]
    Tb = Tbmax
    for x in range(0, round(Ts / Tp)):
        # if Tb < Tbmax:
        #     Tb += 1
        if V[x] > 0.01:
            mu = (c1 * (1 - math.exp(-c2 * slip[x]) - c3 * slip[x]) * math.exp(-c4 * V[x] * slip[x]))
            dot_omega = ((mu * r * m * g) - Tb) / J
            newomega = omega[x] + (dot_omega * Tp)
            if newomega < 0:
                newomega = 0
            omega.append(newomega)
            ax = (-mu * m * g) / m
            newv = V[x] + (ax * Tp)
            V.append(newv)
            S.append(S[x] + (newv * Tp))
            dslip = (((-mu * m * g) / newv) * (((1 - slip[x]) / m) + ((r * r) / J))) + ((r * Tb) / (J * newv))
            newslip = slip[x] + (dslip * Tp)
            if newslip > 1:
                newslip = 1
            if newslip < 0:
                newslip = 0

            slip.append(newslip)
            #print(omega[x], x * Tp)
        else:
            slip.pop()
            V.pop()
            S.pop()
            omega.pop()
            break
    return V, S, (x-1)*Tp, slip, omega


def withPtypefeedback(m, J, r, g, v0, Tbmax, c1, c2, c3, c4, Tp, Ts):
    S = [v0 * Tp]
    slip = [0]
    V = [v0]
    omega = [v0 / r]
    desired = 0.2
    Kp = 250
    newTb = 0
    preve = 0
    newe = (desired - slip[0])
    for x in range(0, round(Ts / Tp)):
        if V[x] > 0.5:
            preve = newe
            newe = (desired - slip[x])
            newTb = newTb +  (Kp * (desired - slip[x]))
            if newTb > Tbmax:
                newTb = Tbmax
            if newTb < 0:
                newTb = 0
            mu = (c1 * (1 - math.exp(-c2 * slip[x]) - c3 * slip[x])) * math.exp(-c4 * V[x] * slip[x])
            dot_omega = ((mu * r * m * g) - newTb) / J
            newomega = omega[x] + (dot_omega * Tp)
            if newomega < 0:
                newomega = 0
            omega.append(newomega)
            ax = (-mu * m * g) / m
            newv = V[x] + (ax * Tp)
            V.append(newv)
            S.append(S[x] + (newv * Tp))
            dslip = (((-mu * m * g) / newv) * (((1 - slip[x]) / m) + ((r * r) / J))) + ((r * newTb) / (J * newv))
            newslip = slip[x] + (dslip * Tp)
            if newslip > 1:
                newslip = 1
            if newslip < 0:
                newslip = 0
            slip.append(newslip)
            # print(omega[x], x * Tp)

            #print(slip[x])
        else:
            slip.pop()
            V.pop()
            S.pop()
            omega.pop()
            break
    return V, S, x * Tp, slip, omega


def withPIDtypefeedback(m, J, r, g, v0, Tbmax, c1, c2, c3, c4, Tp, Ts, Kp, Ki, Kd):
    S = [v0 * Tp]
    slip = [0]
    V = [v0]
    omega = [v0 / r]
    desired = 0.2
    newTb = 0
    e = [desired - slip[0]]
    for x in range(0, round(Ts / Tp)):
        if V[x] > 0.5:
            e.append((desired - slip[x]))
            newTb += (Kp * (desired - slip[x])) + (Ki * (sum(e) * (Tp*x))) + (Kd * (e[x+1] - e[x]))
            if newTb > Tbmax:
                newTb = Tbmax
            if newTb < 0:
                newTb = 0
            mu = (c1 * (1 - math.exp(-c2 * slip[x]) - c3 * slip[x])) * math.exp(-c4 * V[x] * slip[x])
            dot_omega = ((mu * r * m * g) - newTb) / J
            newomega = omega[x] + (dot_omega * Tp)
            if newomega < 0:
                newomega = 0
            omega.append(newomega)
            ax = (-mu * m * g) / m
            newv = V[x] + (ax * Tp)
            V.append(newv)
            S.append(S[x] + (newv * Tp))
            dslip = (((-mu * m * g) / newv) * (((1 - slip[x]) / m) + ((r * r) / J))) + ((r * newTb) / (J * newv))
            newslip = slip[x] + (dslip * Tp)
            if newslip > 1:
                newslip = 1
            if newslip < 0:
                newslip = 0
            slip.append(newslip)
            # print(omega[x], x * Tp)

            #print(slip[x])
        else:
            slip.pop()
            V.pop()
            S.pop()
            omega.pop()
            break
    return V, S, x * Tp, slip, omega
# for x in range(0, round(Ts/Tp)):
#     if V[x] > 0.5:
#         mu = (c1 * (1 - math.exp(-c2 * slip[x]) - c3 * slip[x])) * math.exp(-c4 * V[x] * slip[x])
#         dot_omega = ((mu * r * m * g) - Tbmax) / J
#         newomega = omega[x] + (dot_omega * Tp)
#         if newomega < 0:
#             newomega = 0
#         omega.append(newomega)
#         ax = (-mu * m * g)/m
#         newv = V[x] + (ax * Tp)
#         V.append(newv)
#         S.append(S[x] + (newv * Tp))
#         dslip = (((-mu * m * g)/ newv) * (((1-slip[x])/m) + ((r*r)/J))) + ((r * Tbmax)/(J * newv))
#         newslip = slip[x] + (dslip * Tp)
#         if newslip > 1:
#             newslip = 1
# 
#         slip.append(newslip)
#         print(omega[x], x * Tp)
#     else:
#         break

x = 0.0
# for item in withoutABS(350, 1, 0.1, 9.81, 11, 450, 1.1973, 25.168, 0.5373, 0.03, 0.01, 5)[0]:
#     print(item, x)
#     x += 0.01
#print(withoutABS(1000, 1.13, 0.33, 9.81, 27.78, 1200, 1.2801, 23.99, 0.52, 0.03, 0.01, 10)[2])
#
# z = [mab*Tp for mab in range(0, round(x*100))]
# plt.plot(z, withoutABS(350, 1, 0.1, 9.81, 11, 450, 1.1973, 25.168, 0.5373, 0.03, 0.01, 5)[3])
# plt.show()
for item in withPIDtypefeedback(342, 1.13, 0.33, 9.81, 27.78, 1200, 1.2801, 23.99, 0.52, 0.03, 0.01, 10, 250, 5, 10)[0]:
   #print(item)
   x+= 0.01
#print(withPIDtypefeedback(1000, 1.13, 0.33, 9.81, 27.78, 1200, 1.2801, 23.99, 0.52, 0.03, 0.01, 10, 250, 5, 10)[2])
for item in withoutABS(1000, 1.13, 0.33, 9.81, 50, 1200, 1.2801, 23.99, 0.52, 0.03, 0.01, 100)[0]:
    print(item)
#print(x for x in withoutABS(10000, 1.13, 0.33, 9.81, 50, 1200, 1.2801, 23.99, 0.52, 0.03, 0.1, 1000)[0])
z = list(x/100 for x in range(0,len(withoutABS(10000, 1.13, 0.33, 9.81, 50, 1200, 1.2801, 23.99, 0.52, 0.03, 1, 1000)[0])))
plt.plot(z, withoutABS(10000, 1.13, 0.33, 9.81, 50, 1200, 1.2801, 23.99, 0.52, 0.03, 1, 1000)[0])
#plt.plot(z, withPIDtypefeedback(1000, 1.13, 0.33, 9.81, 27.78, 1200, 1.2801, 23.99, 0.52, 0.03, 0.01, 10, 250, 5, 10)[1])
plt.show()
#plt.show()
