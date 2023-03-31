from scipy.optimize import minimize, NonlinearConstraint, Bounds
import math
import numpy as np
import matplotlib.pyplot as plt

solutions = [
    [[0, 0], [0, 0], [0, 0], [0, 0]],
    [[0, 0], [0, 0], [0, 0], [0, 0]],
    [[0, 0], [0, 0], [0, 0], [0, 0]]
]

lengths = [0, 0, 0]

requirements = {
    "positions": [[0.75, 0.1], [0.5, 0.5], [0.2, 0.6]],
    "angles": [-math.pi/3, 0, math.pi/4]
}

def totalTorque(m, p):
    return np.dot(m, p)

def findMass(p1, p2, a):
    return findLength(p1, p2)*a, (p2[0]+p1[0])/2

def findLength(p1, p2):
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

def findAngle(p1, p2):
    return math.atan2(p2[1]-p1[1], p2[0]-p1[0])

def findPoint(l, a, ref):
    return [l*math.cos(a) + ref[0], l*math.sin(a) + ref[1]]

def calcPoints(x, i):
    p3 = requirements['positions'][i]
    p2 = findPoint(x[2], requirements['angles'][i] + math.pi, p3)
    p1a, p1b = get_intersections(0, 0, x[0], p2[0], p2[1], x[1])
    
    if (p1a[0] > p1b[0]):
        p1 = p1b
    else:
        p1 = p1a

    return p1, p2, p3

def get_intersections(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    d=math.sqrt((x1-x0)**2 + (y1-y0)**2)
    
    # non intersecting
    if d > r0 + r1 :
        raise Exception("Lengths are not long enough")
    # One circle within other
    if d < abs(r0-r1):
        print(d, r1, r0)
        raise Exception("Difference between lengths is greater than distance")
    # coincident circles
    if d == 0 and r0 == r1:
        raise Exception("Circles are coincident")
    else:
        a=(r0**2-r1**2+d**2)/(2*d)
        h=math.sqrt(r0**2-a**2)
        x2=x0+a*(x1-x0)/d   
        y2=y0+a*(y1-y0)/d   
        x3=x2+h*(y1-y0)/d     
        y3=y2-h*(x1-x0)/d 

        x4=x2-h*(y1-y0)/d
        y4=y2+h*(x1-x0)/d
        
        return [x3, y3], [x4, y4]

def objective(x):
    p1 = [x[0], x[1]]
    p2 = [x[2], 0.1-(0.75-x[2])*math.tan(-math.pi/3)]
    p3 = [0.75, 0.1]

    lengths[0] = findLength([0, 0], p1)
    lengths[1] = findLength(p1, p2)
    lengths[2] = findLength(p2, p3)

    # calculate mass and mass pos
    m1, mp1 = findMass([0, 0], p1, 4)

    m2, mp2 = findMass(p1, p2, 2)

    m3, mp3 = findMass(p2, p3, 1)

    m4, mp4 = 5, p3[0]

    m = [m1, m2, m3, m4]
    np.multiply(m, 9.81)
    p = [mp1, mp2, mp3, mp4]

    return pow((totalTorque(m, p)), 2)

def optimize_scenarios(x):
    torques = [0, 0, 0]

    x = np.abs(x)

    for i in range(len(requirements['positions'])):
        p3 = requirements['positions'][i]
        p2 = findPoint(x[2], requirements['angles'][i] + math.pi, p3)
        p1a, p1b = get_intersections(0, 0, x[0], p2[0], p2[1], x[1])
        
        if (p1a[0] > p1b[0]):
            p1 = p1b
        else:
            p1 = p1a

        # calculate mass and mass pos
        m1, mp1 = findMass([0, 0], p1, 4)

        m2, mp2 = findMass(p1, p2, 2)

        m3, mp3 = findMass(p2, p3, 1)

        m4, mp4 = 5, p3[0]

        m = [m1, m2, m3, m4]
        np.multiply(m, 9.81)
        p = [mp1, mp2, mp3, mp4]

        torques[i] = abs(totalTorque(m, p))

    return np.linalg.norm(torques)

def constrainMinLength(x):
    return x[0]+x[1]-findLength([0, 0], findPoint(x[2], requirements['angles'][0] + math.pi, requirements['positions'][0]))

def constrainMaxDiff(x):
    return x[1]-x[0]

conMaxDiff = NonlinearConstraint(constrainMaxDiff, lb=-0, ub=0, keep_feasible=True)

bounds = Bounds([0, 0, 0], [np.inf, np.inf, 0.3])

def main():
    x0 = [-0.5, 0.5, 0.5]

    res = minimize(objective, x0=x0, method='Nelder-Mead')

    print(res)

    x = [0, res.x[0], res.x[2], 0.75]
    y = [0, res.x[1], 0.1-(0.75-res.x[2])*math.tan(-math.pi/3), 0.1]

    plt.plot(x, y, 'bo', linestyle='--')
    plt.show()

    x0=[1, 1, 0.2]

    res = minimize(optimize_scenarios, x0=lengths, method='trust-constr', constraints=conMaxDiff, bounds=bounds)
    print(res)

    for i in range(len(requirements['positions'])):
        p1, p2, p3 = calcPoints(res.x, i)

        solutions[i][1] = p1
        solutions[i][2] = p2
        solutions[i][3] = p3

    x1 = [solutions[0][i][0] for i in range(len(requirements['positions']) + 1)]
    y1 = [solutions[0][i][1] for i in range(len(requirements['positions']) + 1)]

    x2 = [solutions[1][i][0] for i in range(len(requirements['positions']) + 1)]
    y2 = [solutions[1][i][1] for i in range(len(requirements['positions']) + 1)]

    x3 = [solutions[2][i][0] for i in range(len(requirements['positions']) + 1)]
    y3 = [solutions[2][i][1] for i in range(len(requirements['positions']) + 1)]

    fig = plt.figure()
    axis = fig.add_subplot()

    axis.plot(x1, y1, 'bo', c='r', linestyle='--', label='scenario 1')
    axis.plot(x2, y2, 'bo', c='b', linestyle='--', label='scenario 2')
    axis.plot(x3, y3, 'bo', c='g', linestyle='--', label='scenario 3')

    plt.legend(loc='upper left')

    plt.show()


main()