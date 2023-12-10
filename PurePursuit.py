"""
Alapot szolgáltató kód készítője:
author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)
"""

import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy import interpolate


x=[0, 5, 2, 0]
y=[0, 0, 5, 0]

#x = [0, 1, 0, 1, 0]
#y = [0, 0, 0.7, 0.7, 0]

#x = [0, 0.5, 0.5, 0, 0]
#y = [0, 0, 0.5, 0.5, 0]

    #x = [20,50, -10, -50, 0]
    #y = [10, 20, 30, -20, 0]

koztes = 20

OnlySim = True
if not OnlySim:
    import motion
    import rclpy
    

# Parameters
k = 0.1  # forward gain
Lfc = 0.1  # [m] look-ahead táv
Kp = 0.1  # speed gain
dt = 0.1  # [s] time tick
WB = 0.1  # [m] tengelytávolság
eps = 0.2 # [m] epszilon eltérés a céltól
target_speed = 1  # [m/s]
T = 1000.0  # max szimulációs idő
slow_distance = 15 #[m] távolság, ami alatt le kéne lassulnia checkpointig a robotnak


show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        # self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        # self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        self.rear_x = self.x
        self.rear_y = self.y

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        # self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        # self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        self.rear_x = self.x
        self.rear_y = self.y

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        #következő pont keresésének meggyorsítása
        if self.old_nearest_point_index is None:
            #következő pont indexének keresése
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True and (ind < len(self.cx)-1): #Ha valami miatt a this_index nem kisebb a nextnél, akkor se fagyjon ki
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        #következő pont keresése
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  #nem ért célt
            ind += 1
        
        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def plot_arrow(x, y, yaw, length=0.01, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def continous_cord(x, y, num_intermediate_points):
    '''Felosztja N szakaszra a koordináták által meghatározott szakaszt'''
    result_x = []
    result_y = []

    for i in range(len(x) - 1):
        current_x = x[i]
        current_y = y[i]
        next_x = x[i + 1]
        next_y = y[i + 1]

        result_x.append(current_x)
        result_y.append(current_y)

        intermediate_x = np.linspace(current_x, next_x, num_intermediate_points + 2)[1:-1]
        intermediate_y = np.linspace(current_y, next_y, num_intermediate_points + 2)[1:-1]

        result_x.extend(intermediate_x)
        result_y.extend(intermediate_y)

    result_x.append(x[-1])
    result_y.append(y[-1])

    return result_x, result_y


def main():
    if not OnlySim:
        rclpy.init(args=None)
        robot = motion.Motion()
        while not robot.robot.local_deg:
            print(robot.robot.local_deg)
        robot.robot.reset()
        
    
    

    global target_speed

   


    cx = x
    cy = y

    state = State(x=0, y=0, yaw=-0.0, v=0.0)
    states = States() #Ebben vannak tárolva a statek, amik x,y koordinátából, irányból, időből és sebességből állnak
    states.append(0,state)

    ai = 0
    di = 0
    time = 0.0

    for i in range(1, len(x)):
    
        state = State(x=states.x[-1], y=states.y[-1], yaw=states.yaw[-1], v=0.0) #Megállunk a pontban, itt csinálunk valamit és megyünk tovább 0 sebességről


        print('Cx: ',cx)
        print('Cy: ', cy)
        cx, cy = continous_cord([x[i-1],x[i]], [y[i-1], y[i]], koztes)
        plt.plot(cx,cy)
        #cx, cy = continous_cord(x[i-1:i], y[i-1:i], 5)
        print('Cx: ',cx)
        print('Cy: ', cy)
        #lastIndex = len(cx) - 1
        states.append(time, state)
        target_course = TargetCourse(cx, cy)
        target_ind, _ = target_course.search_target_index(state)

        

        while T >= time and np.sqrt((state.x - cx[-1])**2 + (state.y - cy[-1])**2) > eps: #Itt indul a móka, ha kifut az időből vagy már epszilon sugarú körben van akkor megáll eredeti: lastIndex > target_ind
            
            # Calc control input
            ai = proportional_control(target_speed, state.v) #Kiszámolt gyorsulás - valós
            di, target_ind = pure_pursuit_steer_control(
                state, target_course, target_ind)


            if np.sqrt((state.x - cx[-1])**2 + (state.y - cy[-1])**2) < slow_distance:
                if state.v > 0.5: ai = -0.5
            
            print(np.rad2deg(di), ' ', np.rad2deg(state.yaw))


            state.update(ai, di)  # Control vehicle
            if not OnlySim:
                #print(robot.robot.local_deg)
                state.yaw = np.deg2rad(robot.robot.local_deg)
                #state.yaw = robot.robot.local_deg
                
                robot.teszt(state.v, di)
            #print(np.rad2deg(di))

            

            time += dt
            states.append(time, state)

            if show_animation and OnlySim:  # pragma: no cover
                plt.cla()
                #kilépéshez nyomd meg az esc gombot
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                plot_arrow(state.x, state.y, state.yaw)
                plt.plot(x,y)
                plt.plot(cx, cy, "-r", label="course")
                plt.plot(states.x, states.y, "-b", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                plt.axis("equal")
                plt.grid(False)
                plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                plt.pause(0.001)

        # state.v = 0
        # state.update(-ai, di)
        # states.append(time, state)
        #print(states.x[-1])
        print('check')
        if not OnlySim:
            robot.forward(0. , 0.)
    
    
        

if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()