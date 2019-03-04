import numpy as np
from pylab import *

map_grid = np.zeros((1000,1000,1000))

#define drone area with start and end point
# 0 shows avaliable waypoints
# 1 shows obtacles
# 7 is start points
# 5 is ends
class Jsonable:
    def json(self):
        return self.__dict__
    
class Waypoints(Jsonable):
    def __init__(self, id, waypoints1,waypoints2):
        self.id = id
        self.waypoints1 = waypoints1
        self.waypoints2 = waypoints2
       
    
class Pathfinder(Jsonable):  
    waypoints = []
    def __init__(self):
        return
    
    @staticmethod  
    def getRoute(data):
        pathf = Pathfinder()
        #define obtacles coordinates
        for obstacle in data.static_obstacles:
            x = obstacle['position']['x']
            y = obstacle['position']['y']
            z = obstacle['position']['z']
            map_grid[x, y, z] = 1
        
        
        #define start and end points
        for drone in data.drones:
            if drone['active']:
                #obtain initial position, start and end points from drone data
                x0 = drone['position']['x']
                y0 = drone['position']['y']
                z0 = drone['position']['z']
                initialPoint = [x0, y0, z0]
                
                x1 = drone['pick_up']['x']
                y1 = drone['pick_up']['y']
                z1 = drone['pick_up']['z']
                pickPoint = [x1, y1, z1]
        
                x2 = drone['destination']['x']
                y2 = drone['destination']['y']
                z2 = drone['destination']['z']
                destPoint = [x2, y2, z2]

                map_grid[initialPoint[0], initialPoint[1], initialPoint[2]] = 7
                map_grid[pickPoint[0], pickPoint[1], pickPoint[2]] = 5
                a1 = AStar(initialPoint,pickPoint, map_grid)
                a1.main()
                map_grid[initialPoint[0], initialPoint[1], initialPoint[2]] = 0
                map_grid[pickPoint[0], pickPoint[1], pickPoint[2]] = 0
               
                map_grid[pickPoint[0], pickPoint[1], pickPoint[2]] = 7
                map_grid[destPoint[0], destPoint[1], destPoint[2]] = 5
                a2 = AStar(pickPoint,destPoint,map_grid)
                a2.main()
                
                pathf.waypoints.append(
                        Waypoints(drone['id'], a1.waypoints, a2.waypoints).json())
                
                
                map_grid[pickPoint[0], pickPoint[1], pickPoint[2]] = 0
                map_grid[destPoint[0], destPoint[1], destPoint[2]] = 0

                
#A* algorithm
class AStar:
    def __init__(self,startPoint,endPoint,map_grid):
        self.f = 0
        self.g = 0
        self.startPoint = startPoint
        self.endPoint = endPoint
        self.map_grid = map_grid
        self.last_point = np.array([])  # update last point continuely
        self.current_point = np.array([])  # update current point continuely
        self.open = np.array([[],[],[]])  # create a empty open array
        self.closed = np.array([[], [], []])  # create a empty close array
        self.waypoints = [] # create a waypoints list
        
        self.start = np.array(self.startPoint)  # start point
        self.goal = np.array(self.endPoint)  # end point

    def h_value_tem(self, current_p):
        #calculate heuristic estimated cost h value between each nodes and destination
        #current_p: coordinate of current node
        
        #Manhattan distance
        h = np.abs(current_p[0] - self.endPoint[0]) + np.abs(current_p[1] - self.endPoint[1])+ np.abs(current_p[2] - self.endPoint[2])
        
        return h

    def g_value_tem(self, child_p, current_p):
        #calculate estimated cost g value between current and child points
        #:param child_p:coordinate of child point
        #:param current_p:coordinate of parent point (self.current_point)
        
        g1 = current_p[0] - child_p[0]
        g2 = current_p[1] - child_p[1]
        g3 = current_p[2] - child_p[2]
        g = g1 ** 2 + g2 ** 2 + g3 ** 2
        g = np.sqrt(g)
        return g

    def f_value_tem(self, child_p, current_p):
        #calculate f value considering both g and h value
        #:return:
        
        f = self.g_value_tem(child_p, current_p) + self.h_value_tem(current_p)
        return f

    def min_f(self):
        #find the minimum value in open and take this point as current_point
        #:return index and coordinates of this point
        #if reaching the boundary, next direction is random
        #and clear the list of open
        
        tem_f = []  # creat a temporary list of f values 
        for i in range(self.open.shape[1]):
            # calculate total f values for current nodes
            f_value = self.f_value_tem(self.current_point, self.open[:, i]) + self.g
            tem_f.append(f_value)
        index = tem_f.index(min(tem_f))  # return smallest index value
        location = self.open[:, index]  # return smallest coordinates value
      
        return index, location

    def child_point(self, x):
        #:param x: parent coordinates 
        #:return: void, store child points in open list 

        # search 8 adjacent points
        for j in range(-1, 2, 1):
            for q in range(-1, 2, 1):
                for w in range(-1, 2, 1):
                    if j == 0 and q == 0 and w == 0:  # remove current-point(parent point)
                        continue

                    if self.map_grid[int(x[0] + j), int(x[1] + q),int(x[2] + w)] == 1:  # remove obtacle
                        continue
                    if x[0] + j < 0 or x[0] + j > self.map_grid.shape[0]-1 or x[1] + q < 0 or x[1] + q > self.map_grid.shape[1]-1 or x[2] + w < 0 or x[2] + w > self.map_grid.shape[2]-1:  # remove point that is out of boundary
                        continue
                    # remove searching point from open list 
                    a = self.judge_location(x, j, q, w, self.open)
                    if a == 1:
                        continue
                    # remove searching point from close list
                    b = self.judge_location(x, j, q, w, self.closed)
                    if b == 1:
                        continue

                    m = np.array([x[0] + j, x[1] + q, x[2] + w])
                    self.open = np.c_[self.open, m]  # adding the child point in open list 


    def judge_location(self, x, j, q, w, list_co):
        #decide point exists whether in open or close lists 
        #:return:

        jud = 0
        for i in range(list_co.shape[1]): #print how many columns

            if x[0] + j == list_co[0, i] and x[1] + q == list_co[1, i] and x[2] + w == list_co[2, i]:

                jud = jud + 1
            else:
                jud = jud

        return jud




    def main(self):

        self.open = np.column_stack((self.open, self.start))  # put start point in open list
        self.current_point = self.start  
        
        ite = 1
        while ite <= 2000:
            # return if open list is empty
            if self.open.shape[1] == 0:
                print('no routing path！')
                return

            last_point = self.current_point 

            index, self.current_point = self.min_f()  # decide f values in open file
            
            self.waypoints.append(self.current_point) 

            #choose points with smallest f value and put them into close list
            self.closed = np.c_[self.closed, self.current_point]

            if self.current_point[0] == self.endPoint[0] and self.current_point[1] == self.endPoint[1] and self.current_point[2] == self.endPoint[2]: 
                
                return

            self.child_point(self.current_point)  # generate child_point
            self.open = delete(self.open, index, axis=1)  # delete optimal point in open list

            self.g = self.g + self.g_value_tem(self.current_point, last_point)

            ite = ite+1




