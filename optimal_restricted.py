from z3 import Not, Bool, Int, Optimize, Solver
from z3 import And, Or, simplify, Implies, sat, unsat
import random
import sys
import time

class Primitive:
    def __init__(self, id, swath, final_x, final_y):
        self.id = id
        self.swath = swath
        self.final_x = final_x
        self.final_y = final_y

class Obstacle:
    moves = [(0,0), (1,0), (0,1), (-1,0), (0,-1)]
    path = []

    def __init__(self, x, y, grid):
        self.x = x
        self.y = y
        self.grid = grid
        self.dx, self.dy = random.choice(self.moves)
    
    def next_move(self):
        # if ((0 <= self.x+self.dx < self.grid) and (0 <= self.y+self.dy < self.grid)):
        self.x += self.dx
        self.y += self.dy
        return (self.x, self.y)

    def add_constraints(self, s, X, HOPS, GRID_SZ):
        for t in range(HOPS+1):
            nx, ny = self.next_move()
            if((0 <= nx < GRID_SZ) and (0 <= ny  < GRID_SZ)):
                self.path.append((nx, ny))
                s.add(Not(X[t][nx][ny]))
    


def next_intersection_points(next_robot_pos, obs_pos):
    (rx, ry) = next_robot_pos
    (obx, oby) = obs_pos

    robot_pos = set([(rx, ry)])
    obs_halo = set([(obx, oby),(obx+1, oby), (obx, oby+1), (obx-1, oby),(obx, oby-1)])
    return list(robot_pos.intersection(obs_halo))

def get_plan(m):
    return sorted([d.name() for d in m.decls() if d.name()[0] == 'x' and m[d]==True], key=lambda item: int(item.split('_')[1]))

def get_robot_pos(m,hop):
    (_, _, x, y) = get_plan(m)[hop].split('_') 
    return (int(x), int(y)) 

def path_valid(robot_plan, obs_plan):
    return len([(a, b) for a, b in list(zip(robot_plan, obs_plan)) if a == b]) == 0

def distance(x1, y1, x2, y2):
        return abs(x1-x2) + abs(y1-y2)    

# GRID_SZ = 10
# HOPS = 18

# print("WORKSPACE SIZE (%s x %s)" % (GRID_SZ, GRID_SZ))
# print("HOPS ALLOWED = %s" % (HOPS))

def main(args):
    # print(args)
    seed = int(args[0])
    GRID_SZ = int(args[1])
    HOPS = int(args[2])


    print("WORKSPACE SIZE (%s x %s)" % (GRID_SZ, GRID_SZ))
    print("HOPS ALLOWED = %s" % (HOPS))


    random.seed(seed)


    # New primitive
    primitives = []

    # Stay there
    primitives.append(Primitive(1, [[0,0]], 0, 0))

    # Move right
    primitives.append(Primitive(2, [[0,0], [1,0]], 1, 0))

    # Move left
    primitives.append(Primitive(3, [[0,0], [-1,0]], -1, 0))

    # Move up
    primitives.append(Primitive(4, [[0,0], [0,1]], 0, 1))
    
    # Move down
    primitives.append(Primitive(5, [[0,0], [0,-1]], 0, -1))

    P =  [ Int("p_%s" % (k)) for k in range(HOPS+1) ]
    
    
    # X is a three dimensional grid containing (t, x, y)
    X =  [ [ [ Bool("x_%s_%s_%s" % (k, i, j)) for j in range(GRID_SZ) ]
        for i in range(GRID_SZ) ] 
        for k in range(HOPS+1)]

    s = Solver()

    # P should be between 1 and 5 for each time step
    # s.add([And(1 <= prim , prim <= 5) for prim in P])
    for prim in P:
        s.add(1 <= prim)
        s.add(prim <= 5)

    # for d in m.decls():
    #     print "%s = %s" % (d.name(), m[d])

  

    #  Make the swath true for the chosen primitive
    # for t in range(HOPS):
    #     for i in range(GRID_SZ):
    #         for j in range(GRID_SZ):
    #             for prim_var in P:
    #                 for prim_instance in primitives:
    #                     for sw in prim_instance.swath:
    #                         if ((0 <= i+sw[0] < GRID_SZ) and (0 <= j+sw[1] < GRID_SZ)):
    #                             s.add(Implies(prim_var == prim_instance.id, X[t+1][i + sw[0]][j + sw[1]]))
                            # else:
                            #     s.add(prim_var != prim_instance.id) # Since a swath cell lies outside the grid point

    # After the timestep the position of the robot should be curr + (final_x, final_y)
    for t in range(HOPS):
        for x in range(GRID_SZ):
                for y in range(GRID_SZ):
                    for prim_instance in primitives:
                        if ((0 <= x + prim_instance.final_x < GRID_SZ) and (0 <= y + prim_instance.final_y < GRID_SZ)):
                            s.add(Implies(And(P[t] == prim_instance.id, X[t][x][y]), X[t+1][x + prim_instance.final_x][y + prim_instance.final_y]))
                        else:
                            s.add(Implies(X[t][x][y], P[t] != prim_instance.id))

    # Initial Constraints
    s.add(X[0][0][0])
    s.add([Not(cell) for row in X[0] for cell in row][1:])

    # Final constraints
    s.add(X[HOPS][GRID_SZ-1][GRID_SZ-1])
    s.add([Not(cell) for row in X[HOPS] for cell in row][:-1])

    #Sanity Constraints
    for grid in X:
        for i in range(len(grid)):
            for j in range(len(grid)):
                for p in range(len(grid)):
                    for q in range(len(grid)):
                        if not (i==p and j==q):
                            s.add(Not(And(grid[i][j], grid[p][q])))


    ## SIMULATION STARTS HERE ##
    # hop = 0
    if s.check() == sat:
        m = s.model()
    else:
        print("No.of hops too low...")
        exit(1)

    obs = [Obstacle(0, 3, GRID_SZ), Obstacle(2, 2, GRID_SZ), Obstacle(7,8, GRID_SZ)]
    
    obs[0].add_constraints(s, X, HOPS, GRID_SZ) # Add future positions of obs1 to solver
    obs[1].add_constraints(s, X, HOPS, GRID_SZ)
    obs[2].add_constraints(s, X, HOPS, GRID_SZ)

    robot_plan = []
    if (s.check() == sat):
        m = s.model()
        robot_plan = get_plan(m)
        print("Robot plan:")
        print(robot_plan)
        print("Obstacle path")
        for obstacle in obs:
            obs_path = obstacle.path
            print(obs_path)

    # for d in m.decls():
    #     if d.name()[0] == 'p':
    #         print("%s = %s" % (d.name(), m[d]))

    
    # # obs_plan = []
    # # for a in s.assertions():
    # #     print(a)
    # while (hop < HOPS):
        
    #     robot_pos = (0, 0) if hop == 0 else get_robot_pos(m,hop)
    #     obs_pos = obs1.next_move()
        
    #     s.add(X[hop][robot_pos[0]][robot_pos[1]])
    #     # print("hop is ", hop)
    #     # print("robot at ", robot_pos)
    #     # print("obs at ", obs_pos)

    #     if robot_pos == obs_pos:
    #         print("COLLISION!!!")
    #         print(robot_plan)
    #         print(obs_plan)
    #         exit()

    #     robot_plan.append(robot_pos)
    #     obs_plan.append(obs_pos)
    #     #next position of the robot
    #     next_robot_pos = get_robot_pos(m,hop+1)
    #     s.push()
    #     # print("intersection points")
    #     # print(intersection_points(robot_pos, obs_pos))
    #     # count = 0
    #     next_overlap = next_intersection_points(next_robot_pos, obs_pos)
    #     for (x, y) in next_overlap:
    #         # consider only the intersection with the next step in the plan
    #         s.add(Not(X[hop+1][x][y]))
        
    #     if len(next_overlap)>0: # we need to find a new path
    #         if (s.check() == unsat):
    #             print("stay there")
    #         else:
    #             m = s.model()
    #             # print("Plan for hop = " + str(hop+1))
    #             # print(get_plan(m))
    #             hop += 1
    #     else:
    #         # we don't need to worry about the path
    #         hop += 1

    #     s.pop()
        
    # robot_pos = get_robot_pos(m,hop)
    # obs_pos = obs1.next_move()
    # # print("hop is ", hop)
    # # print("robot at ", robot_pos)
    # # print("obs at ", obs_pos)
    # robot_plan.append(robot_pos)
    # obs_plan.append(obs_pos)

    # if path_valid(robot_plan, obs_plan):
    #     print("PATH IS VALID!!!")
    # else:
    #     print("PATH IS INVALID!!!")
    # print("ROBOT MOVEMENT:")
    # print(robot_plan)
    # print("OBSTACLE MOVEMENT:")
    # print(obs_plan)

if __name__ == "__main__":
    # print(sys.argv)
    start_time = time.time()
    return_code = main(sys.argv[1:])
    print("--- %s seconds ---" % (time.time() - start_time))
    if return_code == 1:
        exit(1)
    else:
        exit(0)

# add only general motion primitives
# minimize the cost function





