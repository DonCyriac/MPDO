from z3 import Not, Bool, Int, Optimize
from z3 import And, Or, simplify, Implies, sat, unsat
import random
import sys
import time

class Obstacle:
    moves = [(0,0), (1,0), (0,1), (-1,0), (0,-1)]

    def __init__(self, x, y, grid):
        self.x = x
        self.y = y
        self.grid = grid
        self.dx, self.dy = random.choice(self.moves)
        self.path = [(x, y)]
    
    def next_move(self):
        # while (True):
        # if ((0 <= self.x+self.dx < self.grid) and (0 <= self.y+self.dy < self.grid)):
        self.x += self.dx
        self.y += self.dy
        self.path.append((self.x, self.y))
        return (self.x, self.y)
            # else:
            #     break

def next_intersection_points(robot_pos, obs_pos):
    (rx, ry) = robot_pos
    (obx, oby) = obs_pos # (a, b)

    robot_pos = set([(rx, ry), (rx+1, ry), (rx, ry+1), (rx-1, ry), (rx, ry-1)])
    obs_halo = set([(obx, oby),(obx+1, oby), (obx, oby+1), (obx-1, oby),(obx, oby-1)])
    return list(robot_pos.intersection(obs_halo))

def get_plan(m):
    return sorted([d.name() for d in m.decls() if m[d]==True], key=lambda item: int(item.split('_')[1]))

def get_robot_pos(m,hop):
    (_, _, x, y) = get_plan(m)[hop].split('_') 
    return (int(x), int(y)) 

def path_valid(robot_plan, obs_plan):
    return len([(a, b) for a, b in list(zip(robot_plan, obs_plan)) if a == b]) == 0

def distance(x1, y1, x2, y2):
        return abs(x1-x2) + abs(y1-y2)    

# GRID_SZ = 10
# HOPS = 18

def main(args):
    # print(args)
    seed = int(args[0])
    random.seed(seed)
    GRID_SZ = int(args[1])
    HOPS = int(args[2])


    print("WORKSPACE SIZE (%s x %s)" % (GRID_SZ, GRID_SZ))
    print("HOPS ALLOWED = %s" % (HOPS))


    
    # X is a three dimensional grid containing (t, x, y)
    X =  [ [ [ Bool("x_%s_%s_%s" % (k, i, j)) for j in range(GRID_SZ) ]
        for i in range(GRID_SZ) ] 
        for k in range(HOPS+1)]

    s = Optimize()

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
    
    #Motion primitives
    for t in range(HOPS):
        for x in range(GRID_SZ):
                for y in range(GRID_SZ):
                    temp = Or(X[t][x][y])
                    if (x+1 < GRID_SZ):
                        temp = Or(temp, X[t][x+1][y])
                    if (y+1 < GRID_SZ):
                        temp = Or(temp, X[t][x][y+1])
                    if (x-1 >= 0):
                        temp = Or(temp, X[t][x-1][y])
                    if (y-1 >= 0):
                        temp = Or(temp, X[t][x][y-1])
                    s.add(simplify(Implies(X[t+1][x][y], temp)))


    # Cost constraints
    for t in range(HOPS):
        for x in range(GRID_SZ):
            for y in range(GRID_SZ):
                s.add_soft(Not(X[t][x][y]), distance(x, y, GRID_SZ-1, GRID_SZ-1))


    if s.check() == sat:
        m = s.model()
    else:
        print("No.of hops too low...")
        exit(1)
    # obs1 = Obstacle(0, 3, GRID_SZ)
    obs = [Obstacle(0, 3, GRID_SZ), Obstacle(2, 2, GRID_SZ)]
    robot_plan = []
    hop = 0
    # obs_plan = []
    # for a in s.assertions():
    #     print(a)
    while (hop < HOPS):
        # print("hops = ", hop)
        
        robot_pos = get_robot_pos(m,hop)
        print("robot_pos = ", robot_pos)

        for obstacle in obs:
            print("Obstacle at ", (obstacle.x, obstacle.y))

        print('-----------')
        s.add(X[hop][robot_pos[0]][robot_pos[1]])

        robot_plan.append(robot_pos)
        # obs_plan.append(obs_pos)
        #next position of the robot
        next_robot_pos = get_robot_pos(m,hop+1)
        s.push() 
        # print("intersection points")
        # print(intersection_points(robot_pos, obs_pos))
        # count = 0
        # print("blah!!!")
        next_overlap = []
        for obstacle in obs:
            next_overlap = next_overlap + next_intersection_points(robot_pos, (obstacle.x, obstacle.y))
       
        # print("Lenght = ", len(next_overlap))
        for (x, y) in next_overlap:
            # consider only the intersection with the next step in the plan
            if ((0 <= x < GRID_SZ) and (0 <= y < GRID_SZ)):
                s.add(Not(X[hop+1][x][y]))

        # print("ceeeee!!", next_robot_pos in set(next_overlap))
        
        if next_robot_pos in set(next_overlap): # we need to find a new path
            # print("just before check")
            if (s.check() == unsat):
                print("stay there")
                # hop -= 1
            else:
                m = s.model()
                hop += 1
            # print("just after check")
        else:
            # we don't need to worry about the path
            hop += 1
        # print('dssdfdsfds')
        s.pop() 
        for obstacle in obs:
            # print("12")
            obstacle.next_move()
        
        
    robot_pos = get_robot_pos(m,hop)
    for obstacle in obs:
        obstacle.next_move()
    # print("hop is ", hop)
    # print("robot at ", robot_pos)
    # print("obs at ", obs_pos)
    robot_plan.append(robot_pos)
    # obs_plan.append(obs_pos)

    for obstacle in obs:
        if not path_valid(robot_plan, obstacle.path):
            print("PATH IS INVALID!!!")


    print("ROBOT MOVEMENT:")
    print(robot_plan)
    print("OBSTACLE MOVEMENT:")
    for obstacle in obs:
        print(obstacle.path)

if __name__ == "__main__":
    # print(sys.argv)
    start_time = time.time()
    return_code = main(sys.argv[1:])
    print("--- %s seconds ---" % (time.time() - start_time))
    if return_code == 1:
        exit(1)
    else:
        exit(0)



