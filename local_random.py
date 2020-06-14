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
        self.path = [(x, y)]
    
    def next_move(self):
        # while (True):
        # if ((0 <= self.x+self.dx < self.grid) and (0 <= self.y+self.dy < self.grid)):
        dx, dy = random.choice(self.moves)
        self.x += dx
        self.y += dy
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

    obs = [Obstacle(3, 3, GRID_SZ), Obstacle(4, 5, GRID_SZ), Obstacle(6, 7, GRID_SZ), Obstacle(8, 9, GRID_SZ), Obstacle(9, 3, GRID_SZ), Obstacle(1, 8, GRID_SZ), Obstacle(7, 7, GRID_SZ)]
    # obs = [Obstacle(3, 3, GRID_SZ)]


    ## 

    # obs = []
    # for i in range(0, GRID_SZ-1):
    #     for j in range(0, GRID_SZ-1):
    #         if (i+j)%3 != 0:
    #             print(i, j)
    #             obs.append(Obstacle(i, j, GRID_SZ))
    # print("-----")




    robot_plan = []
    hop = 0
    stay_count = 0
    # obs_plan = []
    # for a in s.assertions():
    #     print(a)
    while (hop < HOPS):
        # print("hops = ", hop)
        if stay_count > 0:
            robot_pos = [robot_plan[-stay_count][0], robot_plan[-stay_count][1]]
            next_robot_pos = get_robot_pos(m,hop-stay_count)
        else:
            robot_pos = get_robot_pos(m,hop)
            next_robot_pos = get_robot_pos(m,hop+1)
        # print("robot_pos = ", robot_pos)

        # for obstacle in obs:
        #     print("Obstacle at ", (obstacle.x, obstacle.y))

        # print('-----------')
        s.add(X[hop][robot_pos[0]][robot_pos[1]])

        robot_plan.append(robot_pos)
        # obs_plan.append(obs_pos)
        #next position of the robot
        # if (stay:
        
        # s.push() 
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
                print(robot_pos)
                stay_count += 1
                # hop -= 1
                hop += 1
            else:
                stay_count = 0
                m = s.model()
                hop += 1
            # print("just after check")
        else:
            # we don't need to worry about the path
            hop += 1
        # print('dssdfdsfds')
        # s.pop() 
        for obstacle in obs:
            # print("12")
            obstacle.next_move()

    if (stay_count > 0):
        return 1    
    
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



