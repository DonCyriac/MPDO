from z3 import *
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
        while (True):
            dx, dy = random.choice(self.moves)
            if ((0 <= self.x+dx < self.grid) and (0 <= self.y+dy < self.grid)):
                self.x += dx
                self.y += dy
                self.path.append((self.x, self.y))
                return (self.x, self.y)

def get_plan(m):
    return sorted([d.name() for d in m.decls() if m[d]==True], key=lambda item: int(item.split('_')[1]))

# GRID_SZ = 10
# HOPS = 20

def main(args):
    seed_val = int(args[0])
    GRID_SZ = int(args[1])
    HOPS = int(args[2])

    print("WORKSPACE SIZE (%s x %s)" % (GRID_SZ, GRID_SZ))
    print("HOPS ALLOWED = %s" % (HOPS))

    random.seed(seed_val)
    # X is a three dimensional grid containing (t, x, y)
    X =  [ [ [ Bool("x_%s_%s_%s" % (k, i, j)) 
        for j in range(GRID_SZ) ]
        for i in range(GRID_SZ) ] 
        for k in range(HOPS+1)]

    s = Solver()

    # Initial Constraints
    s.add(X[0][0][0])
    s.add([Not(cell) for row in X[0] for cell in row][1:])

    # Final constraints
    s.add(X[HOPS][GRID_SZ-1][GRID_SZ-1])
    s.add([Not(cell) for row in X[HOPS] for cell in row][:-1])

    # Sanity Constraints
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

    #Collision avoidance
    obs = [Obstacle(3, 3, GRID_SZ), Obstacle(4, 5, GRID_SZ), Obstacle(6, 7, GRID_SZ), Obstacle(8, 9, GRID_SZ), Obstacle(9, 3, GRID_SZ), Obstacle(1, 8, GRID_SZ), Obstacle(7, 7, GRID_SZ)]
    # obs = [Obstacle(0, 3, GRID_SZ), Obstacle(2, 2, GRID_SZ), Obstacle(7, 8, GRID_SZ)]


    # obs_plan = []
    for o in obs:
        for time in range(HOPS+1):
            obs_pos = (o.x, o.y)
            # obs_plan.append(obs_pos)
            s.add(Not(X[time][obs_pos[0]][obs_pos[1]]))
            obs_pos = o.next_move()


    if s.check() == sat:
        m = s.model()
        print(get_plan(m))
        for obstacle in obs:
            print(obstacle.path)
        # print(obs_plan)
        return 0
    else:
        print("UNSAT!!")
        return 1

if __name__ == "__main__":
    # print(sys.argv)
    start_time = time.time()
    return_code = main(sys.argv[1:])
    print("--- %s seconds ---" % (time.time() - start_time))
    if return_code == 1:
        exit(1)
    else:
        exit(0)

