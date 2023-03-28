class MotionPlanning(Drone):
def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        # initial state
        self.flight_state = States.MANUAL
        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()
conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)
drone.start()
# TODO: read lat0, lon0 from colliders into floating point values
        lat0_lon0 = pd.read_csv('colliders.csv', nrows = 1, header = None)
        lat0, lon0 = lat0_lon0.iloc[0,0], lat0_lon0.iloc[0,1]
        _, lat0 = lat0.split()
        _, lon0 = lon0.split()
        lat0 = np.float64(lat0)
        lon0 = np.float64(lon0)
# TODO: set home position to (lat0, lon0, 0)
        self.set_home_position(lon0, lat0, 0)
# TODO: retrieve current global position 
# TODO: convert to current local position using global_to_local()
current_local_pos = global_to_local(self.global_position, self.global_home)
        
# Me: checking current local position
print ('current local position {0} and {1}'.format(current_local_pos[0], current_local_pos[1]))
        
print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,self.local_position))
# Read in obstacle map
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=3)
        
# Define a grid for a particular altitude and safety margin around obstacles
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
# minimum and maximum north coordinates
north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))
# minimum and maximum east coordinates
east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))
# given the minimum and maximum coordinates we can
# calculate the size of the grid.
north_size = int(np.ceil((north_max - north_min + 1)))
east_size = int(np.ceil((east_max - east_min + 1)))
# Initialize an empty grid
grid = np.zeros((north_size, east_size))
# Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1
return grid, int(north_min), int(east_min)
# TODO: convert start position to current position rather than map center
grid_start = (int(current_local_pos[0] - north_offset), int(current_local_pos[1] - east_offset))
# Take GPS co-ordinates as Grid goal -Me
grid_goal = (-122.396582, 37.795714, 0)
grid_goal = global_to_local(grid_goal, self.global_home)
grid_goal = (int(grid_goal[0] - north_offset), int(grid_goal[1] - east_offset))
print('Local Start and Goal: ', grid_start, grid_goal)
path, _ = a_star(grid, heuristic, grid_start, grid_goal)
path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)
branch = {}
found = False
# Check till we have searched all nodes or have found our ‘goal’
while not queue.empty():
    item = queue.get() # Step2. Visit the topmost node in the queue
    current_cost = item[0]
    current_node = item[1]
    if current_node == goal:
      print('Found a path.')
      found = True
      break
    
    else:
#Step3. If that node has any neighbors, check to see if they have been “visited” or not.
      
      for a in valid_actions(grid, current_node):
        next_node = (current_node[0] + a.delta[0], current_node[1] + a.delta[1])
        new_cost = current_cost + a.cost + h(next_node, goal)
#Step4. Add any neighboring nodes that still need to be “visited” to the queue.
        if next_node not in visited:
          visited.add(next_node)
          queue.put((new_cost, next_node))
          branch[next_node] = (new_cost, current_node, a)
          if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost
    
