import numpy as np

def plan_path(start, end, max_step_size):
    # calculate the distance between start and end
    distance = np.linalg.norm(np.array(end) - np.array(start))
    # calculate the number of steps required to reach the end
    num_steps = distance / max_step_size
    # round num_steps to the nearest integer
    num_steps = round(num_steps)
    # create an array of equally spaced points between start and end
    path = np.linspace(start, end, int(num_steps))
    return path

# define the start and end points
start = [0, 0]
end = [10, 10]

# calculate the path with a maximum step size of 1
path = plan_path(start, end, 1)

# print the calculated path
print(path)
