import numpy as np
from robot_properties_go1.config import Go1Config
from reactive_planners.reactive_planners_go1_step_adjustment import go1_reactive_planners
import time
import tracemalloc

tracemalloc.start()
num_simulation = 10
length_simulation = 1000
stepper = []
start_time = time.time()

for i in range(num_simulation):
    print(i)
    stepper.append(go1_reactive_planners())
    stepper[-1].start()

initialization_time = time.time()
print("Initialization time", initialization_time - start_time)

for i in range(length_simulation):
    for j in range(num_simulation):
        stepper[j].step(Go1Config.initial_configuration, Go1Config.initial_velocity, 0)

end_time = time.time()
print("Simulating steps time", end_time - initialization_time)
tracemalloc.stop()
print(tracemalloc.get_tracemalloc_memory(), "bytes")