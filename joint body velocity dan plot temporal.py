# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# %%
# 1. Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')

# %%
# 2. Start Simulation
sim.startSimulation()
print("Simulation Started")

# %%
# 3. Simple Test: Post a message to CoppeliaSim status bar
sim.addLog(1, "Hello from Python!")

p3dx_RW = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_LW = sim.getObject("/PioneerP3DX/leftMotor")
p3dx = sim.getObject("/PioneerP3DX")

rw = 0.195/2
rb = 0.381/2
d = 0.05

dt = 0.001
x_pose_int = 0
y_pose_int = 0
x_pose_int2 = 0
y_pose_int2 = 0
gamma_int = 0
# Data storage for temporal plot
time_data = []
x_pose = []
y_pose = []
x_pose2 = []
y_pose2 = []

# %%
try:
    # 4. Main Loop (Run for 10 seconds)
    start_time = time.time()
    elapsed_prev = 0
    
    while (time.time() - start_time) < 20:
        
        # --- STUDENT CODE GOES HERE ---
        # Example: Print elapsed time
        elapsed = time.time() - start_time
        print(f"Running... {elapsed:.1f}s", end="\r")
        
        dt = elapsed - elapsed_prev
        elapsed_prev = elapsed
    
       
        wr_vel = sim.getJointTargetVelocity(p3dx_RW)
        wl_vel = sim.getJointTargetVelocity(p3dx_LW)
        
        sim.addLog(1, f"RW:{wr_vel:.1f}rad/s, LW:{wl_vel:.1f}rad/s")
        
        vx = (wr_vel + wl_vel)*rw/2
        wx = (wr_vel - wl_vel)*rw/rb
        
        
        theta = sim.getObjectOrientation(p3dx, sim.handle_world)
        
        x_dot = vx * math.cos(theta[2])
        y_dot = vx * math.sin(theta[2])
        
        x_pose_int = x_pose_int + x_dot * dt
        y_pose_int = y_pose_int + y_dot * dt
        
        gamma_int = gamma_int + wx * dt
        
        x_dot2 = vx * math.cos(gamma_int)
        y_dot2 = vx * math.sin(gamma_int)
        
        x_pose_int2 = x_pose_int2 + x_dot2 * dt
        y_pose_int2 = y_pose_int2 + y_dot2 * dt
       
        sim.addLog(1, f"RW:{wr_vel:.1f}rad/s, LW:{wl_vel:.1f}rad/s")
        sim.addLog(1, f"vx:{vx:.1f}m/s, wx:{wx:.1f}rad/s")
        sim.addLog(1, f"x_dot:{x_dot:.1f}m/s, y_dot:{y_dot:.1f}m/s")
        sim.addLog(1, f"x_dot2:{x_dot2:.1f}m/s, y_dot2:{y_dot2:.1f}m/s")
        sim.addLog(1, f"gamma_int:{gamma_int:.2f}rad")
        sim.addLog(1, f"theta:{theta[2]:.2f}rad")
        sim.addLog(1, f"x_pose_int:{x_pose_int:.1f}m, y_pose_int:{y_pose_int:.1f}m")
        sim.addLog(1, f"x_pose_int2:{x_pose_int2:.1f}m, y_pose_int2:{y_pose_int2:.1f}m")
        
        
        
        
        # Save data for plotting
        time_data.append(elapsed)
        x_pose.append(x_pose_int)
        y_pose.append(y_pose_int)
        x_pose2.append(x_pose_int2)
        y_pose2.append(y_pose_int2)
    
        
    

finally:
    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")
    
    # Temporal Plot
plt.figure()
plt.plot(x_pose, y_pose, label="From Orientation")
plt.plot(x_pose2, y_pose2, '--', label="From Integration")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Robot Path")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.show()