import subprocess
import time
import os

robot_names = []
robot_servers = []
waitstart = None
while True:
    if os.path.isfile('/home/baxter_bridge/sockets.txt'):
        with open('/home/baxter_bridge/sockets.txt', 'r') as f:
            lines = f.readlines()
        for l in lines:
            try:
                robot_ns = l.strip().split()[0]
                port = l.strip().split()[1]
                if not (robot_ns in robot_names):
                    waitstart = time.time()          # refresh waitstart, if not refreshed in 10 sec quit infinite loop
                    robot_names.append(robot_ns)
                    pserver = subprocess.Popen(["python", "baxterServer.py", robot_ns, port])
                    robot_servers.append(pserver)
            except:
                pass
        if (waitstart is not None) and (time.time() - waitstart > 10):
            break


for server in robot_servers:
    server.wait()

# erase content before exit
with open('/home/baxter_bridge/sockets.txt', "w") as f:
    pass

