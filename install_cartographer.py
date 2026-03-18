import subprocess

def run_ssh(cmd):
    full_cmd = f"sshpass -p 'moobot' ssh -o StrictHostKeyChecking=no pi@IMOOBOT.local \"{cmd}\""
    res = subprocess.run(full_cmd, shell=True, capture_output=True, text=True)
    return res.stdout, res.stderr

print("Installing cartographer ROS 2 packages on Pi...")
out, err = run_ssh("sudo apt-get update && sudo apt-get install -y ros-humble-cartographer ros-humble-cartographer-ros")
print(out)
if err:
    print(f"Error: {err}")
