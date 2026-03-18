import subprocess

def run_ssh(cmd):
    full_cmd = f"sshpass -p 'moobot' ssh -o StrictHostKeyChecking=no pi@IMOOBOT.local \"{cmd}\""
    res = subprocess.run(full_cmd, shell=True, capture_output=True, text=True)
    return res.stdout, res.stderr

print("Fixing slam_toolbox config...")
out, err = run_ssh("sed -i 's/transform_publish_period: 0.5/transform_publish_period: 0.05/g' /home/pi/robot_ws/src/clean_bot_hardware/config/mapper_params_online_async.yaml")
print(out, err)
print("Verifying...")
out, err = run_ssh("grep transform_publish_period /home/pi/robot_ws/src/clean_bot_hardware/config/mapper_params_online_async.yaml")
print(out)
