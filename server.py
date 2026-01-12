# server.py
from flask import Flask, jsonify
import subprocess

app = Flask(__name__)

@app.route("/run", methods=["POST"])
def run_scripts():
    try:
        subprocess.run(["python3", "clean_bot_hardware/launch/robot_bringup.launch.py"], check=True)
        subprocess.run(["python3", "clean_bot_mission/clean_bot_mission/full_mission.py"], check=True)
        return jsonify({"status": "success"})
    except subprocess.CalledProcessError as e:
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
