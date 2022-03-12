import subprocess, signal
import os

if __name__ == '__main__':
    p = subprocess.Popen(['ps', '-A'], stdout=subprocess.PIPE)
    out, err = p.communicate()
    for line in out.splitlines():
        if b'python' in line or b'rosmaster' in line or b'rosout' in line or b'roscore' in line or b'ffmpeg' in line or b'RosAria' in line or b'rosrun' in line or b'rviz' in line or b'roslaunch' in line or b'rosbag' in line or b'record' in line:
            pid = int(line.split(None, 1)[0])
            os.kill(pid, signal.SIGKILL)
