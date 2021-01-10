import os
import subprocess
import time

code_path = (os.environ.get("NAO_CODE_LOCATION"))

nao_folder = code_path.split("workspace")

#qilci search

output = subprocess.Popen(["locate", "qicli"],
                          stdout=subprocess.PIPE).communicate()[0]
output = output.decode()

search_results = output.split('\n')

qicli_path = ([
    x for x in search_results if "naoqi-sdk-2.8.5" in x and x.endswith("qicli")
][0])

#cpp-nao-control search
output = subprocess.Popen(["find", "./", "-name", "cpp-nao-control"],
                          stdout=subprocess.PIPE).communicate()[0]
output = output.decode()

search_results = output.split('\n')

cpp_nao_control_path = ([
    x for x in search_results if x.endswith("cpp-nao-control")
][0])

process_array = []

for robot_id in range(4):
    cpp_nao_control_process = subprocess.Popen(
        [cpp_nao_control_path, "--qi-url", f":{9600+robot_id}"])
    time.sleep(1)
    
    init_process = subprocess.Popen([
        qicli_path, "call", "VrepSimulation.init",
        str(robot_id), "--qi-url", f":{9600+robot_id}"
    ]).wait()

    jointControl_process = subprocess.Popen([
        qicli_path, "call", "VrepSimulation.jointControl", "--qi-url",
        f":{9600+robot_id}"
    ])

    process_array.append((cpp_nao_control_process, jointControl_process))

input()
for process in process_array:
    process[0].terminate()
    process[1].terminate()