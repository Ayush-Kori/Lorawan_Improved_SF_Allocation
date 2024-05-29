import subprocess
import re

import re

def extract_node_positions(command_output):
    pattern = r'Node Id and its position:(\d+):(\d+):(\d+):(\d+)'
    matches = re.findall(pattern, command_output)
    node_positions = []
    for match in matches:
        node_id = str(match[0])
        x_coord = str(match[1])
        y_coord = str(match[2])
        z_coord = str(match[3])
        node_positions.append(f"{node_id}:{x_coord}:{y_coord}:{z_coord}")
    return node_positions


def main():
    command = "./ns3 run complete-network-example"
    result = subprocess.run(command, shell=True, capture_output=True, text=True)
    print("Subprocess return code:", result.returncode)
    print("Subprocess stdout:")
    print(result.stdout)
    print("Subprocess stderr:")
    print(result.stderr)

    if result.returncode == 0:
        command_output = result.stdout
        node_positions = extract_node_positions(command_output)
        print("\nNode positions:")
        for position in node_positions:
            print(position)

if __name__ == "__main__":
    main()

