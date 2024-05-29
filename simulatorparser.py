from capturer import CaptureOutput
import subprocess
import re

def count_packets_destroyed(command_output):
    pattern = r'Packet destroyed by interference with SF(\d+)'
    sf_counts = {7: 0, 8: 0, 9: 0, 10: 0, 11: 0, 12: 0}
    matches = re.findall(pattern, command_output)
    for sf in matches:
        sf = int(sf)
        if sf in sf_counts:
            sf_counts[sf] += 1
    return sf_counts

def main():
    command = "./ns3 run complete-network-example"
    with CaptureOutput() as capturer:
        subprocess.run(command, shell=True)

    command_output = capturer.get_text()
   
    # Print the top 5 lines
    top_5_lines = command_output.split('\n')[:5]
    print("Top 5 lines:")
    for line in top_5_lines:
        print(line)

    # Extracting values from the line next to 'Computing performance metrics...'
    metrics_line_index = command_output.find('Computing performance metrics...')
    next_line_index = command_output.find('\n', metrics_line_index) + 1
    next_line = command_output[next_line_index:]
    # Extracting two values from the line separated by a space
    extracted_values = next_line.strip().split(' ')[:2]

    sf_counts = count_packets_destroyed(command_output)
    print("\nPacket counts for SF7, SF8, SF9, SF10, SF11, and SF12:")
    # Inserting extracted values at the start of the final array
    sf_count_array = [float(val) for val in extracted_values] + [sf_counts[sf] for sf in [7, 8, 9, 10, 11, 12]]
    print(sf_count_array)

if __name__ == "__main__":
    main()

