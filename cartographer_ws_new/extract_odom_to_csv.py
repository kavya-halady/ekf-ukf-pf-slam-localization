import yaml
import csv

input_file = "estimated_trajectory.txt"
output_file = "estimated_trajectory.csv"

with open(input_file, 'r') as f:
    content = f.read()

# Split messages by '---'
messages = content.split('---')

# Prepare CSV
with open(output_file, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['timestamp', 'pos_x', 'pos_y', 'pos_z', 'ori_x', 'ori_y', 'ori_z', 'ori_w'])

    for msg in messages:
        try:
            data = yaml.safe_load(msg)
            if not data:
                continue
            sec = data['header']['stamp']['sec']
            nanosec = data['header']['stamp']['nanosec']
            timestamp = sec + nanosec * 1e-9

            pos = data['pose']['pose']['position']
            ori = data['pose']['pose']['orientation']

            writer.writerow([
                timestamp,
                pos['x'], pos['y'], pos['z'],
                ori['x'], ori['y'], ori['z'], ori['w']
            ])
        except Exception as e:
            print(f"Skipping a message due to error: {e}")
