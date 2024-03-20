import soundfile as sf
import numpy as np

# Read the data
with open("output_w2.txt") as f:
    lines = f.readlines()
    data = []
    for line in lines:
        try:
            line_arr = line.strip().split()
            data.append(float(line_arr[2])) # Our data was in the second column
        except (IndexError, ValueError):
            continue # Probably just not a line corresponding to real data

# Normalize
data = np.asarray(data)
data = data / np.max(np.abs(data))

# Extend the data to 1 second
desired_length = 48000  # 1 second at 48000 Hz sample rate
current_length = len(data)
remainder = desired_length - current_length
if remainder > 0:
    extended_data = np.concatenate((data, np.zeros(remainder)))
else:
    extended_data = data[:desired_length]  # Truncate if the data is already longer than 1 second

# Convert to .wav file
sf.write("drum_w2_extended.wav", extended_data, 48000)
print("DONE!")
