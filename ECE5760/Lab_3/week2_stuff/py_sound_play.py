#!/usr/bin/env python3
# =========================================================================
# parse.py
# =========================================================================
# Code for turning ModelSim output into a .wav sound file
#
# Author: Aidan McNay
# Date: March 18th, 2024

import soundfile as sf
import numpy as np

# ------------------------------------------------------------------------
# Read the data
# ------------------------------------------------------------------------

with open("output_w2_nonlinear.txt") as f:
    lines = f.readlines()
    data = []
    for line in lines:
        try:
            line_arr = line.strip().split()
            data.append(float(line_arr[2])) # Our data was in the second column
        except (IndexError, ValueError):
            continue # Probably just not a line corresponding to real data

# ------------------------------------------------------------------------
# Normalize
# ------------------------------------------------------------------------

data = np.asarray(data)
data = data / np.max(np.abs(data))

# ------------------------------------------------------------------------
# Convert to .wav file
# ------------------------------------------------------------------------

sf.write("drum_nonlinear.wav", data, 48000)
print("DONE !")
