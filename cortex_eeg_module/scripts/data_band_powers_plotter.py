#!/usr/bin/env python3
"""
data_band_powers_plotter.py

Subscribe to relative band powers via ZeroMQ, plot real-time band powers for each EEG channel.
Each subplot row represents one channel; each of the three columns corresponds to theta, alpha, beta.
The y-axis label for each subplot shows the channel name.
"""

import zmq
import matplotlib
matplotlib.use('TkAgg')  # Ensure a GUI backend is used
import matplotlib.pyplot as plt
from collections import deque
import time
from config import FLEX_CHANNEL_LABELS

# Prepare storage for plotting:
maxlen = 100
times = deque(maxlen=maxlen)
# One deque per channel-band (channels × 3 bands)
channel_band_powers = [deque(maxlen=maxlen) for _ in range(len(FLEX_CHANNEL_LABELS) * 3)]

# Set up a figure with one row per channel and three columns (theta, alpha, beta)
num_channels = len(FLEX_CHANNEL_LABELS)
num_bands = 3
fig, axs = plt.subplots(num_channels, num_bands, figsize=(16, 8), sharex='col')
lines = []
for idx, ax in enumerate(axs.flat):
    channel_idx = idx // num_bands
    channel_name = FLEX_CHANNEL_LABELS[channel_idx]
    # Create an empty line on each subplot and store the line object.
    line, = ax.plot([], [], lw=2)
    lines.append(line)
    ax.set_xlabel('Time')
    ax.set_ylabel(channel_name)

plt.suptitle('Real-time Relative Band Power (Channels × theta/alpha/beta)')
plt.ion()
plt.show(block=False)
time.sleep(0.5)  # Give time for the plot window to open

print("Real-time EEG plot window opened. Waiting for data...")

# ZeroMQ subscriber setup
context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:5556")
subscriber.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages

# Poller to wait for data
poller = zmq.Poller()
poller.register(subscriber, zmq.POLLIN)

while True:
    try:
        socks = dict(poller.poll(timeout=100))
        if subscriber in socks and socks[subscriber] == zmq.POLLIN:
            data = subscriber.recv_json()
            current_time = data.get('time', time.time())
            pow_array = data.get('pow', [])

            # Append the timestamp
            times.append(current_time)

            # Append new band-power values for each channel-band
            for i, val in enumerate(pow_array):
                try:
                    value = float(val)
                except Exception as e:
                    print(f"Error processing channel-band {i}: {e}")
                    value = 0.0
                channel_band_powers[i].append(value)

            # Update each subplot's line with the new data
            for i, line in enumerate(lines):
                line.set_data(list(times), list(channel_band_powers[i]))

            # Rescale axes
            for ax in axs.flat:
                ax.relim()
                ax.autoscale_view()

            fig.canvas.draw_idle()

        plt.pause(0.2)
    except KeyboardInterrupt:
        print("Exiting...")
        break
