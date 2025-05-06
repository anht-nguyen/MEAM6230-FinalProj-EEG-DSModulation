import zmq
import matplotlib
matplotlib.use('TkAgg')  # Ensure a GUI backend is used
import matplotlib.pyplot as plt
from collections import deque
import time

# Set up ZeroMQ subscriber
context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:5555")
subscriber.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages

# Prepare storage for plotting:
# One common time deque for the x-axis and 32 deques for the EEG channels.
maxlen = 100
times = deque(maxlen=maxlen)
channel_values = [deque(maxlen=maxlen) for _ in range(32)]  # channels 0-31 correspond to eeg_array[2]...eeg_array[33]

# Set up a figure with 32 subplots arranged in 4 rows and 8 columns
fig, axs = plt.subplots(4, 8, figsize=(16, 8))
lines = []
for ax in axs.flat:
    # Create an empty line on each subplot and store the line object.
    line, = ax.plot([], [], lw=2)
    lines.append(line)
    ax.set_xlabel('Time')
    ax.set_ylabel('Value')
    # Optionally, adjust font size, etc.
    
plt.suptitle('Real-Time EEG Data (32 Channels)')
plt.ion()
plt.show(block=False)
time.sleep(0.5)  # Give time for the plot window to open

print("Real-time EEG plot window opened. Waiting for data...")

# Set up a poller to wait for data
poller = zmq.Poller()
poller.register(subscriber, zmq.POLLIN)

while True:
    try:
        socks = dict(poller.poll(timeout=100))
        if subscriber in socks and socks[subscriber] == zmq.POLLIN:
            data = subscriber.recv_json()
            print("Received data:", data)  # Debug print

            # Use the 'time' field if available; otherwise, use current time.
            current_time = data.get('time', time.time())
            eeg_array = data.get('eeg', [])
            
            # Ensure there are at least 34 elements (indexes 0,1 and channels 2-33)
            if not eeg_array or len(eeg_array) < 34:
                print("Received EEG data does not have enough elements:", eeg_array)
                continue

            # Append the timestamp
            times.append(current_time)
            
            # For each of the 32 channels (indexes 2 to 33)
            for i in range(32):
                try:
                    value = float(eeg_array[i+2])
                except Exception as e:
                    print(f"Error processing channel {i}: {e}")
                    value = 0.0
                channel_values[i].append(value)
            
            # Update each subplot's line with the new data
            for i, line in enumerate(lines):
                line.set_data(list(times), list(channel_values[i]))
            
            # Update axes limits for each subplot
            for ax in axs.flat:
                ax.relim()
                ax.autoscale_view()

            fig.canvas.draw_idle()

        plt.pause(0.2)
    except KeyboardInterrupt:
        print("Exiting...")
        break
