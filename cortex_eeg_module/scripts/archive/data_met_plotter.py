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

# Prepare storage for plotting
times = deque(maxlen=100)
values = deque(maxlen=100)

# Set up real-time plot
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_xlabel('Time')
ax.set_ylabel('Metric Value')
ax.set_title('Real-Time Performance Metric')
plt.show(block=False)
time.sleep(0.5)  # Give time for the plot window to open

print("Real-time plot window opened. Waiting for data...")

# Set up a poller to wait for data
poller = zmq.Poller()
poller.register(subscriber, zmq.POLLIN)

while True:
    try:
        socks = dict(poller.poll(timeout=100))
        if subscriber in socks and socks[subscriber] == zmq.POLLIN:
            data = subscriber.recv_json()
            print("Received data:", data)  # Debug print

            # Use the 'time' field if available; otherwise, current time
            current_time = data.get('time', time.time())
            met_array = data.get('met', [])

            if not met_array or len(met_array) < 2:
                print("Received met data does not have enough elements:", met_array)
                continue

            try:
                # Use the second element as the metric value (as an example)
                value = float(met_array[1])
            except Exception as e:
                print("Error processing data:", e)
                continue

            times.append(current_time)
            values.append(value)
            line.set_data(list(times), list(values))
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw_idle()

        plt.pause(0.5)
    except KeyboardInterrupt:
        print("Exiting...")
        break