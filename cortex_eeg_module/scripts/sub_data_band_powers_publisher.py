from cortex import Cortex
from config import YOUR_APP_CLIENT_ID, YOUR_APP_CLIENT_SECRET, FS, TIME_WINDOWS
import zmq
import json
import time
import numpy as np
from band_power_helpers import compute_relative_band_powers_all_channels, compute_band_power_ratio_all_channels

class Subcribe():
    """
    A class to subscribe to data streams.
    """
    def __init__(self, app_client_id, app_client_secret, **kwargs):
        print("Subscribe __init__")
        from cortex import Cortex  # Import here to avoid circular dependency if needed
        self.c = Cortex(app_client_id, app_client_secret, debug_mode=True, **kwargs)
        self.c.bind(create_session_done=self.on_create_session_done)
        self.c.bind(new_data_labels=self.on_new_data_labels)
        self.c.bind(new_eeg_data=self.on_new_eeg_data)
        self.c.bind(new_mot_data=self.on_new_mot_data)
        self.c.bind(new_dev_data=self.on_new_dev_data)
        self.c.bind(new_met_data=self.on_new_met_data)
        self.c.bind(new_pow_data=self.on_new_pow_data)
        self.c.bind(inform_error=self.on_inform_error)
        
        # Setup ZeroMQ publisher for data on port 5555
        import zmq
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.bind("tcp://*:5556")
        time.sleep(2)  # allow socket to bind
        
        # Buffer to accumulate EEG data over the time window
        # This buffer will be a list of EEG sample arrays (one per time instance)
        self.eeg_buffer = []

    def start(self, streams, headsetId=''):
        self.streams = streams
        if headsetId != '':
            self.c.set_wanted_headset(headsetId)
        self.c.open()

    def sub(self, streams):
        self.c.sub_request(streams)

    def unsub(self, streams):
        self.c.unsub_request(streams)

    def on_new_data_labels(self, *args, **kwargs):
        data = kwargs.get('data')
        stream_name = data['streamName']
        stream_labels = data['labels']
        print('{} labels are : {}'.format(stream_name, stream_labels))

    def on_new_eeg_data(self, *args, **kwargs):
        """
        Handle EEG data emitted from Cortex. This function accumulates EEG samples until a full time window is
        collected. Then, it computes the band powers for each channel within that time window.
        
        The incoming data is assumed to contain a single time instance's worth of EEG samples for all channels.
        """
        data = kwargs.get('data')
        # Print the raw EEG data for debugging
        # print('eeg data: {}'.format(data))
        
        time_window = TIME_WINDOWS
        fs = FS
        num_samples_per_window = int(time_window * fs)

        # Get the EEG data for this time instance (list of samples for each channel)
        eeg_sample = data.get('eeg', [])
        if not eeg_sample:
            return
        
        # Append the new sample to the buffer.
        # Assuming eeg_sample is a list with length equal to NUM_CHANNELS
        self.eeg_buffer.append(eeg_sample)
        
        # When enough samples are accumulated, process the window
        if len(self.eeg_buffer) >= num_samples_per_window:
            # Convert buffer to a 2D numpy array of shape (NUM_CHANNELS, n_samples)
            # Transpose if necessary: each row should correspond to a channel.
            window_data = np.array(self.eeg_buffer).T  # shape becomes (NUM_CHANNELS, n_samples)
            
            # Compute band powers over the windowed data
            band_powers = compute_relative_band_powers_all_channels(window_data)
            
            # publish the band powers
            self.publish_data(band_powers)
            
            # Clear the buffer for the next time window
            self.eeg_buffer = []

    def on_new_mot_data(self, *args, **kwargs):
        pass

    def on_new_dev_data(self, *args, **kwargs):
        data = kwargs.get('data')
        print('dev data: {}'.format(data))

    def on_new_met_data(self, *args, **kwargs):
        pass

    def on_new_pow_data(self, *args, **kwargs):
        pass

    def on_create_session_done(self, *args, **kwargs):
        print('on_create_session_done')
        self.sub(self.streams)

    def on_inform_error(self, *args, **kwargs):
        error_data = kwargs.get('error_data')
        print("Error:", error_data)

    def publish_data(self, data):
        """
        Publish data via ZeroMQ.
        """
        if data:
            try:
                self.publisher.send_json(data)
                print("Published data:", data)
            except Exception as e:
                print("Failed to publish data:", e)

def main():
    your_app_client_id = YOUR_APP_CLIENT_ID
    your_app_client_secret = YOUR_APP_CLIENT_SECRET

    s = Subcribe(your_app_client_id, your_app_client_secret)
    streams = ['eeg']
    s.start(streams)

    # Keep the script running
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting...")

if __name__ == '__main__':
    main()
