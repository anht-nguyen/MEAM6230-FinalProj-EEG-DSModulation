from cortex import Cortex
from config import YOUR_APP_CLIENT_ID, YOUR_APP_CLIENT_SECRET
import zmq
import json
import time

class Subcribe():
    """
    A class to subscribe to data streams.
    """
    def __init__(self, app_client_id, app_client_secret, **kwargs):
        print("Subscribe __init__")
        self.c = Cortex(app_client_id, app_client_secret, debug_mode=True, **kwargs)
        self.c.bind(create_session_done=self.on_create_session_done)
        self.c.bind(new_data_labels=self.on_new_data_labels)
        self.c.bind(new_eeg_data=self.on_new_eeg_data)
        self.c.bind(new_mot_data=self.on_new_mot_data)
        self.c.bind(new_dev_data=self.on_new_dev_data)
        self.c.bind(new_met_data=self.on_new_met_data)
        self.c.bind(new_pow_data=self.on_new_pow_data)
        self.c.bind(inform_error=self.on_inform_error)
        
        # Setup ZeroMQ publisher for met data on port 5555
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.bind("tcp://*:5555")
        time.sleep(2)  # allow socket to bind

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
        data = kwargs.get('data')
        print('eeg data: {}'.format(data))
        # Publish the data as JSON
        try:
            self.publisher.send_json(data)
        except Exception as e:
            print("Error publishing data:", e)

    def on_new_mot_data(self, *args, **kwargs):
        data = kwargs.get('data')
        print('motion data: {}'.format(data))

    def on_new_dev_data(self, *args, **kwargs):
        data = kwargs.get('data')
        print('dev data: {}'.format(data))

    def on_new_met_data(self, *args, **kwargs):
        """
        Handle performance metric data and publish it via ZeroMQ.
        Expected data format (example):
          {'met': [True, 0.5, True, 0.5, ...], 'time': 1627459390.4229}
        """
        data = kwargs.get('data')
        print('pm data: {}'.format(data))



    def on_new_pow_data(self, *args, **kwargs):
        data = kwargs.get('data')
        print('pow data: {}'.format(data))


    def on_create_session_done(self, *args, **kwargs):
        print('on_create_session_done')
        self.sub(self.streams)

    def on_inform_error(self, *args, **kwargs):
        error_data = kwargs.get('error_data')
        print("Error:", error_data)


def main():
    your_app_client_id = YOUR_APP_CLIENT_ID
    your_app_client_secret = YOUR_APP_CLIENT_SECRET

    s = Subcribe(your_app_client_id, your_app_client_secret)
    streams = ['eeg', 'mot', 'met', 'pow']
    s.start(streams)

    # Keep the script running
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting...")

if __name__ == '__main__':
    main()
