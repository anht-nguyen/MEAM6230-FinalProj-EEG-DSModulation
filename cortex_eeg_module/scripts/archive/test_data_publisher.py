import zmq
import time
import random

def test_publisher():
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.bind("tcp://*:5555")
    
    # Wait for subscribers to connect
    print("Waiting for subscribers to connect...")
    time.sleep(2)
    
    while True:
        test_data = {
            'met': [True, random.random(), True, 0.5],  # example test data
            'time': time.time()
        }
        publisher.send_json(test_data)
        print("Test message sent:", test_data)
        time.sleep(1)

if __name__ == '__main__':
    test_publisher()