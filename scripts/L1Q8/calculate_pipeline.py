import os

# This script was used to calculate the transport times for all nodes in the pipeline communication.

def read_times(filename):
    """Read time values from a file."""
    with open(filename, 'r') as file:
        times = [float(line.strip()) for line in file.readlines()]
    return times


def main():
    # Define base directory
    base_dir = '/home/raoul/Documents/QEES/lab1.3/evaluation'
    publish_dir = os.path.join(base_dir, 'publish_time')
    subscribe_dir = os.path.join(base_dir, 'subscribe_time')
    transport_dir = os.path.join(base_dir, 'transport_time')

    # Ensure the transport_time directory exists
    os.makedirs(transport_dir, exist_ok=True)

    # List of message sizes
    sizes = ['256byte', '512byte', '1Kbyte', '2Kbyte', '4Kbyte', '8Kbyte', '16Kbyte',
             '32Kbyte', '64Kbyte', '128Kbyte', '256Kbyte', '512Kbyte', '1Mbyte',
             '2Mbyte', '4Mbyte']

    # Process each message size
    for size in sizes:
        # Read publish times
        publish_file = os.path.join(publish_dir, f'publish_time_{size}.txt')
        publish_times = read_times(publish_file)

        # Get all the subscribe files for the current size
        subscribe_files = [f for f in os.listdir(subscribe_dir) if f.startswith(f'subscribe_time_{size}_listener')]

        # Process each subscribe file (each listener)
        for subscribe_file in subscribe_files:
            listener_id = subscribe_file.split('_')[-1].replace('.txt', '')
            subscribe_file_path = os.path.join(subscribe_dir, subscribe_file)

            # Read subscribe times
            subscribe_times = read_times(subscribe_file_path)

            # Ignore if the first line is 0, or throw an error if not
            if subscribe_times[0] == 0:
                subscribe_times = subscribe_times[1:]
            else:
                raise ValueError(f"First line in {subscribe_file_path} is not 0!")

            # Ensure the number of sent and received messages match
            if len(publish_times) != len(subscribe_times):
                raise ValueError(f"Mismatch in message count for size {size} and listener {listener_id}: "
                                 f"published {len(publish_times)}, received {len(subscribe_times)}")

            # Calculate transport times (difference between publish and subscribe)
            transport_times = [sub - pub for pub, sub in zip(publish_times, subscribe_times)]

            # Write transport times to the transport_time directory
            transport_file = os.path.join(transport_dir, f'transport_time_{size}_listener_{listener_id}.txt')
            with open(transport_file, 'w') as file:
                for time in transport_times:
                    file.write(f"{time}\n")

            print(f"Processed {size} for listener {listener_id}")


# Run the main function
if __name__ == '__main__':
    main()