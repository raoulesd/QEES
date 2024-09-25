import os

def calculate_transport_times(base_dir, sizes):
    """Calculate transport times and handle message loss or discrepancy between sent and received messages."""
    publish_dir = os.path.join(base_dir, 'publish_time')
    subscribe_dir = os.path.join(base_dir, 'subscribe_time')

    for size in sizes:
        publish_file = os.path.join(publish_dir, f'publish_time_{size}.txt')

        # Read publish times
        with open(publish_file, 'r') as pub_file:
            publish_times = [float(line.strip()) for line in pub_file]
        num_sent = len(publish_times)

        # Iterate through all listener files in subscribe directory for the current size
        for file_name in os.listdir(subscribe_dir):
            if file_name.startswith(f'subscribe_time_{size}_listener_'):
                listener_num = file_name.split('listener_')[1].split('.txt')[0]
                subscribe_file = os.path.join(subscribe_dir, file_name)

                # Read subscribe times
                with open(subscribe_file, 'r') as sub_file:
                    lines = sub_file.readlines()
                    num_lost = int(lines[0].strip())  # The first line is the number of lost messages
                    subscribe_times = [float(line.strip()) for line in lines[1:]]  # Remaining lines are received times
                    num_received = len(subscribe_times)

                    # Print file if messages were lost
                    if num_lost != 0:
                        print(f"Messages lost in file {subscribe_file}: {num_lost} messages lost.")

                    # Check if the number of received messages matches the number of sent messages
                    if num_received != num_sent:
                        raise Exception(
                            f"Mismatch in messages for listener {listener_num} with size {size}. "
                            f"Sent: {num_sent}, Received: {num_received}."
                        )

                    # Calculate transport times
                    transport_times = [subscribe_times[i] - publish_times[i] for i in range(num_received)]

                    output_file = os.path.join(base_dir, 'transport_time',
                                               f'transport_time_{size}_listener_{listener_num}.txt')

                    with open(output_file, 'w') as file:
                        for time in transport_times:
                            file.write(f"{time}\n")

                    print(f"Transport times for {size}, listener {listener_num} written to {output_file}")


def main():
    base_dir = '/home/raoul/Documents/QEES/lab1.1/evaluation'
    sizes = ['256byte', '512byte', '1Kbyte', '2Kbyte', '4Kbyte', '8Kbyte', '16Kbyte', '32Kbyte',
             '64Kbyte', '128Kbyte', '256Kbyte', '512Kbyte', '1Mbyte', '2Mbyte', '4Mbyte']

    calculate_transport_times(base_dir, sizes)

if __name__ == "__main__":
    main()