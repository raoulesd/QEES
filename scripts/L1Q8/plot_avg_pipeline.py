import os
import matplotlib.pyplot as plt
import numpy as np

# This script was used to calculate the adjusted transport times (i.e. latency from Node X-1 to Node X instead of Node 1 to Node X).
# This data is averaged and plotted using a line graph.


def load_transport_times(size, listener_ids, transport_dir):
    """Load transport times for a given size and listener IDs."""
    transport_times = {}
    for listener_id in listener_ids:
        transport_file = os.path.join(transport_dir, f'transport_time_{size}_listener_{listener_id}.txt')
        if os.path.exists(transport_file):  # Check if the file exists
            with open(transport_file, 'r') as file:
                times = [float(line.strip()) for line in file.readlines()]
                transport_times[listener_id] = times
    return transport_times

def calculate_adjusted_transport_times(transport_data):
    """Calculate adjusted transport times based on specified rules."""
    adjusted_times = {}

    for size in transport_data.keys():
        adjusted_times[size] = {}

        # Transport time for listener B
        adjusted_times[size]['B'] = transport_data[size]['B']

        # Transport time for listener C
        adjusted_times[size]['C'] = [c - b for b, c in zip(adjusted_times[size]['B'], transport_data[size]['C'])]

        # Transport time for listener D
        adjusted_times[size]['D'] = [d - c for c, d in zip(adjusted_times[size]['C'], transport_data[size]['D'])]

        # Transport time for listener E
        adjusted_times[size]['E'] = [e - d for d, e in zip(adjusted_times[size]['D'], transport_data[size]['E'])]

    return adjusted_times

def main():
    # Define base directory and transport directory
    base_dir = '/home/raoul/Documents/QEES/lab1.3/evaluation'
    transport_dir = os.path.join(base_dir, 'transport_time')

    # List of message sizes
    sizes = ['256byte', '512byte', '1Kbyte', '2Kbyte', '4Kbyte', '8Kbyte', '16Kbyte',
             '32Kbyte', '64Kbyte', '128Kbyte', '256Kbyte', '512Kbyte', '1Mbyte',
             '2Mbyte', '4Mbyte']

    # Strip the 'yte' suffix from each size label
    stripped_sizes = [size.replace('yte', '') for size in sizes]

    # Define listener IDs
    listener_ids = ['B', 'C', 'D', 'E']  # Adjust this based on your listeners

    # Load transport times
    transport_data = {}
    for size in sizes:
        transport_data[size] = load_transport_times(size, listener_ids, transport_dir)

    # Calculate adjusted transport times
    adjusted_transport_times = calculate_adjusted_transport_times(transport_data)

    # Compute averages for each listener for each size
    avg_transport_times = {size: {} for size in sizes}

    for size in sizes:
        for listener_id in listener_ids:
            avg_transport_times[size][listener_id] = np.mean(adjusted_transport_times[size][listener_id])

    # Prepare data for plotting
    listener_colors = {'B': 'r', 'C': 'g', 'D': 'b', 'E': 'm'}  # Assigning colors to each listener
    plt.figure(figsize=(10, 6))

    for listener_id in listener_ids:
        avg_times = [avg_transport_times[size][listener_id] for size in sizes]
        plt.plot(stripped_sizes, avg_times, marker='o', color=listener_colors[listener_id], label=f'{chr(ord(listener_id)-1)} -> {listener_id}')

    # Customize the graph
    plt.xlabel('Message Size', fontsize=12)
    plt.ylabel('Average Transport Time (s)', fontsize=12)
    plt.title('Average Transport Time per Message Size and Listener', fontsize=14, fontweight='bold')
    plt.xticks(rotation=45, ha='right')
    plt.legend(title='Nodes')

    # Adjust layout for better appearance
    plt.tight_layout()

    output_path = os.path.join('../', 'avg_pipeline_plot.png')
    plt.savefig(output_path)

    # Show the plot
    plt.show()

# Run the main function
if __name__ == '__main__':
    main()
