import os
import matplotlib.pyplot as plt
import pandas as pd

# This script was used to calculate the adjusted transport times (i.e. latency from Node X-1 to Node X instead of Node 1 to Node X).
# This data is plotted together in one large graph using box plots.


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

    # Prepare data for plotting
    plot_data = []
    x_labels = []
    size_positions = []  # Store the x-axis positions of the size groupings

    for i, size in enumerate(sizes):
        for listener_id in listener_ids:
            plot_data.append(adjusted_transport_times[size][listener_id])
            x_labels.append(listener_id)
        # Position for the size label (centered within the group)
        size_positions.append(len(x_labels) - len(listener_ids) / 2)

    # Create a boxplot with a larger figure size and adjusted margins
    plt.figure(figsize=(14, 7))  # Increase figure size
    boxplot = plt.boxplot(plot_data, labels=x_labels, patch_artist=True)

    # Customize X-axis
    plt.xticks(rotation=45, ha='right')

    # Shift listener labels by half an interval to the right
    tick_positions = [i + 0.5 for i in range(1, len(x_labels) + 1)]
    plt.gca().set_xticks(tick_positions)  # Adjust tick positions to shift labels

    # Move size labels to the top of the graph
    for pos, size in zip(size_positions, stripped_sizes):
        plt.text(pos, plt.ylim()[1] + (plt.ylim()[1] - plt.ylim()[0]) * 0.05 - 0.002, size, ha='center', va='bottom',
                 fontsize=10, fontweight='bold')

    # Draw vertical lines to separate groups by size
    for pos in size_positions[:-1]:  # Avoid drawing a line after the last size
        plt.axvline(x=pos + len(listener_ids) / 2 + 0.5, color='grey', linestyle='--')

    # Set labels and title
    plt.xlabel('Listener ID and Message Size')
    plt.ylabel('Transport Time (s)')

    # Move title above the size labels
    plt.suptitle('Adjusted Transport Time (Latency) per Message Size and Listener', fontsize=14, fontweight='bold',
                 y=0.92)

    # Adjust the top margin for the size labels
    plt.subplots_adjust(bottom=0.2, top=0.85)  # Adjust margins for top labels


    output_path = os.path.join('../', 'inter_pipeline_plot.png')
    plt.savefig(output_path)

    # Show the plot
    plt.show()



# Run the main function
if __name__ == '__main__':
    main()
