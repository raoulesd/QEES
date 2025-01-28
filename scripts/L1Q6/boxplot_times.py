import os
import matplotlib.pyplot as plt
import pandas as pd

def load_transport_times(size, scenario_dirs):
    """Load transport times for a given size from each scenario directory."""
    transport_times = {}
    for scenario_name, scenario_dir in scenario_dirs.items():
        transport_file = os.path.join(scenario_dir, f'transport_time_{size}.txt')
        if os.path.exists(transport_file):  # Check if the file exists
            with open(transport_file, 'r') as file:
                times = [float(line.strip()) for line in file.readlines()]
                transport_times[scenario_name] = times
    return transport_times

def main():
    # Define base directory and scenario directories
    base_dir = 'M:/Github/QEES/lab1.4/evaluation'  # Update this path accordingly
    scenario_dirs = {
        'CQ0': os.path.join(base_dir, 'tt_clean_qos_0'),
        'CQ1': os.path.join(base_dir, 'tt_clean_qos_1'),
        'LQ0': os.path.join(base_dir, 'tt_load_qos_0'),
        'LQ1': os.path.join(base_dir, 'tt_load_qos_1')
    }

    # List of message sizes
    sizes = ['256byte', '512byte', '1Kbyte', '2Kbyte', '4Kbyte', '8Kbyte', '16Kbyte',
             '32Kbyte', '64Kbyte', '128Kbyte', '256Kbyte', '512Kbyte', '1Mbyte',
             '2Mbyte', '4Mbyte']

    # Strip the 'yte' suffix from each size label
    stripped_sizes = [size.replace('yte', '') for size in sizes]

    # Load transport times
    transport_data = {}
    for size in sizes:
        transport_data[size] = load_transport_times(size, scenario_dirs)

    # Prepare data for plotting
    plot_data = []
    x_labels = []
    size_positions = []  # Store the x-axis positions of the size groupings

    for i, size in enumerate(sizes):
        for scenario_name in scenario_dirs.keys():
            if scenario_name in transport_data[size]:
                plot_data.append(transport_data[size][scenario_name])
                x_labels.append(scenario_name)
        # Position for the size label (centered within the group)
        size_positions.append(len(x_labels) - len(scenario_dirs) / 2)

    # Create a boxplot with a larger figure size and adjusted margins
    plt.figure(figsize=(14, 7))  # Increase figure size
    boxplot = plt.boxplot(plot_data, labels=x_labels, patch_artist=True)

    # Customize X-axis
    plt.xticks(rotation=45, ha='right')

    # Shift scenario labels by half an interval to the right
    tick_positions = [i + 0.5 for i in range(1, len(x_labels) + 1)]
    plt.gca().set_xticks(tick_positions)  # Adjust tick positions to shift labels

    # Move size labels to the top of the graph
    for pos, size in zip(size_positions, stripped_sizes):
        plt.text(pos + 0.6, plt.ylim()[1] + (plt.ylim()[1] - plt.ylim()[0]) * 0.009 - 0.002, size, ha='center', va='bottom',
                 fontsize=10, fontweight='bold')

    # Draw vertical lines to separate groups by size
    for pos in size_positions[:-1]:  # Avoid drawing a line after the last size
        plt.axvline(x=pos + len(scenario_dirs) / 2 + 0.5, color='grey', linestyle='--')

    # Set labels and title
    plt.xlabel('Scenario and Message Size')
    plt.ylabel('Transport Time (s)')

    # Move title above the size labels
    plt.suptitle('Transport Time per Message Size and Scenario', fontsize=14, fontweight='bold', y=0.92)

    # Adjust the top margin for the size labels
    plt.subplots_adjust(bottom=0.2, top=0.85)  # Adjust margins for top labels

    output_path = os.path.join('scenario_boxplots.png')
    plt.savefig(output_path)

    # Show the plot
    plt.show()

if __name__ == '__main__':
    main()
