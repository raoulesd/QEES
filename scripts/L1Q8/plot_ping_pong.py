import os
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import math
import numpy as np

# In this script, the one-way and round-trip times of the ping-pong communication are calculated.
# Both transport times are shown in an individual graph using box plots, and an average line graph is plotted after this.

def read_transport_and_round_trip_times(base_dir, sizes):
    """Calculate transport and round-trip times from publish, subscribe, and ack files."""
    data = {'Size': [], 'Transport Time (s)': [], 'Round Trip Time (s)': [], 'Listener': []}

    for size in sizes:
        publish_dir = os.path.join(base_dir, 'publish_time')
        subscribe_dir = os.path.join(base_dir, 'subscribe_time')
        ack_dir = os.path.join(base_dir, 'ack_time')

        for file_name in os.listdir(publish_dir):
            if file_name.startswith(f'publish_time_{size}'):
                # Extract listener number from the file name
                listener_num = 1  # Adjust as needed

                publish_file_path = os.path.join(publish_dir, file_name)
                subscribe_file_path = os.path.join(subscribe_dir, file_name.replace('publish_time', 'subscribe_time'))
                ack_file_path = os.path.join(ack_dir, file_name.replace('publish_time', 'ack_time'))

                # Read publish, subscribe, and ack times
                with open(publish_file_path, 'r') as publish_file, open(subscribe_file_path,
                                                                        'r') as subscribe_file, open(ack_file_path,
                                                                                                     'r') as ack_file:
                    publish_times = [float(line.strip()) for line in publish_file][:-1]
                    subscribe_times = [float(line.strip()) for line in subscribe_file][1:-1]
                    ack_times = [float(line.strip()) for line in ack_file][:-1]

                    # Calculate transport times (subscribe - publish) and round-trip times (ack - publish)
                    for pub_time, sub_time, ack_time in zip(publish_times, subscribe_times, ack_times):
                        transport_time = sub_time - pub_time
                        round_trip_time = ack_time - sub_time

                        data['Size'].append(size)
                        data['Transport Time (s)'].append(transport_time)
                        data['Round Trip Time (s)'].append(round_trip_time)
                        data['Listener'].append(listener_num)

    return pd.DataFrame(data)

def get_grid_layout(num_listeners):
    num_listeners += 1
    return 1,1


def main():
    base_dir = '/home/raoul/Documents/QEES/lab1.2/evaluation'
    sizes = ['256byte', '512byte', '1Kbyte', '2Kbyte', '4Kbyte', '8Kbyte', '16Kbyte', '32Kbyte',
             '64Kbyte', '128Kbyte', '256Kbyte', '512Kbyte', '1Mbyte', '2Mbyte', '4Mbyte']

    df = read_transport_and_round_trip_times(base_dir, sizes)

    print(df.to_string())

    # Modify size labels
    df['Size'] = df['Size'].str.replace('byte', 'b')

    size_order = ['256b', '512b', '1Kb', '2Kb', '4Kb', '8Kb', '16Kb', '32Kb',
                  '64Kb', '128Kb', '256Kb', '512Kb', '1Mb', '2Mb', '4Mb']

    df['Size'] = pd.Categorical(df['Size'], categories=size_order, ordered=True)

    # Determine number of listeners
    num_listeners = df['Listener'].nunique()

    # Get the grid layout based on the number of listeners
    cols, rows = get_grid_layout(num_listeners)

    # Set up the subplots: 1 row, 2 columns
    fig, axes = plt.subplots(1, 2, figsize=(num_listeners * 10, 6), sharey=True)

    # Plot Transport Time on the first axis
    sns.boxplot(x='Size', y='Transport Time (s)', data=df, order=size_order, ax=axes[0], color='skyblue')
    axes[0].set_title('One-Way Time')
    axes[0].set_xlabel('Message Size')

    # Add average line for Transport Time
    avg_transport = df['Transport Time (s)'].mean()
    axes[0].axhline(avg_transport, color='red', linestyle='--', label=f'Avg. One-Way: {avg_transport:.4f}s')
    axes[0].legend(loc='upper right')

    # Plot Round Trip Time on the second axis
    sns.boxplot(x='Size', y='Round Trip Time (s)', data=df, order=size_order, ax=axes[1], color='lightgreen')
    axes[1].set_title('Round-Trip Time')
    axes[1].set_xlabel('Message Size')

    # Add average line for Round Trip Time
    avg_round_trip = df['Round Trip Time (s)'].mean()
    axes[1].axhline(avg_round_trip, color='blue', linestyle='--', label=f'Avg. Round-Trip: {avg_round_trip:.4f}s')
    axes[1].legend(loc='upper right')

    # Rotate x-tick labels for clarity
    for ax in axes:
        ticks = ax.get_xticks()  # Get current tick positions
        ax.set_xticks(ticks)  # Set ticks at the same positions
        ax.set_xticklabels(ax.get_xticklabels(), rotation=45, ha='center')  # Set labels with rotation

        # Reduce Y-axis step size
        y_min, y_max = ax.get_ylim()
        ax.set_yticks([round(y, 2) for y in np.arange(y_min, y_max, (y_max - y_min) / 10)])

    # Adjust the layout
    plt.tight_layout()

    # Save the figure to a file
    output_path = os.path.join('../', 'transport_and_round_trip_times.png')
    plt.savefig(output_path)

    # Show the figure
    plt.show()

    # Now create a separate figure for the average graph
    # Calculate averages for each size
    avg_df = df.groupby('Size').mean().reset_index()

    # Set up a new figure for the average plot
    fig_avg, ax_avg = plt.subplots(figsize=(num_listeners * 10, 6))  # Adjust figure size as needed

    # Plot average Transport and Round Trip Time per size
    ax_avg.plot(avg_df['Size'], avg_df['Transport Time (s)'], label='Avg. One-Way', marker='o', color='skyblue')
    ax_avg.plot(avg_df['Size'], avg_df['Round Trip Time (s)'], label='Avg. Round-Trip', marker='o',
                color='lightgreen')
    ax_avg.set_title('Average Times per Message Size')
    ax_avg.legend(loc='upper right')

    # Set the tick positions and labels explicitly
    ax_avg.set_xticks(range(len(avg_df['Size'])))  # Set the positions
    ax_avg.set_xticklabels(avg_df['Size'], rotation=45, ha='center')  # Set the labels with rotation

    # Add the X and Y axis labels
    ax_avg.set_xlabel('Message Size')
    ax_avg.set_ylabel('Time (s)')

    # Adjust the layout
    plt.tight_layout()

    # Save the average time figure to a separate file
    output_path_avg = os.path.join('../', 'transport_and_round_trip_averages.png')
    plt.savefig(output_path_avg)

    # Show the average figure
    plt.show()


if __name__ == "__main__":
    main()
