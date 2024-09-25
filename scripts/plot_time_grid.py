from plot_times import read_transport_times
import os
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import math

def get_grid_layout(num_listeners):
    """Return the number of columns and rows for the FacetGrid based on the number of listeners."""
    if num_listeners <= 4:
        return num_listeners, 1
    else:
        rows = 2
        cols = math.ceil(num_listeners / rows)
        return cols, rows

def main():
    base_dir = '/home/raoul/Documents/QEES/lab1.1/evaluation'
    sizes = ['256byte', '512byte', '1Kbyte', '2Kbyte', '4Kbyte', '8Kbyte', '16Kbyte', '32Kbyte',
             '64Kbyte', '128Kbyte', '256Kbyte', '512Kbyte', '1Mbyte', '2Mbyte', '4Mbyte']

    df = read_transport_times(base_dir, sizes)

    # Modify size labels in the main function
    df['Size'] = df['Size'].str.replace('byte', 'b')

    size_order = ['256b', '512b', '1Kb', '2Kb', '4Kb', '8Kb', '16Kb', '32Kb',
                  '64Kb', '128Kb', '256Kb', '512Kb', '1Mb', '2Mb', '4Mb']

    df['Size'] = pd.Categorical(df['Size'], categories=size_order, ordered=True)

    # Determine number of listeners
    num_listeners = df['Listener'].nunique()

    # Get the grid layout based on the number of listeners
    cols, rows = get_grid_layout(num_listeners)

    plt.figure(figsize=(cols * 5, rows * 4))  # Adjust size as needed

    # Create a FacetGrid to have one subplot per listener
    g = sns.FacetGrid(df, col="Listener", col_wrap=cols, height=4, sharey=True)
    g.map(sns.boxplot, "Size", "Transport Time (s)", order=size_order)

    # Add red average line to each subplot
    for ax, listener_num in zip(g.axes.flat, sorted(df['Listener'].unique())):
        # Calculate average latency for each listener
        listener_df = df[df['Listener'] == listener_num]
        avg_latency = listener_df['Transport Time (s)'].mean()

        # Draw the red line for the average latency
        ax.axhline(avg_latency, color='red', linestyle='--', label=f'Avg: {avg_latency:.4f}s')

        # Optionally add a legend showing the average latency
        ax.legend(loc='upper right')

    # Rotate x-tick labels for clarity and reduce their size
    for ax in g.axes.flat:
        ticks = ax.get_xticks()
        ax.set_xticks(ticks)
        ax.set_xticklabels(ax.get_xticklabels(), rotation=45, ha='center', fontsize=10)

    plt.tight_layout()

    # Save the figure to a file
    output_path = os.path.join('../', 'facetgrid_transport_times_' + str(num_listeners) + '.png')
    plt.savefig(output_path)

    plt.show()


if __name__ == "__main__":
    main()