import os
import pandas as pd
import matplotlib.pyplot as plt


def read_transport_times(base_dir, sizes):
    """Read transport times from files and return a DataFrame with listener numbers included."""
    data = {'Size': [], 'Transport Time (s)': [], 'Listener': []}

    for size in sizes:
        transport_dir = os.path.join(base_dir, 'transport_time')
        for file_name in os.listdir(transport_dir):
            if file_name.startswith(f'transport_time_{size}'):
                # Extract listener number from the file name
                listener_num = int(file_name.split('listener_')[1].split('.txt')[0])

                file_path = os.path.join(transport_dir, file_name)
                with open(file_path, 'r') as file:
                    times = [float(line.strip()) for line in file]
                    for time in times:
                        data['Size'].append(size)
                        data['Transport Time (s)'].append(time)
                        data['Listener'].append(listener_num)

    return pd.DataFrame(data)


def main():
    base_dir = '/home/raoul/Documents/QEES/lab1.1/evaluation'
    sizes = ['256byte', '512byte', '1Kbyte', '2Kbyte', '4Kbyte', '8Kbyte', '16Kbyte', '32Kbyte',
             '64Kbyte', '128Kbyte', '256Kbyte', '512Kbyte', '1Mbyte', '2Mbyte', '4Mbyte']

    df = read_transport_times(base_dir, sizes)

    # Ensure 'Size' is treated as a categorical variable with the specified order
    size_order = ['256byte', '512byte', '1Kbyte', '2Kbyte', '4Kbyte', '8Kbyte', '16Kbyte', '32Kbyte',
                  '64Kbyte', '128Kbyte', '256Kbyte', '512Kbyte', '1Mbyte', '2Mbyte', '4Mbyte']
    df['Size'] = pd.Categorical(df['Size'], categories=size_order, ordered=True)

    plt.figure(figsize=(14, 7))

    # Create boxplot with separate boxes for each listener
    df.boxplot(column='Transport Time (s)', by=['Size', 'Listener'], grid=False)

    plt.title('Transport Times by Message Size and Listener')
    plt.suptitle('')
    plt.xlabel('Message Size and Listener')
    plt.ylabel('Latency (seconds)')

    # Improve x-axis labels
    plt.xticks(rotation=45, ha='right')

    plt.tight_layout()
    plt.savefig(os.path.join('../', 'boxplot_transport_times.png'))
    plt.show()


if __name__ == "__main__":
    main()
