import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

def load_transport_times(size, scenario_dirs):
    """Load transport times for a given size from each scenario directory."""
    transport_times = {}
    for scenario_name, scenario_dir in scenario_dirs.items():
        transport_file = os.path.join(scenario_dir, f'transport_time_{size}.txt')
        if os.path.exists(transport_file):
            with open(transport_file, 'r') as file:
                times = [float(line.strip()) for line in file.readlines()]
                transport_times[scenario_name] = times
    return transport_times

def calculate_mean_and_ci(data, confidence=0.80):
    """Calculate mean and confidence interval for a list of data points."""
    n = len(data)
    mean = np.mean(data)
    sem = stats.sem(data)  # Standard error of the mean
    h = sem * stats.t.ppf((1 + confidence) / 2., n-1)  # Margin of error
    return mean, h

def main():
    # Define base directory and scenario directories
    base_dir = 'M:/Github/QEES/lab1.4/evaluation'  # Update this path accordingly
    scenario_dirs = {
        'Clean QoS 0': os.path.join(base_dir, 'tt_clean_qos_0'),
        'Clean QoS 1': os.path.join(base_dir, 'tt_clean_qos_1'),
        'Load QoS 0': os.path.join(base_dir, 'tt_load_qos_0'),
        'Load QoS 1': os.path.join(base_dir, 'tt_load_qos_1')
    }

    # List of message sizes
    sizes = ['256byte', '512byte', '1Kbyte', '2Kbyte', '4Kbyte', '8Kbyte', '16Kbyte',
             '32Kbyte', '64Kbyte', '128Kbyte', '256Kbyte', '512Kbyte', '1Mbyte',
             '2Mbyte', '4Mbyte']

    # Strip the 'yte' suffix from each size label for plotting
    stripped_sizes = [size.replace('yte', '') for size in sizes]

    # Initialize storage for means and confidence intervals
    means = {scenario: [] for scenario in scenario_dirs}
    conf_intervals = {scenario: [] for scenario in scenario_dirs}

    # Load data and compute statistics
    for size in sizes:
        transport_data = load_transport_times(size, scenario_dirs)
        for scenario, times in transport_data.items():
            if times:  # Ensure there is data to process
                mean, ci = calculate_mean_and_ci(times)
                means[scenario].append(mean)
                conf_intervals[scenario].append(ci)
            else:
                means[scenario].append(np.nan)
                conf_intervals[scenario].append(np.nan)

    # Plotting
    plt.figure(figsize=(8, 5))
    for scenario in scenario_dirs:
        plt.errorbar(stripped_sizes, means[scenario], yerr=conf_intervals[scenario], label=scenario, capsize=5, marker='o')

    plt.xlabel('Message Size')
    plt.ylabel('Average Transport Time (s)')
    plt.title('Average Transport Time with 80% Confidence Intervals')
    plt.legend()
    plt.xticks(rotation=45)
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(os.path.join('average_transport_times.png'))

    plt.show()

if __name__ == '__main__':
    main()
