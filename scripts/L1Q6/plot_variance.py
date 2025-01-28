import os
import numpy as np
import matplotlib.pyplot as plt

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

    # Initialize storage for variances
    variances = {scenario: [] for scenario in scenario_dirs}

    # Load data and compute variances
    for size in sizes:
        transport_data = load_transport_times(size, scenario_dirs)
        for scenario, times in transport_data.items():
            if times:  # Ensure there is data to process
                var = np.var(times, ddof=1)  # Sample variance
                variances[scenario].append(var)
            else:
                variances[scenario].append(np.nan)

    # Plotting
    plt.figure(figsize=(8, 5))
    for scenario, var_values in variances.items():
        plt.plot(stripped_sizes, var_values, marker='o', label=scenario)

    plt.xlabel('Message Size')
    plt.ylabel('Variance of Transport Time (s²)')
    plt.title('Variance of Transport Time per Scenario and Message Size')
    plt.legend()
    plt.xticks(rotation=45, ha='right')
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(os.path.join('variance_transport_times.png'))

    plt.show()

if __name__ == '__main__':
    main()
