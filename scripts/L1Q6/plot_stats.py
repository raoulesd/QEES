import os
import numpy as np
from scipy import stats
import pandas as pd

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

def calc_stats(data, confidence=0.80):
    n = len(data)
    mean = np.mean(data)
    variance = np.var(data, ddof=1) # sample data so divide by n-1
    sem = stats.sem(data)
    h = sem * stats.t.ppf((1 + confidence) / 2., n-1)
    ci_lower = mean - h
    ci_upper = mean + h
    return mean, variance, ci_lower, ci_upper

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

    # Initialize a list to store the summary statistics
    summary_stats = []

    # Load data and compute statistics
    for size in sizes:
        transport_data = load_transport_times(size, scenario_dirs)
        for scenario, times in transport_data.items():
            mean, variance, ci_lower, ci_upper = calc_stats(times)
            summary_stats.append({
                'Scenario': scenario,
                'Message Size': size,
                'Mean': mean,
                'Variance': variance,
                'CI Lower Bound': ci_lower,
                'CI Upper Bound': ci_upper
            })

    # Convert the summary statistics to a DataFrame for better readability
    summary_df = pd.DataFrame(summary_stats)

    # Display the summary table
    print(summary_df.to_string())

    # Optionally, save the summary table to a CSV file
    summary_df.to_csv(os.path.join(base_dir, 'summary_statistics.csv'), index=False)

if __name__ == '__main__':
    main()
