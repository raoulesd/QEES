import numpy as np
import os
import matplotlib.pyplot as plt

def main():
    # Given data points
    data_points = [1, 1.3, 1.57, 1.98, 2.4, 2.7, 2.99, 3.2, 3.67, 3.98]

    # Calculate the derivative (difference between successive data points)
    derivative = np.diff(data_points)

    # Calculate the average of the derivative
    average_derivative = np.mean(derivative)

    # Extend the derivative to match the length of the data points (for plotting)
    derivative_extended = np.concatenate(([derivative[0]], derivative))

    # Create the average line (a constant line with the average derivative value)
    average_derivative_line = np.full(len(data_points), average_derivative)

    # Plotting the data
    plt.figure(figsize=(10, 6))

    # Plot the original data points
    plt.plot(range(1, 11), data_points, label='Data Points', marker='o')

    # Plot the derivative of the data points
    # plt.plot(range(1, 11), derivative_extended, label='Derivative', linestyle='--', marker='x')

    # Plot the average of the derivative
    # plt.plot(range(1, 11), average_derivative_line, label='Average Derivative', linestyle=':')

    # Adjust x-axis ticks to show whole numbers from 1 to 10
    plt.xticks(np.arange(1, 11, 1))

    # Labels and title
    plt.xlabel('# listeners')
    plt.ylabel('Time [ms]')
    plt.title('Average transport time per number of listeners')

    # Add legend
    # plt.legend()

    output_path = os.path.join('../', 'ttplot.png')
    plt.savefig(output_path)

    # Display the plot
    plt.show()

# Ensures that the main function runs when the script is executed
if __name__ == "__main__":
    main()