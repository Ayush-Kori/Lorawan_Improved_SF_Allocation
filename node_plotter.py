
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load data from the CSV file
df = pd.read_csv("Data_Set_1_with_SF_Allocator.csv", low_memory=False)

# Convert 'X Coordinate' and 'Y Coordinate' columns to floats, handling NaN values
df['X Coordinate'] = pd.to_numeric(df['X Coordinate'], errors='coerce')
df['Y Coordinate'] = pd.to_numeric(df['Y Coordinate'], errors='coerce')


# Define colors for SF values
sf_colors = {'7': 'blue', '8': 'green', '9': 'red', '10': 'orange', '11': 'purple', '12': 'brown'}

# Create a folder to save plots
import os
os.makedirs("Lorawan_Plots", exist_ok=True)

# Initialize variables to track changes in 'Nodes' and 'Radius'
prev_nodes, prev_radius = None, None

# Set the range for x and y axes
x_range = (-6400, 6400)
y_range = (-6400, 6400)

# Loop through each row in the DataFrame
for index, row in df.iterrows():
    # Check if 'Nodes' and 'Radius' values have changed
    if row['Nodes'] != prev_nodes or row['Radius'] != prev_radius:
        # If changed, update the previous values and save the plot
        plt.title(f'Nodes={row["Nodes"]}, Radius={row["Radius"]}')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.legend(title='SF')
        
        # Set the x and y axis limits
        plt.xlim(x_range)
        plt.ylim(y_range)

        # Save the plot
        plot_name = f"Lorawan_Plots/{row['Nodes']}_{row['Radius']}.png"
        plt.savefig(plot_name)
        plt.close()

        # Update previous values
        prev_nodes, prev_radius = row['Nodes'], row['Radius']

    # Plot the current point with label and color based on 'SF' value
    sf_value = row['SF']  # Retrieve SF value
    plt.scatter(row['X Coordinate'], row['Y Coordinate'], color=sf_colors.get(sf_value), marker='o')

# Save the final plot for the last 'Nodes' and 'Radius'
if prev_nodes is not None and prev_radius is not None:
    plt.title(f'Nodes={prev_nodes}, Radius={prev_radius}')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend(title='SF')

    # Set the x and y axis limits
    plt.xlim(x_range)
    plt.ylim(y_range)

    # Save the plot
    plot_name = f"Lorawan_Plots/{prev_nodes}_{prev_radius}.png"
    plt.savefig(plot_name)
    plt.close()

