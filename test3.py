import pandas as pd
import matplotlib.pyplot as plt
import os

# Load data from the CSV file
df = pd.read_csv("Predicted_SF_Result.csv", low_memory=False)

# Convert 'X Coordinate' and 'Y Coordinate' columns to floats, handling NaN values
df['X Coordinate'] = pd.to_numeric(df['X Coordinate'], errors='coerce')
df['Y Coordinate'] = pd.to_numeric(df['Y Coordinate'], errors='coerce')

# Define colors for SF values
sf_colors = {'7': 'blue', '8': 'green', '9': 'red', '10': 'orange', '11': 'purple', '12': 'brown'}
sf_labels = {'7': 'SF 7', '8': 'SF 8', '9': 'SF 9', '10': 'SF 10', '11': 'SF 11', '12': 'SF 12'}

model_dir = f"Lorawan_NS3_simulator_Plots/"
os.makedirs(model_dir, exist_ok=True)  # Create directory if it doesn't exist

# Set the range for x and y axes
x_range = (-6400, 6400)
y_range = (-6400, 6400)

# Initialize lists to store legend handles and labels
handles = []
labels = []

# Loop through each row in the DataFrame
for index, row in df.iterrows():
    # Check if 'Nodes' = 1750 and 'Radius' = 6400
    if row['Nodes'] == 1750 and row['Radius'] == 6400:
        # Plot the current point with label and color based on 'SF' value
        sf_value = str(row['SF'])  # Retrieve SF value
        handle = plt.scatter(row['X Coordinate'], row['Y Coordinate'], color=sf_colors.get(sf_value), marker='o')
        
        # Add handle and label for legend
        handles.append(handle)
        labels.append(sf_labels.get(sf_value))

# Save the plot
plot_name = f"{model_dir}/Nodes_1750_Radius_6400.png"
plt.title(f'Lorawan_NS3_simulator - Nodes=1750, Radius=6400')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.xlim(x_range)
plt.ylim(y_range)
plt.legend(handles, labels)  # Add legend
plt.savefig(plot_name)
plt.close()

