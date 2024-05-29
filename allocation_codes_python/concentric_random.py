import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

# Load data from the CSV file
df = pd.read_csv("Predicted_SF_Result.csv")

# Filter rows where 'Nodes' is 1750 and 'Radius' is 6400
df = df[(df['Nodes'] == 1750) & (df['Radius'] == 6400)].reset_index(drop=True)

# Create directory for saving plots
plots_dir = "Lorawan_Plots/Clustering"
os.makedirs(plots_dir, exist_ok=True)

# Define color scheme for SF values
sf_colors = {'7': 'blue', '8': 'green', '9': 'red', '10': 'orange', '11': 'purple', '12': 'brown'}

# Function to calculate Euclidean distance between two points
def euclidean_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

# Function to find the nearest 6 coordinates to a given coordinate
def find_nearest_6(coordinates, current_index):
    current_coord = coordinates[current_index]
    distances = [(i, euclidean_distance(current_coord, coordinates[i])) for i in range(len(coordinates)) if i != current_index]
    distances.sort(key=lambda x: x[1])
    return [coordinates[index] for index, _ in distances[:6]]

# Function to allocate SFs based on distance slot
def allocate_sf(distance, sf_allocation):
    if distance <= 1200:
        sf_options = ['7', '8']
    elif distance <= 2400:
        sf_options = ['8', '9']
    elif distance <= 3600:
        sf_options = ['9', '10']
    elif distance <= 4800:
        sf_options = ['10', '11']
    elif distance <= 6000:
        sf_options = ['11', '12']
    else:
        sf_options = ['12']

    # Allocate SF to cluster such that no alternate clusters get allocated the same SF within that distance slot
    for sf in sf_options:
        if len(sf_allocation[sf]) == 0 or sf_allocation[sf][-1] != index - 1:
            sf_allocation[sf].append(index)
            return sf
    return None

# Initialize variables to track the previous combination of 'Nodes' and 'Radius'
prev_nodes = None
prev_radius = None

# Initialize SF allocation dictionary for each distance slot
sf_allocation = {'7': [], '8': [], '9': [], '10': [], '11': [], '12': []}

# Initialize cluster centroids dictionary for each distance slot
cluster_centroids = {'7': [], '8': [], '9': [], '10': [], '11': [], '12': []}

# Plotting
for index, row in df.iterrows():
    nodes = row['Nodes']
    radius = row['Radius']
    x_coord = row['X Coordinate']
    y_coord = row['Y Coordinate']
    
    # Check if combination of 'Nodes' and 'Radius' changed
    if nodes != prev_nodes or radius != prev_radius:
        # If changed, reset cluster ID
        plt.figure(figsize=(12, 10))
        prev_nodes = nodes
        prev_radius = radius
    
    # Form the cluster by finding nearest 6 coordinates
    cluster_coords = find_nearest_6(df[['X Coordinate', 'Y Coordinate']].values, index)
    cluster_coords.append([x_coord, y_coord])  # Include the current coordinate in the cluster
    cluster_coords = np.array(cluster_coords)
    
    # Calculate distance of centroid from origin
    distance_from_origin = np.sqrt(x_coord**2 + y_coord**2)
    
    # Allocate SF based on distance slot
    sf = allocate_sf(distance_from_origin, sf_allocation)
    
    # If SF couldn't be allocated, skip plotting this cluster
    if sf is None:
        continue
    
    # Plot cluster
    plt.scatter(cluster_coords[:, 0], cluster_coords[:, 1], marker='o', label=f'Nodes={nodes}, Radius={radius}', color=sf_colors[sf])
    
    # Save cluster centroid
    cluster_centroids[sf].append([x_coord, y_coord])
    
    # Save plot when combination of 'Nodes' and 'Radius' changes or at the end of the file
    if index == len(df) - 1 or (nodes != df.iloc[index+1]['Nodes'] or radius != df.iloc[index+1]['Radius']):
        plt.title(f'Cluster for Nodes={nodes}, Radius={radius}')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.grid(True)
        
        # Save legend of SFs with their color schemes outside the plot
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5), title='SF Allocation')
        legend_handles = [plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=sf_colors[sf], markersize=10, label=f'SF {sf}') for sf in sf_allocation.keys()]
        plt.legend(handles=legend_handles, loc='center left', bbox_to_anchor=(1, 0.5), title='SF Allocation')
        
        # Save plot
        plot_name = os.path.join(plots_dir, f"{nodes}_{radius}.png")
        plt.savefig(plot_name, bbox_inches='tight')
        plt.close()

# Allocate alternate SFs based on cluster centroids
alternate_sf_allocation = {'7': '8', '8': '7', '9': '10', '10': '9', '11': '12', '12': '11'}

for sf, centroids in cluster_centroids.items():
    for centroid in centroids:
        distance_from_origin = np.sqrt(centroid[0]**2 + centroid[1]**2)
        if sf in ['7', '8']:
            alternate_sf = alternate_sf_allocation[sf]
        elif sf in ['9', '10']:
            alternate_sf = alternate_sf_allocation[sf]
        elif sf in ['11', '12']:
            alternate_sf = alternate_sf_allocation[sf]
        else:
            alternate_sf = None
        if alternate_sf:
            plt.scatter(centroid[0], centroid[1], marker='*', color=sf_colors[alternate_sf], s=200)

