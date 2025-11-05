#!/usr/bin/env python3
import yaml
import numpy as np
import matplotlib.pyplot as plt

def load_yaml_data(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def plot_distance_heatmap(data, grid_range=(-15, 15), grid_res=0.05):
    """
    Heatmap with red background and white spots at robot positions.
    Far distances -> bigger, brighter spots; close distances -> smaller, weaker spots.
    """
    if not data:
        print("No data to plot.")
        return

    x_min, x_max = grid_range
    y_min, y_max = grid_range

    x_grid = np.arange(x_min, x_max, grid_res)
    y_grid = np.arange(y_min, y_max, grid_res)
    heatmap = np.zeros((len(y_grid), len(x_grid)))

    X, Y = np.meshgrid(x_grid, y_grid)

    for entry in data:
        robot_x = entry['robot_position']['x']
        robot_y = entry['robot_position']['y']
        distance = entry['distance']

        sigma = np.clip(distance / 5.0, 0.05, 1)
        intensity = np.clip(distance / 10.0, 0.1, 0.8)
        gauss = np.exp(-(((X - robot_x) ** 2 + (Y - robot_y) ** 2) / (2 * sigma ** 2)))
        heatmap += gauss * intensity

    heatmap = np.clip(heatmap, 0, 1)

    rgb = np.zeros((*heatmap.shape, 3))
    rgb[..., 0] = 1.0  # red
    rgb[..., 1] = 0.0
    rgb[..., 2] = 0.0

    rgb = rgb * (1 - heatmap[..., None]) + heatmap[..., None]

    plt.figure(figsize=(8, 8))
    plt.imshow(rgb[::-1, :, :], extent=[x_min, x_max, y_min, y_max], origin='lower')
    plt.xlabel("X position")
    plt.ylabel("Y position")
    plt.title("Distance Heatmap (Red = close, White = far)")
    plt.tight_layout()
    plt.savefig("distance_heatmap_rgb.png")
    print("Heatmap saved as 'distance_heatmap_rgb.png'")
    plt.show()


if __name__ == "__main__":
    yaml_path = "tree_distance_graph.yaml"
    data = load_yaml_data(yaml_path)
    plot_distance_heatmap(data)
