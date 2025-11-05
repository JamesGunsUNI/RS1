import yaml
import numpy as np
import matplotlib.pyplot as plt

def plot_points_gaussian_heatmap(filename, grid_range=(-15, 15), grid_res=0.05, sigma=1):
    """
    Heatmap where each moisture point radiates as a Gaussian spot.
    White → light blue → deep blue colormap.
    - sigma controls how far the gradient spreads (larger = wider, slower drop-off)
    """
    cmap = plt.get_cmap("Blues")

    with open(filename, "r") as f:
        data = yaml.safe_load(f)

    for name, points in data.items():
        xs = np.array([p["x"] for p in points])
        ys = np.array([p["y"] for p in points])
        vals = np.array([p["value"] for p in points])
        vals = np.clip(vals, 0, 1)

        x_min, x_max = grid_range
        y_min, y_max = grid_range
        x_grid = np.arange(x_min, x_max, grid_res)
        y_grid = np.arange(y_min, y_max, grid_res)
        X, Y = np.meshgrid(x_grid, y_grid)
        heatmap = np.zeros_like(X)

        for x, y, v in zip(xs, ys, vals):
            stretch = 0.4 
            gauss = np.exp(-(((X - x)**2 + (Y - y)**2) / (2 * sigma**2))**stretch)
            heatmap += gauss * v

        heatmap = np.clip(heatmap, 0, 1)

        plt.figure(figsize=(6,6))
        plt.imshow(
            heatmap[::-1, :],
            extent=[x_min, x_max, y_min, y_max],
            origin="lower",
            cmap=cmap,
            vmin=0,
            vmax=1,
            aspect="equal"
        )
        plt.colorbar(label="Moisture Level (0 = dry, 1 = wet)")
        plt.title(f"{name} (Moisture Heatmap)")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    plot_points_gaussian_heatmap("heatmap_data.yaml")
