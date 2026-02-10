import numpy as np
import matplotlib.pyplot as plt

def visualize_trajectory(file_path="arm_write_trajectory.npy"):
    # Load the data
    try:
        data = np.load(file_path)
    except FileNotFoundError:
        print(f"Error: {file_path} not found.")
        return

    # Create time axis (assuming 100Hz / 0.01s interval)
    time_axis = np.linspace(0, len(data) * 0.01, len(data))

    fig, axs = plt.subplots(7, 1, figsize=(10, 15), sharex=True)
    fig.suptitle(f'Franka Arm Trajectory: {file_path}', fontsize=16)

    labels = ['Pos X', 'Pos Y', 'Pos Z', 'Quat X', 'Quat Y', 'Quat Z', 'Quat W']
    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']

    for i in range(7):
        axs[i].plot(time_axis, data[:, i], color=colors[i], linewidth=1.5)
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True, alpha=0.3)
        
        # Highlight the specific value you were trying to skip earlier if it appears
        # target_val = [0.4544, 0.0322, 0.3676, 0.9400, 0.3407, 0.0118, 0.0064]
        # axs[i].axhline(y=target_val[i], color='gray', linestyle='--', alpha=0.5)

    axs[-1].set_xlabel('Time (seconds)')
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

if __name__ == "__main__":
    visualize_trajectory()