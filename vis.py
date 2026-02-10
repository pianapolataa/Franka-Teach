import numpy as np
import matplotlib.pyplot as plt

def save_trajectory_plot(file_path="arm_write_trajectory.png"):
    # Load the recorded data
    try:
        data = np.load(file_path)
    except FileNotFoundError:
        print(f"Error: {file_path} not found. Ensure you have recorded a trajectory first.")
        return

    # Assuming 100Hz recording (0.01s intervals)
    time_axis = np.arange(len(data)) * 0.01

    # Create 7 subplots (3 for Position, 4 for Orientation)
    fig, axs = plt.subplots(7, 1, figsize=(12, 18), sharex=True)
    fig.suptitle(f'Franka Arm Trajectory Visualization\nFile: {file_path}', fontsize=16, fontweight='bold')

    # Define labels and distinct colors for each dimension
    labels = ['Position X (m)', 'Position Y (m)', 'Position Z (m)', 
              'Quat X', 'Quat Y', 'Quat Z', 'Quat W']
    colors = ['#e63946', '#2a9d8f', '#457b9d', '#f4a261', '#7209b7', '#4cc9f0', '#2b2d42']

    # Your specific "Home" values to visualize as reference lines
    home_vals = [0.45442212, 0.03222251, 0.36761105, 0.94005555, 0.34075424, 0.01184287, 0.00646103]

    for i in range(7):
        # Plot the actual recorded data
        axs[i].plot(time_axis, data[:, i], color=colors[i], linewidth=2, label='Recorded Path')
        
        # Plot the reference line for the home position
        axs[i].axhline(y=home_vals[i], color='black', linestyle='--', alpha=0.4, label='Home/Static Ref')
        
        axs[i].set_ylabel(labels[i], fontsize=10, fontweight='bold')
        axs[i].grid(True, which='both', linestyle=':', alpha=0.7)
        axs[i].legend(loc='upper right', fontsize='small')

    # Set the x-axis label only on the bottom plot
    axs[-1].set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')

    # Adjust layout to make room for titles and labels
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Save as the requested filename
    plt.savefig("states_arm", dpi=300)
    print(f"Visualization saved to: states_arm")
    plt.close()

if __name__ == "__main__":
    # Ensure the script runs using the data you recorded earlier
    save_trajectory_plot()