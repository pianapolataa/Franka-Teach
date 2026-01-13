import pickle
from pathlib import Path
import numpy as np

def modify_ruka_files(root_dir_name="bread_data"):
    root_path = Path(root_dir_name)
    
    if not root_path.exists():
        print(f"Error: Directory '{root_dir_name}' not found.")
        return

    # Find all demonstration folders
    demo_folders = sorted(list(root_path.glob("demonstration_*")))
    
    files_to_process = [
        "ruka_states.pkl",
        "ruka_commanded_states.pkl"
    ]
    
    target_index = 12
    subtraction_value = -300

    print(f"Found {len(demo_folders)} demonstration folders.")

    for demo_folder in demo_folders:
        print(f"Processing {demo_folder.name}...")
        
        for file_name in files_to_process:
            file_path = demo_folder / file_name
            
            if not file_path.exists():
                print(f"  Skipping {file_name} (not found)")
                continue

            # 1. Load the data
            try:
                with open(file_path, "rb") as f:
                    data = pickle.load(f)
            except Exception as e:
                print(f"  Error loading {file_name}: {e}")
                continue

            # 2. Modify the data
            # Structure is assumed to be: [{'state': [16 items], 'timestamp': float}, ...]
            modified_count = 0
            for entry in data:
                if 'state' in entry:
                    # Check if it's a list or numpy array and handle accordingly
                    state_vector = entry['state']
                    state_vector[target_index] -= subtraction_value
                    modified_count += 1
            
            # 3. Create Backup (Safety first!)
            backup_path = file_path.with_suffix('.pkl.bak')
            if not backup_path.exists():
                with open(file_path, "rb") as original_file:
                    with open(backup_path, "wb") as backup_file:
                        backup_file.write(original_file.read())

            # 4. Save the modified data back
            with open(file_path, "wb") as f:
                pickle.dump(data, f)
            
            print(f"  Updated {file_name}: Modified {modified_count} entries. (Backup saved as .bak)")

    print("\nProcessing complete.")

if __name__ == "__main__":
    modify_ruka_files()