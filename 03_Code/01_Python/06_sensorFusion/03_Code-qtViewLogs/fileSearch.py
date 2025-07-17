import os

def find_project_root(start_path, target_folder_name):
    """Search upward in the directory tree for the specified folder."""
    current_path = start_path
    while True:
        if os.path.basename(current_path) == target_folder_name:
            return current_path
        parent_path = os.path.dirname(current_path)
        if parent_path == current_path:
            raise FileNotFoundError(f"Folder '{target_folder_name}' not found in path hierarchy.")
        current_path = parent_path