#!/usr/bin/env python3
"""
Script to recursively find and delete all 'build' folders in the current directory and subdirectories.
"""

import os
import shutil
from pathlib import Path


def delete_build_folders(root_path="."):
    """
    Recursively search for 'build' folders and delete them.
    
    Args:
        root_path: The starting directory to search from (defaults to current directory)
    """
    root_path = Path(root_path).resolve()
    deleted_folders = []
    errors = []
    
    print(f"Searching for 'build' folders in: {root_path}")
    print("-" * 60)
    
    # Walk through all directories
    for dirpath, dirnames, filenames in os.walk(root_path, topdown=False):
        # Check if current directory name is 'build'
        if os.path.basename(dirpath).lower() == 'build':
            try:
                print(f"Found: {dirpath}")
                shutil.rmtree(dirpath)
                deleted_folders.append(dirpath)
                print(f"✓ Deleted: {dirpath}")
            except Exception as e:
                error_msg = f"✗ Failed to delete {dirpath}: {str(e)}"
                print(error_msg)
                errors.append(error_msg)
    
    # Print summary
    print("-" * 60)
    print(f"\nSummary:")
    print(f"  Total 'build' folders deleted: {len(deleted_folders)}")
    
    if deleted_folders:
        print("\nDeleted folders:")
        for folder in deleted_folders:
            print(f"  - {folder}")
    
    if errors:
        print("\nErrors encountered:")
        for error in errors:
            print(f"  - {error}")
    
    if not deleted_folders and not errors:
        print("  No 'build' folders found.")


def main():
    """Main entry point."""
    # Get the directory where the script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    print("=" * 60)
    print("Build Folder Deletion Script")
    print("=" * 60)
    
    # Ask for confirmation
    response = input(f"\nThis will delete all 'build' folders found in:\n{script_dir}\n\nContinue? (yes/no): ")
    
    if response.lower() in ['yes', 'y']:
        delete_build_folders(script_dir)
    else:
        print("Operation cancelled.")


if __name__ == "__main__":
    main()
