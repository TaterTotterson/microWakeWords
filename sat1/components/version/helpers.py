import subprocess

def has_changed_files():
    try:
        # Run the 'git status --porcelain' command
        result = subprocess.run(['git', 'status', '--porcelain'], capture_output=True, text=True, check=True)
        
        # Check if the output is not empty
        if result.stdout.strip():
            return True
        else:
            return False
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while checking the Git status: {e}")
        return False

def get_current_commit_hash():
    try:
        # Run the 'git rev-parse HEAD' command
        result = subprocess.run(['git', 'rev-parse', '--short=10' ,'HEAD'], capture_output=True, text=True, check=True)
        
        # Return the commit hash, stripping any trailing newline
        return result.stdout.strip()
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while getting the current commit hash: {e}")
        return None

