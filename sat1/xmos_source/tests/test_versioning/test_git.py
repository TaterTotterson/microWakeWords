import pytest
import subprocess

from unittest.mock import patch, MagicMock
from versioning import GitInfo


# ✅ Mock subprocess output for Git commands
@patch("subprocess.run")
def test_git_info(mock_subproc):
    """Test GitInfo.from_ws() with mocked Git responses."""

    # Create a mock return object with `.stdout.strip()`
    def mock_git_command(args, cwd, capture_output, text, check):
        mock = MagicMock()
        
        # Simulated outputs for each Git command
        if args == ["git", "rev-parse", "--abbrev-ref", "HEAD"]:
            mock.stdout = "main\n"
        elif args == ["git", "rev-parse", "--short", "HEAD"]:
            mock.stdout = "abc123\n"
        elif args == ["git", "status", "--porcelain"]:
            mock.stdout = ""  # No changed files
        elif args == ["git", "describe", "--tags", "--abbrev=0"]:
            mock.stdout = "v1.2.3\n"
        elif args == ["git", "diff"]:
            mock.stdout = "patch content"
        return mock

    # Set side_effect for mock_subproc
    mock_subproc.side_effect = mock_git_command

    # Call the function being tested
    git_info = GitInfo.from_ws()

    # ✅ Assertions
    assert git_info.branch == "main"
    assert git_info.commit == "abc123"
    assert git_info.last_tag == "v1.2.3"
    assert git_info.patch_str == "patch content"

# ✅ Mock case where no Git tags exist
@patch("subprocess.run")
def test_git_info_no_tags(mock_subproc):
    """Test GitInfo when no Git tags exist."""

    def mock_git_command(args, cwd, capture_output, text, check):
        mock = MagicMock()
        if args == ["git", "rev-parse", "--abbrev-ref", "HEAD"]:
            mock.stdout = "main\n"
        elif args == ["git", "rev-parse", "--short", "HEAD"]:
            mock.stdout = "abc123\n"
        elif args == ["git", "status", "--porcelain"]:
            mock.stdout = " "
        elif args == ["git", "describe", "--tags", "--abbrev=0"]:
            raise subprocess.CalledProcessError(1, "git describe")  # Simulate no tags
        elif args == ["git", "diff"]:
            mock.stdout = "patch content"
        return mock

    mock_subproc.side_effect = mock_git_command

    git_info = GitInfo.from_ws()

    # ✅ Expect branch and commit, but last_tag should be None
    assert git_info.branch == "main"
    assert git_info.commit == "abc123"
    assert git_info.last_tag is None
