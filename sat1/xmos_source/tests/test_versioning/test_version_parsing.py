import pytest
from versioning import XMOSVersion, PRE_RELEASES

# ✅ Fixture to provide a valid XMOSVersion instance
@pytest.fixture
def version():
    return XMOSVersion(1, 2, 3, PRE_RELEASES.index("dev"), 5)

# ✅ Test version parsing with multiple cases
@pytest.mark.parametrize(
    "version_str, expected",
    [
        ("v1.2.3", (1, 2, 3, 0, 0)), # Standard version
        ("v1.2.3-alpha.4", (1, 2, 3, PRE_RELEASES.index("alpha"), 4)),  # Pre-release
        ("v1.2.3-beta.4", (1, 2, 3, PRE_RELEASES.index("beta"), 4)),  # Pre-release
        ("v10.20.30-rc", (10, 20, 30, PRE_RELEASES.index("rc"), 0)),  # Release Candidate
        ("v1.2.3-dev", (1, 2, 3, PRE_RELEASES.index("dev"), 0)),  # untracked dev-build version
        ("v1.2.3-dev.12", (1, 2, 3, PRE_RELEASES.index("dev"), 12)),  # tracked dev-build version
    ]
)
def test_version_parsing(version_str, expected):
    version = XMOSVersion.from_string(version_str)
    assert (version.major, version.minor, version.patch, version.pre_release, version.pre_counter) == expected

# ✅ Test invalid version raises SystemExit
@pytest.mark.parametrize("invalid_version", ["invalid", "v1", "v1.x.y"])
def test_invalid_version_string(invalid_version):
    with pytest.raises(SystemExit):
        XMOSVersion.from_string(invalid_version)

