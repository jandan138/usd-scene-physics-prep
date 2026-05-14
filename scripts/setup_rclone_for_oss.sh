#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# Setup Rclone for OSS Upload on a Fresh DSW
# =============================================================================
# Usage: bash scripts/setup_rclone_for_oss.sh [--non-interactive]
#
# This script installs rclone v1.68.1+ (verified for Alibaba Cloud OSS support)
# and configures it with the project's standard settings.
#
# PREREQUISITES:
#   - Environment variables (or interactive input):
#       ALIBABA_ACCESS_KEY_ID
#       ALIBABA_SECRET_ACCESS_KEY
#   - sudo access (for dpkg install)
#
# WHY THIS SCRIPT EXISTS:
#   - apt-installed rclone (v1.53.3) does NOT support Alibaba Cloud OSS provider
#   - DSW external network is extremely slow (~20 KB/s); we use pre-downloaded
#     package if available, or wait for the wget download
#   - The bucket has IP whitelist; MUST use internal endpoint
#   - Proxy variables (HTTP_PROXY, etc.) can cause rclone to hang in
#     non-interactive shells
#
# SECURITY NOTES:
#   - This script never stores credentials in the repository.
#   - Provide credentials via env vars or enter them interactively:
#       export ALIBABA_ACCESS_KEY_ID=your-key
#       export ALIBABA_SECRET_ACCESS_KEY=your-secret
#   - Generated config file has 600 permissions.
# =============================================================================

RCLONE_VERSION="1.68.1"
RCLONE_DEB="rclone-v${RCLONE_VERSION}-linux-amd64.deb"
RCLONE_URL="https://downloads.rclone.org/v${RCLONE_VERSION}/${RCLONE_DEB}"
REMOTE_NAME="aliyun-a-oss-demo"
BUCKET="pjlab-bjpai-zhuzihou-assets"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# ---------------------------------------------------------------------------
# Parse arguments
# ---------------------------------------------------------------------------
NON_INTERACTIVE=0
while [[ $# -gt 0 ]]; do
    case $1 in
        --non-interactive)
            NON_INTERACTIVE=1
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--non-interactive]"
            echo ""
            echo "Environment variables:"
            echo "  ALIBABA_ACCESS_KEY_ID       Alibaba Cloud Access Key ID"
            echo "  ALIBABA_SECRET_ACCESS_KEY   Alibaba Cloud Secret Access Key"
            echo ""
            echo "Examples:"
            echo "  # Interactive mode (prompts for credentials if env vars not set):"
            echo "  bash scripts/setup_rclone_for_oss.sh"
            echo ""
            echo "  # Non-interactive mode (requires env vars):"
            echo "  export ALIBABA_ACCESS_KEY_ID=your-key"
            echo "  export ALIBABA_SECRET_ACCESS_KEY=your-secret"
            echo "  bash scripts/setup_rclone_for_oss.sh --non-interactive"
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# ---------------------------------------------------------------------------
# Step 0: Pre-flight checks
# ---------------------------------------------------------------------------
log_info "Running pre-flight checks..."

# Check sudo access
if ! sudo -n true 2>/dev/null; then
    if [ "$NON_INTERACTIVE" -eq 1 ]; then
        log_error "sudo access required but not available in non-interactive mode"
        log_error "Please configure passwordless sudo or run interactively"
        exit 1
    else
        log_warn "This script requires sudo for installing rclone"
        log_warn "You may be prompted for your password"
        # Try sudo once to trigger password prompt early
        sudo true || {
            log_error "sudo authentication failed"
            exit 1
        }
    fi
fi

# Check network connectivity (basic)
if ! curl -s --max-time 10 https://downloads.rclone.org > /dev/null 2>&1; then
    log_warn "External network appears slow or unavailable"
    log_warn "If rclone installation is needed, consider pre-downloading the package"
fi

# ---------------------------------------------------------------------------
# Step 1: Get credentials
# ---------------------------------------------------------------------------
ACCESS_KEY="${ALIBABA_ACCESS_KEY_ID:-}"
SECRET_KEY="${ALIBABA_SECRET_ACCESS_KEY:-}"

if [ -z "$ACCESS_KEY" ] || [ -z "$SECRET_KEY" ]; then
    if [ "$NON_INTERACTIVE" -eq 1 ]; then
        log_error "Credentials are required in non-interactive mode"
        log_error "Set ALIBABA_ACCESS_KEY_ID and ALIBABA_SECRET_ACCESS_KEY before running"
        exit 1
    fi

    echo ""
    echo "Enter Alibaba Cloud OSS credentials."
    echo "These values will be written only to ${HOME}/.config/rclone/rclone.conf."
    echo ""
    if [ -z "$ACCESS_KEY" ]; then
        read -r -p "ALIBABA_ACCESS_KEY_ID: " ACCESS_KEY
    fi
    if [ -z "$SECRET_KEY" ]; then
        read -r -s -p "ALIBABA_SECRET_ACCESS_KEY: " SECRET_KEY
        echo
    fi
fi

if [ -z "$ACCESS_KEY" ] || [ -z "$SECRET_KEY" ]; then
    log_error "Credentials cannot be empty"
    exit 1
fi

# ---------------------------------------------------------------------------
# Step 2: Check existing rclone
# ---------------------------------------------------------------------------
log_info "Checking existing rclone installation..."

INSTALL_NEEDED=0
if command -v rclone &> /dev/null; then
    CURRENT_VERSION=$(rclone version | head -1 | grep -oP 'v\K[0-9.]+' || echo "unknown")
    log_info "Found rclone v${CURRENT_VERSION}"

    # Check if version is >= 1.68.1 (handles "unknown" gracefully)
    if [ "$CURRENT_VERSION" = "unknown" ] || \
       ! printf '%s\n' "${RCLONE_VERSION}" "$CURRENT_VERSION" | sort -V -C; then
        log_warn "Version ${CURRENT_VERSION} is too old or unknown. Verified working version is ${RCLONE_VERSION}+"
        INSTALL_NEEDED=1
    else
        log_info "Version is sufficient (>= ${RCLONE_VERSION})"
    fi
else
    log_info "rclone not found. Will install..."
    INSTALL_NEEDED=1
fi

# ---------------------------------------------------------------------------
# Step 3: Install rclone if needed
# ---------------------------------------------------------------------------
if [ "$INSTALL_NEEDED" -eq 1 ]; then
    log_info "Installing rclone v${RCLONE_VERSION}..."

    # Check if pre-downloaded package exists in common locations
    PRE_DOWNLOADED=""
    for loc in "/tmp/${RCLONE_DEB}" "${HOME}/downloads/${RCLONE_DEB}" "${HOME}/${RCLONE_DEB}"; do
        if [ -f "$loc" ]; then
            PRE_DOWNLOADED="$loc"
            log_info "Found pre-downloaded package at: $loc"
            break
        fi
    done

    if [ -n "$PRE_DOWNLOADED" ]; then
        sudo dpkg -i "$PRE_DOWNLOADED" || {
            log_error "Failed to install from pre-downloaded package"
            exit 1
        }
    else
        log_warn "Downloading rclone from ${RCLONE_URL}"
        log_warn "DSW external network is slow (~20 KB/s). This may take 15+ minutes for 21 MB."
        log_warn "Tip: Download on your local machine and scp to DSW:"
        log_warn "     scp rclone-v${RCLONE_VERSION}-linux-amd64.deb dsw-host:/tmp/"

        cd /tmp
        wget --progress=dot:giga "${RCLONE_URL}" -O "${RCLONE_DEB}" || {
            log_error "Download failed. Please download manually and place at /tmp/${RCLONE_DEB}"
            exit 1
        }

        sudo dpkg -i "/tmp/${RCLONE_DEB}" || {
            log_error "Failed to install downloaded package"
            exit 1
        }
    fi

    log_info "rclone installed successfully"
    rclone version | head -1
fi

# ---------------------------------------------------------------------------
# Step 4: Configure rclone remote
# ---------------------------------------------------------------------------
log_info "Configuring rclone remote: ${REMOTE_NAME}"

CONFIG_DIR="${HOME}/.config/rclone"
mkdir -p -m 700 "$CONFIG_DIR"
CONFIG_FILE="${CONFIG_DIR}/rclone.conf"

# Check if remote already exists
if [ -f "$CONFIG_FILE" ] && grep -q "^\[${REMOTE_NAME}\]$" "$CONFIG_FILE" 2>/dev/null; then
    if [ "$NON_INTERACTIVE" -eq 1 ]; then
        log_info "Remote '${REMOTE_NAME}' already exists. Updating configuration..."
        # Remove existing remote block and recreate
        # Use sed to delete from [REMOTE_NAME] to next [ or end of file
        sed -i "/^\[${REMOTE_NAME}\]$/,/^\[/ { /^\[/!d; /^\[${REMOTE_NAME}\]$/d }" "$CONFIG_FILE"
    else
        log_warn "Remote '${REMOTE_NAME}' already exists in ${CONFIG_FILE}"
        read -p "Overwrite existing config? [y/N]: " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            sed -i "/^\[${REMOTE_NAME}\]$/,/^\[/ { /^\[/!d; /^\[${REMOTE_NAME}\]$/d }" "$CONFIG_FILE"
        else
            log_info "Skipped configuration. Existing config kept."
            exit 0
        fi
    fi
fi

# CRITICAL: Use internal endpoint to bypass IP whitelist
# The bucket '${BUCKET}' has IP whitelist restrictions
# Public endpoint (oss-cn-beijing.aliyuncs.com) will be blocked
# Internal endpoint (oss-cn-beijing-internal.aliyuncs.com) works within the same VPC

# Append new remote config to file (safer than overwriting entire file)
cat >> "$CONFIG_FILE" << EOF

[${REMOTE_NAME}]
type = s3
provider = Alibaba
env_auth = false
access_key_id = ${ACCESS_KEY}
secret_access_key = ${SECRET_KEY}
endpoint = oss-cn-beijing-internal.aliyuncs.com
region = cn-beijing
acl = private
EOF

chmod 600 "$CONFIG_FILE"
log_info "Configuration written to ${CONFIG_FILE}"

# ---------------------------------------------------------------------------
# Step 5: Verify configuration
# ---------------------------------------------------------------------------
log_info "Verifying configuration..."

# Test with proxy variables unset (they can cause hangs in non-interactive shells)
if env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy -u ALL_PROXY -u all_proxy \
   timeout 30 rclone listremotes | grep -q "^${REMOTE_NAME}:$"; then
    log_info "Remote '${REMOTE_NAME}:' is configured"
else
    log_error "Remote '${REMOTE_NAME}:' not found in rclone config"
    exit 1
fi

# Test connectivity to bucket
log_info "Testing bucket connectivity..."
if env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy -u ALL_PROXY -u all_proxy \
   timeout 30 rclone lsf "${REMOTE_NAME}:${BUCKET}" > /dev/null 2>&1; then
    log_info "SUCCESS: Bucket '${BUCKET}' is accessible!"
else
    log_error "FAILED: Cannot access bucket '${BUCKET}'"
    log_error "Possible causes:"
    log_error "  1. Network issues (check if you're in the correct VPC)"
    log_error "  2. Invalid credentials"
    log_error "  3. Bucket permission issues"
    log_error ""
    log_error "Try manually: rclone lsf ${REMOTE_NAME}:${BUCKET}"
    exit 1
fi

# ---------------------------------------------------------------------------
# Step 6: Print usage summary
# ---------------------------------------------------------------------------
echo ""
echo "================================================================================"
echo "                    Rclone Setup Complete!"
echo "================================================================================"
echo ""
echo "Remote: ${REMOTE_NAME}:"
echo "Bucket: ${BUCKET}"
echo "Endpoint: oss-cn-beijing-internal.aliyuncs.com (internal)"
echo ""
echo "Quick test commands:"
echo "  rclone version"
echo "  rclone listremotes"
echo "  rclone lsf ${REMOTE_NAME}:${BUCKET}"
echo ""
echo "Standard upload:"
echo "  LOCAL_ROOT='/path/to/dataset'"
echo "  RELEASE_NAME='GRScenes-test1-YYYY-MM-DD'"
echo "  rclone copy \"\${LOCAL_ROOT}\" \"${REMOTE_NAME}:${BUCKET}/\${RELEASE_NAME}\" \\"
echo "    --transfers 16 --checkers 32 --stats 10s --progress"
echo ""
echo "IMPORTANT NOTES:"
echo "  1. Always use the INTERNAL endpoint (oss-cn-beijing-internal.aliyuncs.com)"
echo "     The public endpoint is blocked by IP whitelist."
echo "  2. If rclone hangs in scripts/agents, unset proxy variables:"
echo "     env -u HTTP_PROXY -u HTTPS_PROXY ... rclone ..."
echo "  3. Use 'copy' for uploads (not 'sync') to avoid deleting remote files."
echo "  4. Run 'rclone check' after upload to verify integrity."
echo ""
echo "Full runbook: docs/operations/grscenes_oss_rclone_runbook.md"
echo "================================================================================"
