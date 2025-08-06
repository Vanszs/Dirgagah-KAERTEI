#!/bin/bash
# KAERTEI 2025 FAIO - Just Command Runner Installer for Ubuntu 22.04
# ===================================================================

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}🔧 KAERTEI 2025 - Installing Just Command Runner${NC}"
echo -e "${BLUE}=================================================${NC}"
echo ""

# Check if Just is already installed
if command -v just >/dev/null 2>&1; then
    echo -e "${GREEN}✅ Just is already installed${NC}"
    just --version
    exit 0
fi

# Install Just command runner
echo -e "${YELLOW}📦 Installing Just command runner...${NC}"

# Create ~/bin directory if it doesn't exist
mkdir -p ~/bin

# Download and install Just
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/bin

# Add ~/bin to PATH in bashrc if not already there
if ! grep -q 'export PATH="$HOME/bin:$PATH"' ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# Just command runner" >> ~/.bashrc
    echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc
    echo -e "${GREEN}✅ Added ~/bin to PATH in bashrc${NC}"
fi

# Export PATH for current session
export PATH="$HOME/bin:$PATH"

# Verify installation
if command -v just >/dev/null 2>&1; then
    echo -e "${GREEN}✅ Just installed successfully!${NC}"
    just --version
    echo ""
    echo -e "${YELLOW}🎯 Usage:${NC}"
    echo "   just                    # Show quick start"
    echo "   just setup              # Complete Ubuntu setup"
    echo "   just mission-debug      # Run debug mission"
    echo "   just help               # Show all commands"
    echo ""
    echo -e "${YELLOW}⚠️  Note:${NC} Please restart your terminal or run:"
    echo -e "   ${BLUE}source ~/.bashrc${NC}"
else
    echo -e "${RED}❌ Just installation failed${NC}"
    exit 1
fi
