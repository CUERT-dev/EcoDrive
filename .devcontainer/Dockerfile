# Start with Ubuntu
FROM mcr.microsoft.com/devcontainers/base:noble

# Switch to root to install everything
USER root

# Install Python
RUN apt-get update 
RUN apt-get install -y \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install CMake and build tools
RUN apt-get update
RUN apt-get install -y cmake
RUN apt-get install -y build-essential
RUN apt-get install -y usbutils
RUN rm -rf /var/lib/apt/lists/*

# Install ARM toolchain
RUN apt-get update
RUN apt-get install -y gcc-arm-none-eabi
RUN apt-get install -y gdb-arm-none-eabi 
RUN apt-get install -y openocd
RUN  rm -rf /var/lib/apt/lists/*



# Install PyOCD for CMSIS-DAP support and gdbgui
RUN apt-get update && apt-get install -y pipx
RUN pipx install pyocd
RUN pipx install gdbgui 

# Create symlink for consistent openocd access
RUN ln -sf /usr/bin/openocd /usr/local/bin/openocd

# Add user to dialout group for USB access
RUN usermod -a -G dialout vscode

# Switch back to vscode user
USER vscode