# Use a specific Debian version for reproducibility
FROM ubuntu:22.04 AS build-env

# 1. Install prerequisites for building the application
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# 2. Copy the entire project structure into the container
WORKDIR /clone
COPY . .

# 3. Initialize submodules inside the container
# This ensures the build works even if the user forgets --recurse-submodules
RUN git config --global --add safe.directory '*' && \
    git submodule update --init --recursive

# 4. Configure and build the project using CMake
WORKDIR /clone/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN cmake --build . && ls -l /clone/build/bin

# 5. Create the final, lightweight runtime image
FROM ubuntu:22.04

# Install only the runtime dependency (socat)
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y socat && rm -rf /var/lib/apt/lists/*

# Copy the compiled executables from the build stage
WORKDIR /clone
COPY --from=build-env /clone/build/bin/emulator .
COPY --from=build-env /clone/build/bin/consumer .

# The docker-compose.yml file will provide the entrypoint command
