FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Install common command-line utilities and sudo for the development user.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        git \
        git-lfs \
        vim \
        unzip \
        bash-completion \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# Install the C++ build toolchain.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        g++ \
        gcc \
        gdb \
        clang \
        clang-format \
        make \
        ninja-build \
        pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Install Qt development tools.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        qtbase5-dev \
        qtchooser \
        qt5-qmake \
        qtbase5-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Install C++ libraries that may be used by the project.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libtbb-dev \
        nlohmann-json3-dev \
        libceres-dev \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python and its development/package-management tools.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3 \
        python3-dev \
        python3-pip \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Install Python scientific packages in dependency order.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-numpy \
        python3-scipy \
        python3-pandas \
        python3-matplotlib \
        python3-sklearn \
        python3-plotly \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home "${USERNAME}" \
    && printf '%s ALL=(ALL) NOPASSWD:ALL\n' "${USERNAME}" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

USER ${USERNAME}