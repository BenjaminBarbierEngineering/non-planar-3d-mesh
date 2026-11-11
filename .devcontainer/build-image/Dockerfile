FROM ubuntu:jammy-20240111

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Install common command-line utilities and sudo for the development user.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        bash \
        git \
        git-lfs \
        vim \
        wget \
        tree \
        file \
        unzip \
        bash-completion \
        sudo \
        net-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Python and its development, package-management, and scientific tools.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3 \
        python3-dev \
        python3-pip \
        python3-venv \
        python3-numpy \
        python3-scipy \
        python3-pandas \
        python3-matplotlib \
        python3-sklearn \
        python3-plotly \
    && rm -rf /var/lib/apt/lists/*

# Install the C++/CMake build toolchain and project libraries.
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
        doxygen \
        pkg-config \
        libtbb-dev \
        nlohmann-json3-dev \
        libceres-dev \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install UI tools and Qt development tools.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        dbus-x11 \
        terminator \
        gnome-terminal \
        x11-apps \
        qtbase5-dev \
        qtchooser \
        qt5-qmake \
        qtbase5-dev-tools \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}" \
    && printf '%s ALL=(ALL) NOPASSWD:ALL\n' "${USERNAME}" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

USER ${USERNAME}