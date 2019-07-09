FROM fedora:30

RUN dnf install -y gcc-c++ cmake ninja-build spdlog-devel fmt-devel && \
    ( \
        curl https://codeload.github.com/Z3Prover/z3/tar.gz/Z3-4.8.5 | tar -xzf - && \
        cd z3-Z3-4.8.5 && \
        mkdir build && \
        cd build && \
        cmake .. -DCMAKE_BUILD_TYPE=Release -G Ninja && \
        ninja && \
        ninja install && \
        cd ../.. && \
        rm -rf z3-Z3-4.8.5 \
    ) && \
    dnf clean all
