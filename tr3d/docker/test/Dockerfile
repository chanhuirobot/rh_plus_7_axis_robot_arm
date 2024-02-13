ARG PYTORCH="1.12.0"
ARG CUDA="11.3"
ARG CUDNN="8"

FROM pytorch/pytorch:${PYTORCH}-cuda${CUDA}-cudnn${CUDNN}-devel

ENV CUDA_HOME=/usr/local/cuda-11.3
ENV TORCH_CUDA_ARCH_LIST="7.5"
ENV CC=g++-7
ENV DEBIAN_FRONTEND=noninteraective

RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub \
    && apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/7fa2af80.pub \
    && apt-get update \
    && apt-get install -y ffmpeg libsm6 libxext6 git ninja-build libglib2.0-0 libsm6 libxrender-dev libxext6

# Install MMCV, MMDetection and MMSegmentation
RUN pip install mmcv-full==1.6.0 -f https://download.openmmlab.com/mmcv/dist/cu113/torch1.12.0/index.html
RUN pip install mmdet==2.24.1
RUN pip install mmsegmentation==0.24.1

# Install MMDetection3D
RUN git clone https://github.com/samsunglabs/tr3d.git /mmdetection3d
WORKDIR /mmdetection3d
# RUN sed -i 's/numba==0.53.0/numba==0.57.0/' requirements/runtime.txt
RUN pip install --no-cache-dir -e .

# Version check
# ENV LC_ALL=C.UTF-8
# ENV LANG=C.UTF-8
RUN apt install -y nvidia-driver-470 && apt update -y && apt upgrade -y
# RUN pip list | grep torch && cat /usr/include/cudnn_version.h | grep CUDNN_MAJOR -A 2 && nvcc --version && \
#     apt --installed list | grep nvidia-driver && cat /proc/driver/nvidia/version && nvidia-smi

# ENV FORCE_CUDA = "1"
# # Install MinkowskiEngine
# RUN apt-get -y install libopenblas-dev
# RUN pip install -U git+https://github.com/NVIDIA/MinkowskiEngine@@405b39cb7e68c2ec1b4ac8c4b4ca9fa6fcfcb2fc -v --no-deps \
#     --install-option="--blas=openblas" \
#     --install-option="--force_cuda"

# Install MinkowskiEngine dependencies
RUN apt-get -y install libopenblas-dev
RUN apt-get install -y python3 python3-pip g++-7 libopenblas-dev
# RUN python3 -m pip install torch numpy ninja
# Install g++-7
# RUN apt-get install -y software-properties-common
# RUN add-apt-repository ppa:ubuntu-toolchain-r/test
# RUN apt-get update
# RUN apt-get install -y g++-7
# RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 60 --slave /usr/bin/g++ g++ /usr/bin/g++-7

# Clone and install MinkowskiEngine
RUN git clone https://github.com/NVIDIA/MinkowskiEngine.git /minkowskiengine
WORKDIR /minkowskiengine

# 필요한 패키지를 설치하고 MinkowskiEngine을 빌드하여 설치합니다.
RUN python setup.py install --force_cuda --blas=openblas
