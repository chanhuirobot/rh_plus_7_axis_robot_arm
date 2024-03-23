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
RUN git clone https://github.com/samsunglabs/tr3d.git /tr3d
WORKDIR /tr3d
# RUN sed -i 's/numba==0.53.0/numba==0.57.0/' requirements/runtime.txt
RUN pip install --no-cache-dir -e .

# Install MinkowskiEngine dependencies
RUN apt-get -y install libopenblas-dev
RUN apt-get install -y python3 python3-pip g++-7 libopenblas-dev

# Clone and install MinkowskiEngine
RUN git clone https://github.com/NVIDIA/MinkowskiEngine.git /minkowskiengine
WORKDIR /minkowskiengine
RUN python setup.py install --force_cuda --blas=openblas

# build and result show depend library
RUN pip install yapf==0.40.0
RUN pip install open3d
