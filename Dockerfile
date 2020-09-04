FROM osrf/ros:noetic-desktop-full


#ENV PATH="/root/miniconda3/bin:${PATH}"
#ARG PATH="/root/miniconda3/bin:${PATH}"
RUN apt-get update
RUN apt-get install -y python3-pip
RUN apt-get install -y vim
#RUN apt-get install -y wget && rm -rf /var/lib/apt/lists/*

#RUN wget \
#    https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
#    && mkdir /root/.conda \
#    && bash Miniconda3-latest-Linux-x86_64.sh -b \
#    && rm -f Miniconda3-latest-Linux-x86_64.sh 



#RUN conda --version

#COPY environment.yml /data/
#RUN conda env create -f /data/environment.yml
# Make RUN commands use `bash --login`:
#RUN conda init bash

#SHELL ["conda", "run", "-n", "pybullet", "/bin/bash", "-c"]


#RUN conda activate pybullet
RUN pip3 install gym pybullet pyqtgraph pyopengl pyopengl-accelerate pyqt5 trimesh pyyaml 
RUN pip3 install pygame==2.0.0.dev6
#COPY * /data/
WORKDIR /data
CMD ["python3", "quadcopter_sim.py"]
