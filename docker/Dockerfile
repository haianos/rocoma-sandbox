## Enea Scioni <enea.scioni@gmail.com>
## Dockerfile for distribution Anybotics example


from osrf/ros:kinetic-desktop-full-xenial

# enabling sudo
RUN echo nameserver 134.58.127.1 > /etc/resolve.conf && apt-get update && apt-get -y install sudo lua5.3 liblua5.3-dev wget git ssh unzip doxygen

# defining a user and giving him sudo privileges:
# ENV = available in dependend Dockerfiles, ARG = not available.
ARG UNAME=user
ARG UPASSWD=user
ARG UID=1000
ARG GID=1000
ENV UNAME ${UNAME} 
RUN groupadd -g $GID $UNAME &&\
    useradd -m -u $UID -g $GID -s /bin/bash $UNAME &&\
    echo "$UNAME:$UPASSWD" | chpasswd && adduser $UNAME sudo
#RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# switching to user and creating user directory
USER $UNAME
RUN mkdir -p "/home/${UNAME}"
RUN mkdir -p "/home/${UNAME}/.cache/luarocks"
RUN mkdir -p "/home/${UNAME}/ws/src"
WORKDIR "/home/$UNAME"
ENV WORKDIR "/home/$UNAME/ws"

# Fetching and installing sources
RUN cd ${WORKDIR}/src &&\
    git clone https://github.com/ANYbotics/kindr &&\
    git clone https://github.com/ANYbotics/kindr_ros &&\
    git clone https://github.com/ANYbotics/roco &&\
    git clone https://github.com/ANYbotics/rocoma &&\
    git clone https://github.com/ANYbotics/rocoma_example &&\
    git clone https://github.com/ANYbotics/signal_logger &&\
    git clone https://github.com/ANYbotics/message_logger &&\
    git clone https://github.com/ANYbotics/any_node &&\
    git clone https://github.com/ANYbotics/variant &&\
    git clone https://github.com/haianos/rocoma-sandbox

RUN . /opt/ros/kinetic/setup.sh &&\
    cd ${WORKDIR}/src && catkin_init_workspace &&\
    cd ${WORKDIR} && catkin_make_isolated

    