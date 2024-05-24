# Official SOFA dockerfile for Ubuntu
FROM sofaframework/sofabuilder_ubuntu

# Install dependencies
RUN apt-get update && apt-get install -y \
    openssh-server \
    xauth \
    x11-apps \
    xorg \
    xterm \
    && rm -rf /var/lib/apt/lists/*

# Configure SSH server
RUN mkdir /var/run/sshd
RUN echo 'root:password' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#X11Forwarding yes/X11Forwarding yes/' /etc/ssh/sshd_config
RUN sed -i 's/#X11UseLocalhost yes/X11UseLocalhost no/' /etc/ssh/sshd_config

# Expose SSH port
EXPOSE 22


# # Install additional dependencies if needed
# RUN apt-get update && apt-get install -y \
#     x11-apps \
#     && rm -rf /var/lib/apt/lists/*

# # TODO: Install gurobi and move license into container

# Copy prebuilt SOFA binaries into the container
COPY ./SOFA_v23 /opt/sofa
# Set environment variables
ENV SOFA_ROOT="/opt/sofa"
ENV PYTHONPATH="/opt/sofa/plugins/SofaPython3/lib/python3/site-packages:$PYTHONPATH"
ENV SOFAPYTHON3_ROOT="/opt/sofa/plugins/SofaPython3/lib/python3/site-packages"

# Copy local files into the container
COPY ./src /app/src
COPY ./meshes /app/meshes

# Set the working directory
WORKDIR /app

# # Install Python dependencies
# RUN pip install -r requirements.txt

# # Start with an interactive shell
CMD ["/bin/bash"]
# Start the SSH service
# CMD ["/usr/sbin/sshd", "-D"]

