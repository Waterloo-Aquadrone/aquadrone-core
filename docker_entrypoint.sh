#!/bin/bash
#set -v

# From UW Robohub

set -e
shopt -s nullglob

# check if LOCAL_USER_NAME is present, otherwise just run a shell here and now
if [ -z "$LOCAL_USER_NAME" ]; then
    echo "\$LOCAL_USER_NAME not specified, giving you a root shell"
    bash
    return
fi
set +e
if ! grep -q $LOCAL_USER_NAME /etc/passwd; then
    set -e
    # this is ran once at init
    addgroup --quiet --gid $LOCAL_GROUP_ID $LOCAL_GROUP_NAME
    # Is putting home in a non-standard location really useful
    # Could also mimic the path on the host system
    adduser --quiet --gecos "" --disabled-password --uid $LOCAL_USER_ID --gid $LOCAL_GROUP_ID $LOCAL_USER_NAME
    usermod -aG tty $LOCAL_USER_NAME
    usermod -aG video $LOCAL_USER_NAME
    # enable password-less sudo
    echo "$LOCAL_USER_NAME   ALL=(ALL:ALL) NOPASSWD: ALL" > /etc/sudoers.d/docker
    # bashrc needs to source this file. This enables
    if [ ! -z $SSH_AUTH_SOCK ]; then
        echo "SSH_AUTH_SOCK=$SSH_AUTH_SOCK" >> /etc/environment
    fi
    # execute hooks in /etc/robohub/docker_hooks/creation.d/
    for file in /etc/robohub/docker_hooks/creation.d/*
    do
        source $file
    done
fi
set -e

# fix tty permissions so user can run screen
chmod g+r /dev/pts/0

# fix access to /dev/video*
for file in /dev/video*; do
	chmod a+rwx $file
done
# allow graphics card access
for file in /dev/dri/*; do
	chmod a+rwx $file
done

# show ip address on stdout
echo "Container IPs: $(hostname --all-ip-addresses)"
    
# TODO: execute hooks in /etc/robohub/docker_hooks/run.d/
for file in /etc/robohub/docker_hooks/run.d/*
do
    echo "Running" $file
    source $file
done

# run command as user if specified
# su strips nearly all environment variables
if [[ ! -z "${@}" ]]; then
	echo $@ > /tmp/.runscript.sh
	chmod +x /tmp/.runscript.sh
	su -c /tmp/.runscript.sh -l $LOCAL_USER_NAME
else
	su -l $LOCAL_USER_NAME
fi

source /opt/ros/kinetic/setup.bash
exec "$@"