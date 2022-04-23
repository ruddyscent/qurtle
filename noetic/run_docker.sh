set -exu
xhost +local:docker
docker run -it --net=host \
	--volume=/tmp/.X11-unix:/tmp/.X11-unix \
	--device=/dev/dri:/dev/dri \
	--env="DISPLAY=$DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	${1} bash -c "${2}"
