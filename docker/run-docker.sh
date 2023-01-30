#!/bin/bash
function simulation_main() {
    docker build --rm -f Dockerfile -t "fleet-adapter" ..
    docker rm $(docker ps --filter status=exited -q)
    docker rmi $(docker images -f "dangling=true" -q)
       docker run \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --device=/dev/dri \
        --group-add video \
        --device=/dev/snd:/dev/snd \
        --group-add audio \
        --net=host \
        --privileged \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --name "fleet-adapter_ins" \
        fleet-adapter
}
simulation_main "$@"
exit $?
