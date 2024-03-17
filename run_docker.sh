docker run --rm -it \
    -v $HOME/welt_auv:/welt_auv \
    --device=/dev/video0 \
    --net=host \
    welt_auv:latest \
    bash
