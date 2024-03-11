docker run --rm -it \
    -v $HOME/welt_auv:/welt_auv \
    --device=/dev/ttyTHS0 \
    --device=/dev/video0 \
    --net=host \
    welt_auv:latest \
    bash
