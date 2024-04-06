docker run --rm -it --gpus all \
    -v $HOME/welt_auv:/welt_auv \
    --device=/dev/ttyUSB0 \
    --ipc=host \
    --net=host \
    hydronautics/welt_auv:control \
    bash
