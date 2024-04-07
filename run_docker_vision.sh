docker run --rm -it --gpus all \
    -v $HOME/records:/welt_auv/records \
    --ipc=host \
    --device="/dev/video0" \
    --net=host \
    hydronautics/welt_auv:vision \
    bash
