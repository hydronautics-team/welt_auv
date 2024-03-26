docker run --rm -it --gpus all \
    --ipc=host \
    --device=/dev/video0 \
    --net=host \
    -v ~/hydronautics/welt_auv:/welt_auv \
    hydronautics/welt_auv:vision \
    bash
