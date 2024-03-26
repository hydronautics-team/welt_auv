docker run --rm -it --gpus all \
    --ipc=host \
    --net=host \
    -v ~/hydronautics/welt_auv:/welt_auv \
    hydronautics/welt_auv:control \
    bash
