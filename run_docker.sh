docker run --rm -it --gpus all \
    --device=/dev/video0 \
    --net=host \
    hydronautics/welt_auv:latest \
    bash
