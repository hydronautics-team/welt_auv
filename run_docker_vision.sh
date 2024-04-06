docker run --rm -it --gpus all \
    --ipc=host \
    --device="/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._H264_USB_Camera_SN0001-video-index0" \
    --device="/dev/v4l/by-id/usb-Microsoft_Microsoft®_LifeCam_HD-3000-video-index0" \
    --net=host \
    hydronautics/welt_auv:vision \
    bash
