#include "welt_communication/messages/welt.h"

WeltMessage::WeltMessage() : AbstractMessage() {
    reset_imu = 0;
    stab_depth = 0;
    stab_roll = 0;
    stab_pitch = 0;
    stab_yaw = 0;
    surge = 0.0;
    sway = 0.0;
    depth = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    surge_accel = 0.0;
    sway_accel = 0.0;
    for (int i = 0; i < dev_amount; i++) {
        dev[i] = 0;
    }
    for (int i = 0; i < flare_amount; i++) {
        flare_seq[i] = 0;
    }
    modem_distance = 0.0;

    checksum = 0;
}

// form byte-vector (raspberry_cm4 to STM) // TODO
void WeltMessage::pack(std::vector<uint8_t> &container) {
    pushToVector(container, reset_imu);
    pushToVector(container, stab_depth);
    pushToVector(container, stab_roll);
    pushToVector(container, stab_pitch);
    pushToVector(container, stab_yaw);
    pushToVector(container, surge);
    pushToVector(container, sway);
    pushToVector(container, depth);
    pushToVector(container, roll);
    pushToVector(container, pitch);
    pushToVector(container, yaw);
    pushToVector(container, surge_accel);
    pushToVector(container, sway_accel);
    for (int i = 0; i < dev_amount; i++) {
        pushToVector(container, dev[i]);
    }
    for (int i = 0; i < flare_amount; i++) {
        pushToVector(container, flare_seq[i]);
    }
    pushToVector(container, modem_distance);

    uint16_t checksum = getChecksum16b(container);
    pushToVector(container, checksum);  // do i need to revert bytes here?
}

// pull message from byte-vector (pult to raspberry_cm4)
bool WeltMessage::parse(std::vector<uint8_t> &input) {
    popFromVector(input, checksum, true);
    uint16_t checksum_calc = getChecksum16b(input);
    // if (checksum_calc != checksum) {
    //     return false;
    // }
    popFromVector(input, modem_distance);
    for (int i = 0; i < flare_amount; i++) {
        popFromVector(input, flare_seq[flare_amount - i]);
    }
    for (int i = 0; i < dev_amount; i++) {
        popFromVector(input, dev[dev_amount - i]);
    }
    popFromVector(input, sway_accel);
    popFromVector(input, surge_accel);
    popFromVector(input, yaw);
    popFromVector(input, pitch);
    popFromVector(input, roll);
    popFromVector(input, depth);
    popFromVector(input, sway);
    popFromVector(input, surge);
    popFromVector(input, stab_yaw);
    popFromVector(input, stab_pitch);
    popFromVector(input, stab_roll);
    popFromVector(input, stab_depth);
    popFromVector(input, reset_imu);

    return true;
}