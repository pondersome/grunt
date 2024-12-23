#!/bin/bash

# set up consistent aliases for USB devices across boot
# Path to the udev rules file
UDEV_RULES_FILE="/etc/udev/rules.d/99-grunt-usb-serial.rules"

# Define the udev rules to be appended
UDEV_RULES_CONTENT="
# udev rules for assigning consistent names to robot USB devices

# Device 1: P3AT Chassis ARCOS microcontroller on a serial-to-usb cable
SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6001\", ATTRS{serial}==\"FTDJPBGV\", SYMLINK+=\"grunt_p3at\"

# Device 2: u-blox GNSS receiver primary interface - Sparkfun or u-blox C099
SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"1546\", ATTRS{idProduct}==\"01a9\", SYMLINK+=\"grunt_f9p\"

# Device 3: GPS Receiver - u-blox ZED-F9P UART2 if enabled - works on C099 board
SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"1546\", ATTRS{idProduct}==\"0502\", ATTRS{serial}==\"DBTQ8VRU\", SYMLINK+=\"grunt_f9p_p2\"

# Device 4: GPS Receiver u-blox C099 with ODIN-W2 wireless interface - not actually used
SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"1546\", ATTRS{idProduct}==\"0503\", ATTRS{serial}==\"DBTQ8VS3\", SYMLINK+=\"grunt_f9p_odin\"

# Device 5: WaveShare RoArm running from their Genereral Driver for Robots board
SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", ATTRS{serial}==\"3a96bedc64e4ee11820c777d4c11a646\", SYMLINK+=\"grunt_arm\"


# **NOT WORKING ** Device 6: Joystick: Gamesir Neo Lite gamepad - activate with home button + X
#SUBSYSTEM=="input", ATTRS{idVendor}=="3537", ATTRS{idProduct}=="1041", SYMLINK+="input/grunt_joystick"
KERNEL=="js?", ENV{ID_VENDOR}=="Zikway", ENV{ID_MODEL}=="HID_gamepad", SYMLINK+="input/grunt_joystick"
"

# Remove the existing udev rules file if it exists
if [ -f "$UDEV_RULES_FILE" ]; then
    sudo rm "$UDEV_RULES_FILE"
    echo "Existing udev rules file removed."
fi

# Append the rules to the file
echo "$UDEV_RULES_CONTENT" | sudo tee -a "$UDEV_RULES_FILE" > /dev/null

# Reload udev rules and apply changes
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "udev rules added and reloaded successfully."
