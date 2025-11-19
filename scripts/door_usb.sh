#! /bin/bash
rm -rf /etc/udev/rules.d/99-box-serial.rules
echo "ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"7524\",   MODE:=\"0777\",SUBSYSTEM==\"tty\", SYMLINK+=\"ttyBOX1\"" > /etc/udev/rules.d/99-box-serial.rules