#! /bin/bash
rm -rf /etc/udev/rules.d/99-box-serial.rules
echo "ATTRS{serial}==\"0000:00:14.0\"  MODE:=\"0777\",SUBSYSTEM==\"tty\", SYMLINK+=\"ttyBOX1\"" > /etc/udev/rules.d/99-box-serial.rules