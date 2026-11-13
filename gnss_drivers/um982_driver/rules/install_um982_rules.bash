sudo cp ./99-um982.rules /etc/udev/rules.d
sudo udevadm control --reload-rules
sudo udevadm trigger