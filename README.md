# WiL_project_amr_ros2

```
git clone -b ros2 https://github.com/SunnyApp-Robotics/serial.git
```

### activate tty port on ubuntu command 
> note: you need to connect the usb first to able to run this command 
```
sudo chmod 666 /dev/ttyACM0
```

### fix usb name
### see usb detail : udevadm info --name=/dev/ttyACM0 –attribute-walk

sudo nano /etc/udev/rules.d/99-usb-serial.rules

add:
SUBSYSTEM=="tty", ATTRS{idVendor}=="20d2", ATTRS{idProduct}=="5740", SYMLINK+="drive"

### reflesh

sudo udevadm trigger

### port permission

sudo bash -c 'echo "KERNEL==\"ttyUSB*\",MODE=\"0666\"" >> /etc/udev/rules.d/66-tty.rules'

sudo adduser facobot dialout

sudo usermod -a -G dialout facobot
