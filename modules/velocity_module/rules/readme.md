usage: 
```
sudo cp 01-drive_teensy.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```

This rules would make a SYMBOLIC port named /dev/teensy_drive -> /dev/ttyACM*.
