# Activate autostart on this machine
Autostart will autoselect the launchfile according to machine's hostname.

Please edit occurences of username `pi` of file `cs_sawyer.service` with the relevant username hosting the ROS workspace and then install: 
```
sudo cp cs_sawyer.service /lib/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable cs_sawyer.service
```

