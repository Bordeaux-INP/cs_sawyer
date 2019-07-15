# Activate autostart on this machine
Autostart will autoselect the launchfile according to machine's hostname.

Please edit occurences of username `pi` of file `cs_sawyer.service` with the relevant username hosting the ROS workspace and then install: 
```
sudo cp cs_sawyer.service /lib/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable cs_sawyer.service
sudo cp power /etc/acpi/events/power
```

## Manage services at runtime

```
sudo service cs_sawyer status  # Quick view of the current service status, should be RUNNING
sudo journalctl -u cs_sawyer -f -a  # Detailed log output
sudo service cs_sawyer restart  # Restart service
```
