# Activate autostart on this machine
Autostart will autoselect the launchfile according to machine's hostname
```
sudo cp cs_sawyer.service /lib/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable cs_sawyer.service
```

