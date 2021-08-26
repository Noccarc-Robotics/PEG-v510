Documentation :

1. Make a new folder in : /usr/local called /scripts.
2. Paste the counter.sh in that folder.
3. Copy the encoder.service file to /lib/systemd/system
4. Provide executable permission to counter.sh 
5. Provide permission to encoder.service file using chmod 0644 encoder.service.
6. Restart the system daemon : systemctl daemon-reload
7. Enable the service : systemctl enable encoder.service.
8. Reboot the system to see changes.
