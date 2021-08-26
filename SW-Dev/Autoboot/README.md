Documentation :

1. Make a new folder in : /usr/local called /scripts if exists just utilize it.
2. Paste the ui_launch.sh in that folder.
3. Copy the weston.ini file to /etc/xdg/weston/.
4. Copy the ui.service file to /lib/systemd/system/.
5. Provide executable permission to ui_launch.sh 
6. Provide permission to ui.service file using chmod 0644 ui.service.
7. Restart the system daemon : systemctl daemon-reload
8. Enable the service : systemctl enable ui.service.
9. Reboot the system to see changes.
