cp devmem2 /usr/local/
cp sn65dsi83_dsi.sh /usr/local/
cp taj.jpg /usr/local/
cp display.service /etc/systemd/system/
cp weston.ini /etc/xdg/weston/
cp stm32mp157a-visionsom-dsi-emmc-mx.dtb /boot/
chmod +x /usr/local/devmem2
chmod +x /usr/local/sn65dsi83_dsi.sh
systemctl enable display.service
systemctl restart display.service

