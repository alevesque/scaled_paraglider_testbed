cd ~/paraglider
mv UCSD-PROTECTED.config /var/lib/connman
connmanctl enable wifi
connmanctl scan wifi
connmanctl agent on
connmanctl connect wifi_e84e063528df_554353442d50524f544543544544_managed_ieee8021x
apt-get update
apt-get -y upgrade
apt-get install libconfig8-dev
configure_robotics_dt.sh
rc_calibrate_gyro
reboot
