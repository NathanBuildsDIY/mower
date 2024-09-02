echo "get up to date packages with apt-get"
sudo apt-get update
sudo apt-get -y upgrade

#echo "install new packages - various stuff to run waterer script"
#sudo apt-get -y install pip
#sudo apt-get -y install python3
#sudo apt-get -y remove vim-tiny
#sudo apt-get -y install vim

#echo "get pigpio daemon running so we have better servo control"
sudo apt-get -y install pigpio python-pigpio python3-pigpio
sudo apt-get -y install pigpiod
pip3 install pigpio
sudo systemctl enable pigpiod #makes it run at startup (after reboot - sudo shutdown -r now. Check with pigs t)

echo "set up dnsmask to do dns resolution on our wifi hotspot"
sudo apt-get -y install dnsmasq
sudo apt-get -y install dnsutils
#NOTE: I thought dnsmask woudl work like this:
#sudo vi /etc/dnsmasq.conf  Add domain-needed; bogus-priv; expand-hosts; domain=connect.local; address=/#/10.42.0.1; 
#but once wifi hotspot is actually set up we instead config in this file below
echo 'address=/.local/10.42.0.1' | sudo tee -a /etc/NetworkManager/dnsmasq-shared.d/hosts.conf

echo "create entry in crontab to always run rain app on startup"
line="@reboot /usr/bin/python3 -m flask --app ~/mower/mower_v1p6.py run --host=0.0.0.0 >> ~/mower/log.out 2>&1"

(crontab -u $(whoami) -l; echo "$line" ) | crontab -u $(whoami) -
touch ~/mower/log.out

echo "Install flask and associated forms packages"
pip3 install -U Flask 
pip3 install WTForms
pip3 install Flask-WTF
pip3 install flask_autoindex
pip3 install psutil
pip3 install python-crontab

echo "set up iptables and rules"
line="@reboot /usr/bin/sh /home/$(whoami)/rain/iptables.rules"
sudo sh -c "(crontab -l; echo $line) | sort - | uniq | crontab -"

echo "setup local wifi hotspot"
sudo systemctl stop dhcpcd
sudo systemctl disable dhcpcd
sudo systemctl enable NetworkManager 
sudo service NetworkManager start
sleep 20
sudo nmcli device wifi hotspot ssid mower password LetsMower ifname wlan0
UUID=$(nmcli connection | grep Hotspot | tr -s ' ' | cut -d ' ' -f 2)
sudo nmcli connection modify $UUID connection.autoconnect yes connection.autoconnect-priority 100

echo "restarting for changes to take effect. Still must run wifi setup"
sudo shutdown -r now
