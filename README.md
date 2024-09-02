# mower
This is the installation files for version 2 of my automated robot mower. Learn more at the youtube link here:

To set up your raspberry pi zero 2 w

If you log in via ssh and it hangs, improve ssh response times with this command and then reboot: echo "IPQoS cs0 cs0" | sudo tee -a /etc/ssh/sshd_config
install git on your raspberry pi: sudo apt-get -y install git
clone this git repository to your pi home directory: git clone https://github.com/NathanBuildsDIY/mower
Run initial setup: nohup sh weeder/initial-setup.sh & Note - you can watch the output with: tail -f nohup.out
Connect to the new wifi hostpot called mower. Visit http://mower.local to control your mower.  You can still ssh to the mower (while connected to the mower wifi) via ssh or putty, hostname mower.local
