cd ~/Downloads
wget "https://www.baslerweb.com/fp-1551786516/media/downloads/software/pylon_software/pylon-5.2.0.13457-x86_64.tar.gz"
tar xvzf pylon-5.2.0.13457-x86_64.tar.gz
cd pylon-5.2.0.13457-x86_64
sudo tar -C /opt -xzf pylonSDK*.tar.gz
rm -r pylon-5.2.0.13457-x86_64
rm pylon-5.2.0.13457-x86_64.tar.gz
sudo sh -c 'echo "yaml https://raw.githubusercontent.com/magazino/pylon_camera/indigo-devel/rosdep/pylon_sdk.yaml " > /etc/ros/rosdep/sources.list.d/15-plyon_camera.list'
rosdep update
