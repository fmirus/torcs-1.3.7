set -e

make -j4
sudo make install -j4
# cp ~/torcs/torcs-1.3.7/standard_settings/quickrace.xml ~/.torcs/config/raceman/quickrace.xml
cp ~/torcs/torcs-1.3.7/standard_settings/quickrace_ros.xml ~/.torcs/config/raceman/quickrace.xml
cp ~/torcs/torcs-1.3.7/standard_settings/screen.xml ~/.torcs/config/screen.xml
cp ~/torcs/torcs-1.3.7/standard_settings/sound.xml ~/.torcs/config/sound.xml
torcs
