sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

sudo ifconfig enx302303060f7a down
sudo ifconfig enx302303060f7a up 192.168.123.162 netmask 255.255.255.0

#enx302303060f7aのところは各自変更(ロボットとpcを有線でつないで[$ ifconfig -a]と端末で打てば出ると思います。)

ifconfig

ping 192.168.123.161
