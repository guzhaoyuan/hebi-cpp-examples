#!/usr/bin/zsh
sshpass -p "hebi1234" scp -r ~/sy_ws/hebi-cpp-examples/kits/titan6/src hebi@10.10.1.2:~/hebi-cpp-examples/kits/titan6
sshpass -p "hebi1234" scp -r ~/sy_ws/hebi-cpp-examples/kits/titan6/resources/* hebi@10.10.1.2:~/hebi-cpp-examples/kits/titan6/resources/
sshpass -p "hebi1234" scp -r ~/sy_ws/hebi-cpp-examples/kits/titan6/build/gains18.xml hebi@10.10.1.2:~/hebi-cpp-examples/kits/titan6/build/
