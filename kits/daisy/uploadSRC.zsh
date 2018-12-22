#!/usr/bin/zsh
sshpass -p "hebi1234" scp -r ~/daisy_ws/src/hebi-cpp-examples/kits/daisy/src hebi@10.10.1.2:~/hebi-cpp-examples/kits/daisy/
sshpass -p "hebi1234" scp -r ~/daisy_ws/src/hebi-cpp-examples/kits/daisy/resources/* hebi@10.10.1.2:~/hebi-cpp-examples/kits/daisy/resources/
sshpass -p "hebi1234" scp -r ~/daisy_ws/src/hebi-cpp-examples/kits/daisy/build/gains18.xml hebi@10.10.1.2:~/hebi-cpp-examples/kits/daisy/build/
