#!/usr/bin/zsh
sshpass -p "hebi1234" scp -r ~/hebi-cpp-examples/kits/quad/CMakeLists.txt hebi@10.10.1.2:~/hebi-cpp-examples/kits/quad/
sshpass -p "hebi1234" scp -r ~/hebi-cpp-examples/kits/quad/src hebi@10.10.1.2:~/hebi-cpp-examples/kits/quad/
sshpass -p "hebi1234" scp -r ~/hebi-cpp-examples/kits/quad/resources/* hebi@10.10.1.2:~/hebi-cpp-examples/kits/quad/resources/
sshpass -p "hebi1234" scp -r ~/hebi-cpp-examples/kits/quad/resources/quad_gains18.xml hebi@10.10.1.2:~/hebi-cpp-examples/kits/quad/build
