## Quadruped Reinforce Learning Simulator    
本项目旨在构建一个通用的纯上位机机器人控制框架方便进行强化学习或传统运控等方式的快速接入  

runs on Ubuntu 22.04   x86_64

dependencies:    
[FastDDS](https://fast-dds.docs.eprosima.com/en/stable/02-formalia/titlepage.html)


## Cmd: 
```bash
#构建消息类型
cmake --build build --target generate_dds_types   
```