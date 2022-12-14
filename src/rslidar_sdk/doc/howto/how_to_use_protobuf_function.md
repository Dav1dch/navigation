# How to use protobuf functions to send & receive message

## 1 Introduction

Suppose there are two computers, PC-A and PC-B, and they are far away from each other. For some reasons, you connect LiDAR with PC-A and use point cloud message in PC-B. To achieve this, you may need to use the protobuf functions. Typically, there are two ways to do this.

- PC-A send out the packets message to PC-B. PC-B receive the packet message and decode it , then PC-B get the point cloud message and use it.

- PC-A decode the packets message, get the point cloud and send out the point cloud message to PC-B. PC-B receive the point cloud message and use it directly.

This first way is recommended, because the point cloud message is very large and it may take up your bandwidth.  

## 2 Send & Receive packets through Protobuf-UDP

You are supposed to have already read [Intro to parameters](../intro/parameter_intro.md) and already have a basic idea about the config file. 

### 2.1 PC-A(Sender)

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: false                                
  send_point_cloud_ros: false                            
  send_packet_proto: true                              
  send_point_cloud_proto: false                         
```

Since the message come from the LiDAR, set ```msg_source = 1```. 

Send packets through Protobuf-UDP, so set ```send_packet_proto = true```.

```yaml
lidar:
  - driver:
      lidar_type: RS128           
      msop_port: 6699             
      difop_port: 7788            
      start_angle: 0              
      end_angle: 360               
      min_distance: 0.2            
      max_distance: 200            
      use_lidar_clock: false        
    ros:
      ros_frame_id: /rslidar           
      ros_recv_packet_topic: /rslidar_packets    
      ros_send_packet_topic: /rslidar_packets    
      ros_send_point_cloud_topic: /rslidar_points      
    proto:
      point_cloud_recv_port: 60021                     
      point_cloud_send_port: 60021                     
      msop_recv_port: 60022                       
      msop_send_port: 60022                       
      difop_recv_port: 60023                      
      difop_send_port: 60023       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1  
```

Set the ```lidar_type``` for your LiDAR.

Set the ```msop_port``` and ```difop_port``` for your LiDAR. 

Set the ```msop_send_port```, ```difop_send_port```, and ```packet_send_ip```.

### 2.2 PC-B(Receiver)

```yaml
common:
  msg_source: 4                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```

Since the packets message come from protobuf-UDP, set ```msg_source = 4```.

Send point cloud to ROS, so set ```send_point_cloud_ros = true```. 

```yaml
lidar:
  - driver:
      lidar_type: RS128           
      msop_port: 6699             
      difop_port: 7788            
      start_angle: 0              
      end_angle: 360               
      min_distance: 0.2            
      max_distance: 200            
      use_lidar_clock: false        
    ros:
      ros_frame_id: /rslidar           
      ros_recv_packet_topic: /rslidar_packets    
      ros_send_packet_topic: /rslidar_packets    
      ros_send_point_cloud_topic: /rslidar_points      
    proto:
      point_cloud_recv_port: 60021                     
      point_cloud_send_port: 60021                     
      msop_recv_port: 60022                       
      msop_send_port: 60022                       
      difop_recv_port: 60023                      
      difop_send_port: 60023       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1  
```

Set the ```lidar_type``` for your LiDAR type.

Make sure the PC-B's ip address is same with the ```packet_send_ip``` set in PC-A's config.yaml.

Set the ```msop_recv_port``` and  ```difop_recv_port``` to be same with the  ```msop_send_port``` and  ```difop_send_port``` set in PC-A's config.yaml.

## 3 Send & receive point cloud through Protobuf-UDP

You are supposed to have already read [Intro to parameters](../intro/parameter_intro.md) and have a basic idea about the config file. 

### 3.1 PC-A(Sender)

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: false                                
  send_point_cloud_ros: false                            
  send_packet_proto: false                              
  send_point_cloud_proto: true                         
```

Since the message come from the LiDAR, set ```msg_source = 1```. 

Send point cloud through Protobuf-UDP, so set ```send_point_cloud_proto = true```.

```yaml
lidar:
  - driver:
      lidar_type: RS128           
      msop_port: 6699             
      difop_port: 7788            
      start_angle: 0              
      end_angle: 360               
      min_distance: 0.2            
      max_distance: 200            
      use_lidar_clock: false        
    ros:
      ros_frame_id: /rslidar           
      ros_recv_packet_topic: /rslidar_packets    
      ros_send_packet_topic: /rslidar_packets    
      ros_send_point_cloud_topic: /rslidar_points      
    proto:
      point_cloud_recv_port: 60021                     
      point_cloud_send_port: 60021                     
      msop_recv_port: 60022                       
      msop_send_port: 60022                       
      difop_recv_port: 60023                      
      difop_send_port: 60023       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1  
```

Set the ```lidar_type``` for your LiDAR.

Set the ```msop_port``` and ```difop_port```  for your LiDAR. 

Set the ```point_cloud_send_port``` and ```point_cloud_send_ip``.

### 3.2 PC-B(Receiver)

```yaml
common:
  msg_source: 5                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```

Since the point cloud message come from protobuf-UDP, set ```msg_source = 5```.

Send point cloud to ROS, so set ```send_point_cloud_ros = true```. 

```yaml
lidar:
  - driver:
      lidar_type: RS128           
      msop_port: 6699             
      difop_port: 7788            
      start_angle: 0              
      end_angle: 360               
      min_distance: 0.2            
      max_distance: 200            
      use_lidar_clock: false        
    ros:
      ros_frame_id: /rslidar           
      ros_recv_packet_topic: /rslidar_packets    
      ros_send_packet_topic: /rslidar_packets    
      ros_send_point_cloud_topic: /rslidar_points      
    proto:
      point_cloud_recv_port: 60021                     
      point_cloud_send_port: 60021                     
      msop_recv_port: 60022                       
      msop_send_port: 60022                       
      difop_recv_port: 60023                      
      difop_send_port: 60023       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1
```

Make sure the PC-B's ip address is same with the ```point_cloud_send_ip``` set in PC-A's config.yaml.

Set the ```point_cloud_recv_port```  to be same with the  ```point_cloud_send_port```  set in PC-A's config.yaml.











