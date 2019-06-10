ROS2 Qt image transport sample
=====================================
ROS2 & Qt Quick を用いてUSBカメラからの映像を送受信、表示するサンプル。  
  
詳細は技ラボを参照のこと。  
[ROS2 + Qt Quickを使ったシステム開発](http://wazalabo.com/ros2_and_qtquick.html)


動作環境
----------
- Ubuntu 18.04
    - ROS2 crystal
    - Qt 5.9
    - OpenCV 3.4
- PCにUSBカメラが接続されていること (/dev/video0)。

ビルド
-----------
```
$ mkdir -p sample_ws/src
$ cd sample_ws/src
$ git clone https://github.com/ISP-Kazuki-Nagasawa/ROS2_Qt_image_transport.git image_transport
$ cd ../
$ colcon build --symlink-install
```

実行
-----------
画像配信 + Qt受信・表示ウィンドウ
```
$ install/setup.bash
$ ros2 launch image_sender exec.py
```

画像配信 + OpenCV受信・表示ウィンドウ + Qt受信・表示ウィンドウ
```
$ install/setup.bash
$ ros2 launch image_sender exec_all.py
```




