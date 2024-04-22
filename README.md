# camera_ros

### ros2 활용 camera 만들어보기<br/>

<br/>

### 1. configure  <br/>

![노드 구성](https://github.com/hyegyeong-Y/ar_unity/assets/113657807/09404722-dd2a-497f-aceb-6c5078de0e1b)

<br/>

* img_publisher : 카메라 또는 기타 이미지 소스의 이미지를 ROS topic에 게시하는 역할
* camera : 웹캠 카메라 장치에서 이미지를 가져옴
* canny :  Canny Edge 적용(이미지에서 강도가 높은 그라데이션(edge) 영역을 감지)
* camera_server : 이미지 캡처 후 저장(높이 너비 지정 가능)
* video_server : 비디오 녹화 기능 관리(start, stop) 및 저장

  
```
 src
    ├── camera_msgs
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── camera_msgs
    │   ├── package.xml
    │   ├── src
    │   ├── srv
    │   │   ├── CaptureCameraImage.srv
    │   │   ├── StartRecording.srv
    │   │   └── StopRecording.srv
    │   └── storage
    └── camera_package
        ├── camera_package
        │   ├── camera_server.py
        │   ├── camera_subscriber.py
        │   ├── canny.py
        │   ├── canny_with_cam.py
        │   ├── img_publisher.py
        │   ├── __init__.py
        │   ├── test_image_capture.py
        │   ├── video_client.py
        │   └── video_server.py
        ├── capture
        │   ├── 1_captured_image.jpg
        │   └── 2_captured_image.jpg
        ├── launch
        │   ├── camera.launch copy.py
        │   ├── camera.launch.py
        │   ├── canny.launch.py
        │   └── canny_with_cam.launch.py
        ├── package.xml
        ├── param
        │   ├── filter.yaml
        │   ├── size.yaml
        │   ├── storage.yaml
        │   └── video.yaml
        ├── resource
        │   └── camera_package
        ├── service
        │   └── CaptureCameraImage.srv
        ├── setup.cfg
        ├── setup.py
        └── video
            └── recorded_video_1.avi

```



### 2. setting <br/>


* Ros2 humble,  OpenCV, Ubuntu22.04LTS 활용
* ros2 setting

```
mkdir -p ~/camera_ros(or your workspace_name)/src
cd camera_ros/src
git clone (my github repository)
cd ..
colcon build

```
* 터미널 각자 세팅 (terminator추천)
```
source /opt/ros/humble/setup.bash
source ~/camera_ros/install/local_setup.bash

```
### 3. example <br/>
* canny and canny_with_cam: 각각 터미널에서 실행 (4개의 터미널 각자 세팅 부분 필수) <br/>
 ```
ros2 launch camera_package camera.launch.py 
ros2 launch camera_package canny.launch.py
ros2 launch camera_package canny_with_cam.launch.py
rqt

 ```
![canny](https://github.com/hyegyeong-Y/ar_unity/assets/113657807/0b6b0c80-fa4c-4cc0-ac19-cca66ed7b106) <br/>
![canny_with_cam](https://github.com/hyegyeong-Y/ar_unity/assets/113657807/7367d7a5-067f-4f69-99bd-13dd48cf2bbc)
<br/><br/>
* camera_server: 각각 터미널에서 실행 (2개의 터미널 각자 세팅 부분 필수, width height 변경 가능) <br/>
```
ros2 launch camera_package camera.launch.py 
ros2 service call /capture_camera_image camera_msgs/srv/CaptureCameraImage "{width: 640, height: 480}"

```
![camera_server](https://github.com/hyegyeong-Y/ar_unity/assets/113657807/84747db1-b4a9-4bb3-9e75-e5a0ad34b6cf)
<br/><br/>
* video_server 각각 터미널에서 실행 (2개의 터미널 각자 세팅 부분 필수) <br/>
```
ros2 launch camera_package camera.launch.py
ros2 service call /start_recording std_srvs/srv/Trigger
ros2 service call /stop_recording std_srvs/srv/Trigger

```
![video_server](https://github.com/hyegyeong-Y/ar_unity/assets/113657807/dc2e3d20-4696-44b1-ad1a-b3cec10ddce0)


