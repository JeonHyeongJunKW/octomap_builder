# Octomap_builder
Azure kinect DK RGB-D 카메라를 기반으로 Octomap을 생성합니다. 

### 목적 
- Octomap 사용법 익히기
- 데이터셋이 아닌 RGB-D 카메라 기반의 Visual Odometry 경험

### 목표
- RGB-D 입력을 기반으로한 Visual Odometry를 기반으로 카메라 위치 추정 및 Octomap 생성


## 준비하기

### 필요한 기본 라이브러리 
- OpenCV 
- Azurekinect sdk

### Azure Kinect sdk 설치법

``` 
apt install libk4abt1.1-dev
apt install libxi-dev
```

### ROS 패키지설치(Melodic 기준)

```
sudo apt-get install ros-melodic-octomap-ros ros-melodic-octomap-mapping
```

## 실행하기 

### 코드 다운로드

각 workspace의 src폴더에 다음 코드를 풀어주세요.

```
roscd catkin_ws/src
git clone https://github.com/JeonHyeongJunKW/octomap_builder.git
git clone https://github.com/OctoMap/octomap_mapping.git
cd ../ && catkin_make
```

### 실행
```
# step 1
sudo -s #관리자 권한 부여
rosrun octomap_builder octobuilder

# step 2
roslaunch octomap_builder visodom_octomap.launch
# rviz에서 marker array를 add하고,  
```

### 종료 
- esc버튼을 화면에서 누르고, 이미지 종료
```
sudo - $username
```
### 어려웠던점

- Octomap을 c++ 패키지로 사용할 생각이지만, 오래된 라이브러리라서 튜토리얼과 설치하기에는 라이브러리 사양에 맞는 종속 라이브러리(Qt 등등)를 사용하기 어려웠다.
    -> 따라서 ROS를 기반으로 만들었다.

- PCL을 사용할 때, point cloud2 데이터를 어떻게 다뤄야하는지 자세한 설명이 부족하였다. 그래서 그 부분에서 애먹었다. 

### 진행사항

22/09/01 : 레포지터리 생성

22/09/02 : Azure kinect를 통한 rgb-d정보 얻어오기

22/09/05 : ros와 연동하여, octomap 라이브러리 링크 

22/09/10 : pcl을 사용하여, rviz상에 띄우는것에 성공

22/09/11 : pointcloud2 형식을 octomap server에 보내는 것 성공