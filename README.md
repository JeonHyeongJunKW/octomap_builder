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
```

### 실행
```
sudo -s #관리자 권한 부여
rosrun octomap_builder octobuilder
roslaunch octomap_builder visodom_octomap.launch
```

### 종료 
- esc버튼을 화면에서 누르고, 이미지 종료
```
sudo - $username
```
### 어려웠던점

- Octomap을 c++ 패키지로 사용할 생각이지만, 오래된 라이브러리라서 튜토리얼과 설치하기에는 컴퓨터 사양에 맞는 코드를 사용하기 어려웠다.
    -> 따라서 ROS를 기반으로 만들었다.