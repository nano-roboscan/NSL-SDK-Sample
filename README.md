# NSL-SDK-Sample
NSL2206-SDK와 NSL3140-SDK library 사용 예제 입니다.

## WINDOWS 컴파일 방법(C++)
- Visual studio 2019에서 테스트 되었습니다.
- PCL을 사용하는 경우 PCL-1.12.0-AllInOne-msvc2019-win64.exe 를 설치 후 win_build의 win_build.sln을 사용할 수 있습니다.
- PCL 라이브러리를 설치한 경우 main.cpp의 __USED_PCL_LIBLARY__ 활성화 후 사용하세요.
- opencv_library는 https://github.com/nano-roboscan/OpenCv454-LIbrary 를 download후 사용 하세요
- cmake를 사용하여 프로젝트 생성 시 설치된 opencv_library를 함께 사용 : cmake .. -DOpenCV_DIR=D:\git-hub\OpenCv454-LIbrary\opencv_library\x64\vc16\lib
- NSL3140의 기본 아이피는 192.168.0.220 입니다. 변경 시 WINDOWS Application을 사용하여 변경 가능합니다.

## Python  
- 기본 2D 뷰어는 OpenCV를 사용하며, Point Cloud의 경우 Open3D 를 사용 합니다.
- Open3D 설치 후 main.py의 with_open3d를 True로 전환 후 사용 합니다.

## USB 인식용 rules 정의
```
$ sudo vi /etc/udev/rules.d/defined_lidar.rules
KERNEL=="ttyACM*", ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="0094", MODE:="0777",SYMLINK+="ttyNsl3140"
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777",SYMLINK+="ttyNsl2206"

$ service udev reload
$ service udev restart

```
## ubuntu Network UDP speed up
```
$ sudo sysctl -w net.core.rmem_max=22020096
$ sudo sysctl -w net.core.rmem_default=22020096
```

## LINUX 컴파일 방법
```
$ git clone --recurse-submodules https://github.com/nano-roboscan/NSL-SDK-Sample.git
$ cd NSL-SDK-Sample/NSL2206-Sample or cd NSL-SDK-Sample/NSL3140-Sample
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./nslApp
```

## Point cloud C++
<img width="1033" height="863" alt="Image" src="https://github.com/user-attachments/assets/e44727e8-fb14-4700-b090-7506d2959b8f" />

## Point cloud Python
<img width="1320" height="710" alt="Image" src="https://github.com/user-attachments/assets/5f8df3a3-10dc-4be3-9111-e85328b23498" />
