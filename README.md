# NSL-SDK
NSL2206-SDK와 NSL3140-SDK library입니다.

## WINDOWS 컴파일 방법
- Visual studio 2019에서 테스트 되었습니다.
- PCL-1.12.0-AllInOne-msvc2019-win64.exe 를 설치 후 opencv_library의 PATH 를 설정하여 사용 하십시오.
- NSL3140의 기본 아이피는 192.168.0.220 입니다. 변경 시 WINDOWS Application을 사용하여 변경 가능합니다.

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

## Point cloud
<img width="1033" height="863" alt="Image" src="https://github.com/user-attachments/assets/e44727e8-fb14-4700-b090-7506d2959b8f" />
