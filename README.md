# GNSS SPP/RTK Positioning in C++

基于 C++ 实现的卫星导航定位解算程序，包含 GNSS 原始观测数据解码、单点定位（SPP）、RTK 双差定位、LAMBDA 整周模糊度固定以及卡尔曼滤波相关处理模块。

项目主要用于展示 GNSS 定位算法的自编程实现过程，包括观测数据组织、卫星位置计算、误差改正、最小二乘定位、双差观测建模和滤波解算。

## 功能概览

- GNSS 原始数据解码
- GPS/BDS 卫星星历与观测数据组织
- 坐标系统转换
- 卫星位置计算
- 单点定位（SPP）
- RTK 单差、双差观测构建
- 双差最小二乘定位
- LAMBDA 整周模糊度固定
- 卡尔曼滤波平滑定位结果
- TCP/IP 实时数据接收接口

## 项目结构

```text
代码/
├─ main.cpp
├─ allnecehead.h
├─ standardhead.h
├─ const.h
├─ datascales.h
├─ matrix.h
├─ Decode.cpp
├─ Decode.h
├─ TimeConvert.cpp
├─ TimeConvert.h
├─ CoordinateConvert.cpp
├─ CoordinateConvert.h
├─ SatPos.cpp
├─ SatPos.h
├─ Error.cpp
├─ Error.h
├─ SPP.cpp
├─ SPP.h
├─ SDAStation.cpp
├─ SDAStation.h
├─ lambda.cpp
├─ lambda.h
├─ KFC.cpp
├─ KFC.h
├─ sockets.cpp
└─ sockets.h
```

## 文件说明

| 文件 | 作用 |
|---|---|
| `main.cpp` | 程序入口，组织文件读取、SPP、RTK、LAMBDA 和卡尔曼滤波流程。 |
| `datascales.h` | 定义观测数据、星历、定位结果、单差/双差观测等数据结构。 |
| `const.h` | 定义 WGS84、光速、地球自转角速度等常量。 |
| `matrix.h` | 自定义矩阵类，提供矩阵运算、转置、求逆等基础功能。 |
| `Decode.cpp/.h` | 解码 GNSS 原始数据，提取观测值和星历信息。 |
| `TimeConvert.cpp/.h` | 完成 GPS 时间、普通时间等时间系统转换。 |
| `CoordinateConvert.cpp/.h` | 完成 XYZ、BLH、ENU 等坐标转换。 |
| `SatPos.cpp/.h` | 根据星历计算卫星位置和钟差。 |
| `Error.cpp/.h` | 完成粗差探测和误差处理。 |
| `SPP.cpp/.h` | 实现单点定位和速度估计相关算法。 |
| `SDAStation.cpp/.h` | 构造单差、双差观测并完成 RTK 最小二乘解算。 |
| `lambda.cpp/.h` | 实现 LAMBDA 整周模糊度固定。 |
| `KFC.cpp/.h` | 实现基于卡尔曼滤波的连续定位处理。 |
| `sockets.cpp/.h` | 提供网络数据接收接口。 |

## 编译环境

推荐环境：

- Windows 10/11
- Visual Studio 2022 或支持 C++17 的 MSVC 编译器
- C++17
- Winsock 网络库

使用 Visual Studio Developer PowerShell 编译：

```powershell
cl /std:c++17 /EHsc `
  main.cpp Decode.cpp TimeConvert.cpp CoordinateConvert.cpp SatPos.cpp Error.cpp `
  SPP.cpp SDAStation.cpp lambda.cpp KFC.cpp sockets.cpp `
  /Fe:gnss_positioning.exe ws2_32.lib
```

也可以将所有 `.cpp` 和 `.h` 文件加入 Visual Studio C++ 工程后编译。

## 运行说明

当前 `main.cpp` 中的数据文件路径和结果输出路径为本机绝对路径，例如：

```cpp
C:\\Users\\28638\\Desktop\\FH\\oem719-202510311730-rover.bin
C:\\Users\\28638\\Desktop\\FH\\oem719-202510311730-base.bin
```

运行前需要根据自己的数据位置修改 `main.cpp` 中的 rover/base 数据路径和输出路径。

程序主要处理两类数据：

- rover：流动站观测数据
- base：基准站观测数据

程序会对 rover/base 数据进行时间同步，分别完成 SPP 初值计算，再构造单差和双差观测，进行 RTK 解算、LAMBDA 模糊度固定和卡尔曼滤波处理。

## 输出结果

程序运行后会输出两类定位结果：

- 最小二乘 RTK 定位结果
- 卡尔曼滤波定位结果

输出内容通常包括：

- GPS 周
- 周内秒
- 纬度
- 经度
- 高程
- 固定状态标记

结果文件路径由 `main.cpp` 中的 `ofstream` 配置决定，上传或复现实验前建议改为相对路径，例如：

```cpp
ofstream ofs1("least_squares_result.txt");
ofstream ofs2("kalman_filter_result.txt");
```

## 注意事项

- 当前仓库主要展示算法实现代码，不包含完整原始观测数据。
- 代码中存在本机绝对路径，复现运行前需要手动修改。
- 网络实时接收部分包含 IP 和端口配置，使用前需要替换为自己的数据源。
- 如果上传示例数据，建议放在 GitHub Releases 中，不建议直接提交大体积原始数据到 Git 仓库。

## 建议仓库名称

推荐使用：

```text
gnss-spp-rtk-positioning-cpp
```

也可以使用：

```text
gnss-rtk-kalman-cpp
```

仓库描述可以填写：

```text
C++ implementation of GNSS SPP, RTK double-difference positioning, LAMBDA ambiguity resolution, and Kalman filtering.
```
