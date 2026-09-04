# COD_UniCFramework

RoboMatser机甲大师 *COD战队* 机器人通用电控软件框架。

---

## 1. 快速开始

### 1.1 环境搭建

开发环境基于Linux。请使用*Ubuntu 24.04*或*WSL Ubuntu 24.04*(推荐)

IDE使用VSCode。(微软大战代码)

#### 1.1.1 安装 arm-gcc 工具链

以下命令将从 [ARM 官网](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) 下载arm-gcc工具链，并把 `bin` 路径添加进环境变量

```sh
mkdir -p ~/tools

tar -xf ~/Downloads/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi.tar.xz -C ~/tools

echo 'export PATH=$HOME/tools/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi/bin:$PATH' >> ~/.bashrc

echo 'export ARM_TOOLCHAIN_BIN=$HOME/tools/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi/bin' >> ~/.bashrc

source ~/.bashrc

arm-none-eabi-gcc --version    # 能打出版本号就没问题
```

#### 1.1.2 安装构建、调试、代码索引、代码格式化工具

```sh
sudo apt install cmake make openocd clangd clang-format
```

#### 1.1.3 安装其他工具

如果想共同维护这个~~屎山~~框架，可以安装以下工具，否则没必要安装

```sh
sudo apt install gcc python3 ruby cppcheck clang-tidy-18 clang-format-18
```

#### 1.1.4 克隆本仓库

```sh
git clone https://github.com/fakeSkyy/COD_UniCFramework
```

#### 1.1.5 编译

```sh
./build.sh clean

./build.sh --help   #查看所有命令
```

---

## 2. 工程架构

```
COD_UniCFramework
├── 01_application   
├── 02_device       
├── 03_platform      
├── 04_impl         
├── 05_vender       
├── 06_utils        
└── docs           
```

### 使用规范

依赖总体向下：`01 → 02 → 03 → 04 → 05`，`06_utils` 可被所有层使用。应用层可以直接依赖
`02_device`、`03_platform` 和 `06_utils`；`01_application/board` 是唯一允许同时组合平台与厂商对象的位置。`02_device` 不得出现厂商类型，`03_platform` 只能通过
`04_impl/common` 的 ops + opaque context 接口绑定实现，禁止直接依赖具体后端。

`06_utils` 原则上不得向上依赖。现存的 4 条 `SEGGER_RTT`/`plat_mutex`/`plat_task` 依赖以逐条reviewed architecture baseline 记录，任何新增依赖或已过期条目都会使质量门禁失败，不能以目录或规则通配方式放行。

---

## 致谢

todo

---
