# 首次使用

[English](../getting-started.md)

## 环境前提

| 语言 | 必要环境 |
| --- | --- |
| C++ | CMake 3.16 或更高版本，以及 MSVC、MinGW、GCC 等受支持的 C++ 编译器 |
| C# | Visual Studio，或能够构建 .NET Framework 4.8 项目的 MSBuild/.NET 环境 |
| Python | 与随附 Python SDK 兼容的 Python 环境 |

配置 SDK 依赖时，仓库目录必须具备写权限。

## 有 Python：自动配置

这是推荐方式。在仓库根目录查看项目，并配置一个项目：

```bash
python configure.py --list
python configure.py --project e-series-cpp
```

配置命令会根据 `.sha256` 文件校验 SDK ZIP、检查压缩包，并整体替换所选项目中由工具管理的 SDK 文件。

构建 C++ 项目：

```bash
cd e-series-cpp
python build.py --config Release
```

构建 C# 项目：

```bat
cd e-series-csharp
build.bat
```

Python 案例的 SDK 配置和运行都需要 Python 环境。

## 没有 Python：C++

1. 进入 `sdk/cpp/`，找到其中唯一的 SDK ZIP 和同名 `.sha256` 文件。
2. 校验 ZIP。Windows 环境执行 `Get-FileHash <archive.zip> -Algorithm SHA256`，并与 `.sha256` 中的摘要比较；Linux 环境在压缩包目录执行 `sha256sum -c <archive.zip.sha256>`。
3. 使用操作系统或经过批准的解压工具解压 ZIP。
4. 将解压后 SDK 中的 `include/HR_Pro.h` 复制到 `<project>/samples/HR_Pro.h`。
5. 创建 `<project>/lib/`，清理其中的旧文件，并根据编译工具复制对应库：

| 编译工具 | 解压后 SDK 中的源目录 |
| --- | --- |
| Visual Studio / MSVC | `MSVC/` |
| MinGW | `MinGW/` |
| Linux GCC | `Linux/` |

6. 使用 CMake 构建：

```bash
cmake -S <project> -B <project>/build
cmake --build <project>/build --config Release
```

## 没有 Python：C#

1. 进入 `sdk/csharp/`，按照上述方式使用同名 `.sha256` 文件校验 SDK ZIP。
2. 解压 ZIP。
3. 创建 `<project>/lib/`，清理其中的旧文件，然后复制 `RobotLibrarys.dll` 和 `Newtonsoft.Json.dll`。
4. 使用 Visual Studio 打开项目解决方案，或者在项目目录运行 `build.bat`。

SDK 配置和项目构建过程不会连接机器人，也不会执行任何案例。
