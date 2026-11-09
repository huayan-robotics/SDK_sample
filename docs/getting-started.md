# Getting started

[简体中文](zh-CN/getting-started.md)

## Prerequisites

| Language | Required environment |
| --- | --- |
| C++ | CMake 3.16 or later and a supported C++ compiler such as MSVC, MinGW, or GCC |
| C# | Visual Studio or an MSBuild/.NET environment capable of building .NET Framework 4.8 projects |
| Python | A Python environment compatible with the bundled Python SDK |

The repository directory must be writable while SDK dependencies are being configured.

## With Python: automatic configuration

This is the recommended method. From the repository root, list the projects and configure one project:

```bash
python configure.py --list
python configure.py --project e-series-cpp
```

The command verifies the SDK ZIP against its `.sha256` file, validates the archive, and replaces the SDK files managed inside the selected project.

Build a C++ project:

```bash
cd e-series-cpp
python build.py --config Release
```

Build a C# project:

```bat
cd e-series-csharp
build.bat
```

Python samples require Python both for SDK configuration and execution.

## Without Python: C++

1. Open `sdk/cpp/` and identify the single SDK ZIP and its same-named `.sha256` file.
2. Verify the ZIP. On Windows, compare the result of `Get-FileHash <archive.zip> -Algorithm SHA256` with the digest recorded in the `.sha256` file. On Linux, run `sha256sum -c <archive.zip.sha256>` from the archive directory.
3. Extract the ZIP with the operating system or an approved archive tool.
4. Copy `include/HR_Pro.h` from the extracted SDK to `<project>/samples/HR_Pro.h`.
5. Create `<project>/lib/`, remove its old contents, and copy the libraries for the selected toolchain:

| Toolchain | Source directory in the extracted SDK |
| --- | --- |
| Visual Studio / MSVC | `MSVC/` |
| MinGW | `MinGW/` |
| Linux GCC | `Linux/` |

6. Build with CMake:

```bash
cmake -S <project> -B <project>/build
cmake --build <project>/build --config Release
```

## Without Python: C#

1. Open `sdk/csharp/` and verify its ZIP against the same-named `.sha256` file as described above.
2. Extract the ZIP.
3. Create `<project>/lib/`, remove its old contents, and copy `RobotLibrarys.dll` and `Newtonsoft.Json.dll` into it.
4. Open the project solution in Visual Studio, or run `build.bat` from the project directory.

The configuration and build steps do not connect to a robot or execute a sample.
