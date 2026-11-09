# Huayan Robotics SDK Samples

- [中文说明](README.zh-CN.md)

SDK sample projects for E Series, S Series, ECHO5, and HY7 robots in C++, C#, and Python.

The repository is organized by product and programming language. Choose one project, configure its SDK dependencies, and then use the language-specific build or run command.

> Some samples can move a real robot. Verify the robot model, workspace, connection settings, home position, speed limits, and emergency stop before running a sample.

## Projects

| Product | C++ | C# | Python |
| --- | --- | --- | --- |
| E Series | [`e-series-cpp`](e-series-cpp/) | [`e-series-csharp`](e-series-csharp/) | [`e-series-python`](e-series-python/) |
| S Series | [`s-series-cpp`](s-series-cpp/) | [`s-series-csharp`](s-series-csharp/) | [`s-series-python`](s-series-python/) |
| ECHO5 | [`echo5-cpp`](echo5-cpp/) | [`echo5-csharp`](echo5-csharp/) | [`echo5-python`](echo5-python/) |
| HY7 | [`hy7-cpp`](hy7-cpp/) | [`hy7-csharp`](hy7-csharp/) | [`hy7-python`](hy7-python/) |

## Quick start

List the available projects:

```bash
python configure.py --list
```

Configure one project with the SDK stored in this repository:

```bash
python configure.py --project s-series-cpp
```

Then enter the selected project directory and follow its README:

```bash
cd s-series-cpp
python build.py --config Release
```

## Repository layout

```text
SDK_sample/
├── sdk/                       # SDK release ZIPs and SHA-256 files
├── <product>-<language>/      # 12 generated sample projects
├── LICENSE                    # BSD 3-Clause License
├── projects.json              # Stable product and language mapping
├── configure.py               # Local SDK configuration tool
├── docs/                      # Common usage and safety notes
└── .github/                   # Issue templates
```

## Documentation

- [Getting started](docs/getting-started.md)
- [Safety](docs/safety.md)
- [Troubleshooting](docs/troubleshooting.md)

Complete feature principles and API documentation remain in the Huayan Robotics web documentation.

## Feedback and security

- Report reproducible problems and sample requests through GitHub Issues. See [SUPPORT.md](SUPPORT.md) for the information required in a report.
- External pull requests are not accepted. Code and documentation changes are maintained by the internal team and synchronized to this repository.
- Do not report security vulnerabilities in public issues. See [SECURITY.md](SECURITY.md).

## License

Content copyrighted by Guangdong Huayan Robotics Co., Ltd., including the
samples, tools, documentation, and SDK components distributed by the company,
is licensed under the [BSD 3-Clause License](LICENSE).

Third-party components remain subject to their respective licenses and
copyright notices.
