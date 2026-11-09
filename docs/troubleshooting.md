# Troubleshooting

[简体中文](zh-CN/troubleshooting.md)

## SDK files are missing

If Python is available, run the configuration command from the repository root:

```bash
python configure.py --project <project-id>
```

The command fails if the SDK ZIP or its same-named `.sha256` file is missing, or if the recorded SHA-256 does not match. Without Python, follow the manual C++ or C# procedure in [Getting started](getting-started.md).

## The project does not build

Check the operating system, architecture, compiler or runtime version, and the SDK archive under `sdk/<language>/`.

## The robot cannot be reached

Confirm the robot model, controller state, network route, and configured IP address. Do not include private network information in public issues.

## A motion sample behaves unexpectedly

Stop the robot, preserve the original return code and minimal log, and verify the workspace, home position, speed, force, coordinate system, and tool settings before retrying.
