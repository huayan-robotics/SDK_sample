# 常见问题

[English](../troubleshooting.md)

## 缺少 SDK 文件

如果具备 Python 环境，在仓库根目录运行配置命令：

```bash
python configure.py --project <project-id>
```

如果缺少 SDK ZIP、同名 `.sha256` 文件，或者实际 SHA-256 与记录值不一致，配置命令会停止。没有 Python 环境时，按照[首次使用](getting-started.md)中的 C++ 或 C# 手工配置步骤操作。

## 项目无法构建

检查操作系统、系统架构、编译器或运行时版本，以及 `sdk/<language>/` 下的 SDK 压缩包。

## 无法连接机器人

确认机器人型号、控制器状态、网络路由和配置的 IP 地址。请勿在公开 Issue 中提供私有网络信息。

## 运动案例行为异常

停止机器人，保留原始返回码和最小必要日志；再次运行前，检查工作空间、Home 点位、速度、力、坐标系和工具设置。
