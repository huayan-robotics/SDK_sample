# 华沿机器人 SDK 案例

- [English documentation](README.md)

本仓库提供 E 系列、S 系列、ECHO5 和 HY7 机器人的 C++、C#、Python SDK 案例。

仓库按产品和编程语言组织。客户选择对应项目，通过根目录配置脚本配置 SDK 依赖，再使用该语言的标准方式构建或运行。

> 部分案例会驱动真实机器人。运行前必须确认机器人型号、工作空间、连接参数、Home 点位、速度限制和急停条件。

## 项目入口

| 产品 | C++ | C# | Python |
| --- | --- | --- | --- |
| E 系列 | [`e-series-cpp`](e-series-cpp/) | [`e-series-csharp`](e-series-csharp/) | [`e-series-python`](e-series-python/) |
| S 系列 | [`s-series-cpp`](s-series-cpp/) | [`s-series-csharp`](s-series-csharp/) | [`s-series-python`](s-series-python/) |
| ECHO5 | [`echo5-cpp`](echo5-cpp/) | [`echo5-csharp`](echo5-csharp/) | [`echo5-python`](echo5-python/) |
| HY7 | [`hy7-cpp`](hy7-cpp/) | [`hy7-csharp`](hy7-csharp/) | [`hy7-python`](hy7-python/) |

## 快速开始

查看可用项目：

```bash
python configure.py --list
```

将根目录 SDK 配置到指定项目：

```bash
python configure.py --project s-series-cpp
```

然后进入项目目录，按照项目 README 构建或运行：

```bash
cd s-series-cpp
python build.py --config Release
```

## 目录说明

```text
SDK_sample/
├── sdk/                       # SDK 发布 ZIP 和 SHA-256 文件
├── <product>-<language>/      # 12 个案例项目
├── LICENSE                    # BSD 3-Clause License
├── projects.json              # 固定的机型与语言映射
├── configure.py               # 本地 SDK 配置工具
├── docs/                      # 通用使用和安全说明
└── .github/                   # Issue 模板
```

## 文档

- [首次使用](docs/zh-CN/getting-started.md)
- [安全要求](docs/zh-CN/safety.md)
- [常见问题](docs/zh-CN/troubleshooting.md)


完整功能原理和 API 说明继续由华沿机器人网页化文档承载，本仓库以代码获取、构建运行和问题反馈为主。

## 问题反馈与安全

- 可通过 GitHub Issues 反馈能够复现的问题或提出案例需求，提交前请阅读 [问题反馈规范](SUPPORT.zh-CN.md)。
- 本仓库不接收外部 Pull Request。代码和文档由内部团队维护并同步到本仓库。
- 请勿通过公开 Issue 报告安全漏洞，具体要求见 [安全策略](SECURITY.md)。

## 许可证

本仓库中由广东华沿机器人有限公司拥有版权的内容，包括案例、工具、文档以及由公司分发的 SDK 组件，采用 [BSD 3-Clause License](LICENSE)。

第三方组件继续适用其各自的许可证和版权声明。
