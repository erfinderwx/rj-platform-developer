# RJ Platform Developer

Public development resources for RJ Tech mobile platforms. The first software preview is **Luchs 1.1.0-preview.1**, curated from the public Luchs family ROS packages with upstream package version **1.1.0**.

This preview supports source review, building and simulator development. Hardware compatibility and formal vehicle acceptance are pending. In particular, Luchs B LCI describes a draft interface; simulator features do not establish that vehicle firmware implements them.

| Resource | Start here |
| --- | --- |
| ROS 2 Humble and Jazzy | [ROS 2 instructions](ros2/README.md) |
| ROS 1 Noetic | [ROS 1 instructions](ros1/README.md) |
| Release status, tests and formal release requirements | [Luchs version development manual](docs/luchs-development-manual.md) |
| Public source provenance | [Source manifest](docs/luchs-public-sources.json) |

## Get the preview

```bash
git clone --branch luchs-v1.1.0-preview.1 --depth 1 https://github.com/erfinderwx/rj-platform-developer.git ~/luchs_preview
```

`luchs-v1.1.0-preview.1` is a frozen release snapshot branch. Future changes will use a new preview identifier. Record the immutable checkout commit with `git rev-parse HEAD`.

Build **one** of `ros1/` or `ros2/` in a separate ROS workspace. They contain packages with the same names and different build systems. Follow that directory's README for dependencies and commands. The B preview launch and configuration default to `dry_run=true`; begin with simulator examples.

## Contents and license

The preview includes the C++17 SDK, ROS driver, messages and services, bringup, robot descriptions, teleoperation, examples and protocol references. The source packages and original attribution are preserved under [Apache 2.0](LICENSE); see [NOTICE](NOTICE) for the import and modifications. Protocol drafts and example geometry remain provisional.

Only material already approved for the public software package is included. The broader internal material archive is outside this repository.

See [contribution guidance](CONTRIBUTING.md), [documentation](docs/README.md) and [changes](CHANGELOG.md). Other platform sections remain placeholders until public implementations are available.
