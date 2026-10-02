# xCore SDK Prebuilt Libraries

This directory does **not** include prebuilt libraries. Download them from [xCoreSDK-CPP Releases](https://github.com/RokaeRobot/xCoreSDK-CPP/releases); the required version is in [`VERSION`](../VERSION).

**Release URL:**

`https://github.com/RokaeRobot/xCoreSDK-CPP/releases/tag/v{VERSION}`

Example for 0.7.1: [Release v0.7.1](https://github.com/RokaeRobot/xCoreSDK-CPP/releases/tag/v0.7.1)

## How to obtain

1. Check the version in `rokae_hardware/sdk/VERSION`
2. Open the matching [Release page](https://github.com/RokaeRobot/xCoreSDK-CPP/releases/tag/v0.7.1)
3. Download the Linux library package, for example:
   - `xCoreSDK-0.7.1-linux-x86_64.tar.gz`
   - `xCoreSDK-0.7.1-linux-aarch64.tar.gz`
4. Extract the Release archive and **copy the following files into this directory** `rokae_hardware/sdk/lib/`:

### Linux x86_64 / aarch64

Copy from `lib/Linux/<arch>/` inside the Release archive:

```
rokae_hardware/sdk/lib/
  libxCoreSDK.a
  libxMateModel.a
  libxCoreSDK.so.0.7.1    # optional shared library
  libxCoreSDK.so          # optional, if the archive provides a symlink
```

`rokae_hardware` and `rokae_example` link the static libraries `libxCoreSDK.a` and `libxMateModel.a` by default.

### Windows (cross-development reference)

Copy `xCoreSDK_static.lib`, `xMateModel.lib`, and related files from `lib/Windows/Release/64bit/` in the Release archive into this directory, and adjust library names in CMake per platform (current CMake targets Linux static library names).

## Verify

From the ROS 2 workspace root:

```bash
colcon build --packages-select rokae_hardware
```

If the libraries are missing, CMake prints a WARNING with a download URL.

## Maintainer notes

- When upgrading the SDK: also update `sdk/include/` headers and `sdk/VERSION`, and note it in the stack CHANGELOG
- Do **not** commit `.a` / `.so` / `.dll` / `.lib` files to Git
- Full xCore SDK release process: [xCoreSDK-CPP lib/README.md](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/lib/README.md)
