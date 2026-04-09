# Leadtron Dexterous Hand Descriptions

This repository contains public robot description files for Leadtron dexterous hands.

At present, the repository includes the following hand models:

- `DH116`
- `DH116S`

Available description formats currently include:

- `URDF`
- `MJCF`

## Overview

This repository is intended to provide a clean and reusable description package set for Leadtron dexterous hands, including model geometry, kinematic structure, joint relationships, and simulation-oriented assets.

For each supported model, the repository may include:

- ROS package files such as `package.xml` and `CMakeLists.txt`
- `URDF` model descriptions
- `MJCF` model descriptions
- mesh assets used by the model
- launch and configuration files for visualization or simulation

## Supported Models

### `DH116`

The `DH116` package currently provides:

- left-hand and right-hand description packages
- `URDF` descriptions
- `MJCF` descriptions

Main path:

- `DH116/DH116_URDF_MJCF_Files/`

### `DH116S`

The `DH116S` package currently provides:

- left-hand and right-hand description packages
- `URDF` descriptions
- `MJCF` descriptions

Main path:

- `DH116S/DH116S_URDF_MJCF_Files/`

## Repository Structure

Key directories in this repository:

- `DH116/`: description files and release assets for the `DH116` hand
- `DH116S/`: description files and release assets for the `DH116S` hand
- `LICENSE`: repository license

Within each model package, you will typically find:

- `urdf/`
- `mjcf/`
- `meshes/`
- `launch/`
- `config/`

## Releases

Packaged archives are published through GitHub Releases for versioned distribution.

If you are looking for ready-to-download release bundles, please check:

- `Releases` on this repository page

## Notes

- The repository is focused on description assets for public use and distribution.
- Different hand models may have different joint parameters, coupling relationships, and simulation files.
- Please refer to the model-specific `README.md` files under `DH116/` and `DH116S/` for additional details.

## License

This repository is provided under the Apache License 2.0. See [LICENSE](/D:/Project/SWProject/leadtron_hand_descriptions/LICENSE) for details.
