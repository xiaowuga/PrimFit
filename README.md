# PrimFit

![](./images/overview.png)



**PrimFit** is the implementation of reconstruction method described in  paper “**Structure–Aware Surface Reconstruction via Primitive Assembly**”.

### 📢 Windows Version Released

The [**Windows Executables**](https://github.com/xiaowuga/PrimFit/releases/tag/PrimFit_Windows_Release_v1.0) are now live. 💻
Feel free to test and provide feedback. 📝

> ⚠️ **Note:** For evaluation only.


## Platform
- Windows 11
- CLion2024.1.2 +  Visual Stdio 2022
- Intel(R) Core i9-13900K

## Dependence

The dependent libraries of our code includes:
- Eigen3 (3.4.0 or later)
- CLI11 (2.4.0 or later), command line.
- nlohmann_json (3.11.3 or later), read json file.
- Easy3D (2.5.2), Please visit the [Prof.Nan's repository](https://github.com/LiangliangNan/Easy3D) to obtain it and follow the instructions for installation.
- libigl, A special version, and automatically obtained through the `FetchContent` function in CMake. Ensure the stability of your VPN if you are in mainland China.

For Eigen3, CLI11 and nlohmann_json, we recommend using [vcpkg](https://github.com/microsoft/vcpkg) to install them.
```shell
# Eigen3
vcpkg install Eigen3:x64-windwos
# cli11
vcpkg install cli11:x64-windows
# nlohmann_json
vcpkg install nlohmann_json:x64-windows
```
For Easy3D, you need to set the `${Easy3D_DIR}` environment variable to the directory that contains the `Easy3DConfig.cmake` file.

For libigl, if you're located in mainland China and your VPN connection is unstable, you can download the `external.zip` file via my [Baidu Netdisk link](https://pan.baidu.com/s/1deMHYUQk3k-fq9VsfLWeHg?pwd=7u32).
Please extract `external.zip` into the project root directory, resulting in the following structure:
```plaintext
PrimFit_project_root/
├── external/                
├── CMakeLists.txt
└── ...
```
  




## How to build

Building our code in CLion:
```
# File -> Setting -> Build, Execution, Deployment -> CMake -> CMake Option :
-DCMAKE_TOOLCHAIN_FILE=${YOUR_VCPKG_INSTALL_PATH}/scripts/buildsystems/vcpkg.cmake
```
Making sure that your following settings are correct:
- Toolchains : `Visual Stdio`
- Architecture : `amd64`
- Build Type : `release`


## Usage


### primfit_cli

```shell
primfit_cli.exe -c config.json
```

`primfit_cli.exe` takes a `.seg` file as input and outputs a `.obj` file. The `.seg` format is a custom data format that contains segmented primitives and point cloud data. You can modify the `config.json` file to change the input and output file paths as well as algorithm parameters (configured based on the details in the paper, typically no adjustments are needed).

### Ransac2Seg
```shell
Ransac2Seg.exe -c seg_config.json
```
`Ransac2Seg.exe` takes a `.ply` point cloud file (with normals) as input and generates a `.seg` file, which serves as the input for `primfit.exe`.


You can modify the `seg_config.json` file to change the input and output file paths as well as Ransac parameters. The specific parameter tuning details can be referenced from the paper [Efficient RANSAC](https://onlinelibrary.wiley.com/doi/full/10.1111/j.1467-8659.2007.01016.x).

### vg2Seg
```shell
vg2seg.exe -i input.vg -o out.seg
```
`vg2Seg.exe` converts a `.vg` file to our custom `.seg` format, which serves as the input for `primfit.exe`. The `.vg` file can be obtained from [PolyFit](https://github.com/LiangliangNan/PolyFit) or [KSR](https://www-sop.inria.fr/members/Florent.Lafarge/code/KSR.zip).


The folder `data/sub_abc_seg` provides 40 .seg files for testing; `data/sub_abc_ours` contains our test results; and `data/sub_abc_ply` holds the original test point cloud files.

We also provide executable files for Windows. Feel free to download and test them.

## Citation
If you make use of our work, please cite our paper:

```bibtex
@inproceedings{Jiang2023primfit,
  title={Structure–Aware Surface Reconstruction via Primitive Assembly},
  author={Jingen Jiang, Mingyang Zhao, Shiqing Xin, Yanchao Yang, Hanxiao Wang, Xiaohong Jia, Dong-Ming Yan},
  booktitle={Proceedings of the IEEE/CVF International Conference on Computer Vision},
  year={2023}
}
```

## Acknowledgements

This work was partially funded by the National Key Research and Development Program (2021YFB1715900), the CAS Project for Young Scientists in Basic Research (YSBR-034), the National Natural Science Foundation of China (62172415, 62272277, 12022117), and the HKU-100 Research Award.

Our code is inspired the works of [BSH](https://github.com/duxingyi-charles/Boundary_Sampled_Halfspaces), [PolyFit](https://github.com/LiangliangNan/PolyFit) and [KSR](https://www-sop.inria.fr/members/Florent.Lafarge/code/KSR.zip). We would like to thank Dr. [Xingyi Du](https://duxingyi-charles.github.io/) and Prof. [Liangliang Nan](https://3d.bk.tudelft.nl/liangliang/) for their excellent code.

Furthermore, we are grateful to Jiahui Lv from Shenzhen University for his valuable advice in this work.

## Maintaince

If any problem, please contact me via <xiaowuga@gmail.com>.





