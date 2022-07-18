<div align="center">
  <h1>PandaSet LiDAR points to depth image</h1>
</div>
Change PandaSet LiDAR points to depth image of front camera. To use in other dataset, check tools/Lidar2Camera.py and tools/SE3.py and pandaset_lidar2img.py


## Dependency
open3d  
numpy  
scipy  
opencv  
pyyaml  
glob  
json

## Download PandaSet dataset.
PandaSet dataset download link: https://pandaset.org/

**Copy PandaSet data to ./PandaSet**

## Download Pandaset devkit
dev-kit: https://github.com/scaleapi/pandaset-devkit

**Copy pandaset-devkit/docs/static_extrinsic_calibration.yaml to ./pandaset_calibration.yaml**

## File structure
```
.
├── img2d
│   ├── camera
│   └── lidar
├── PandaSet
│   ├── 001
│   ├── 002
│   ├── 003
│   ├── 004
│   ├── 005
│   ├── 006
│   ├── 008
│   ├── 011
│   ├── 012
│   ├── 013
│   ├── 014
│   ├── 015
│   ├── 016
│   ├── 017
│   ├── 018
│   ├── 019
│   ├── 020
│   ├── 021
│   ├── 023
│   ├── 024
│   ├── 027
│   ├── 028
│   ├── 029
│   ├── 030
│   ├── 032
│   ├── 033
│   ├── 034
│   ├── 035
│   ├── 037
│   ├── 038
│   ├── 039
│   ├── 040
│   ├── 041
│   ├── 042
│   ├── 043
│   ├── 044
│   ├── 045
│   ├── 046
│   └── 047
├── pandaset_calibration.yaml
├── pandaset_lidar2img.py
└── tools
    ├── __init__.py
    ├── Inlier_selection.py
    ├── Lidar2Camera.py
    ├── __pycache__
    └── SE3.py
```

## Usage
```
python3 pandaset_lidar2img.py
```

You can get front camera image and depth image pair in img_2d directory