# BoxDemo

## Basic requirements
```
python == 3.6
```

## Installation
### update pip3 first
````
pip3 install --upgrade pip
````
### 1. numpy, scipy, opencv, numba, and open3d
cd to "depalletizing_ws/src/box_node/boxdemo"
````
cd ~/HKCLR-Depalletizing-Demo/depalletizing_ws/src/box_node/boxdemo
````
install the dependence
```
pip3 install -r requirements.txt
```
### 2. pcl
```
pip3 install cython==0.26.0
```
```
sudo apt-get install libpcl-dev pcl-tools
```

### 3. python-pcl
cd back to home
```
cd ~/
```
clone the python-pcl git repo
```
git clone https://github.com/strawlab/python-pcl.git
cd python-pcl
```

Important Note!: Before build install python-pcl, replace the "setup.py of python-pcl" whth "python_pcl.setup.py of boxdemo" to avoid possible conflicts:

1. copy "python_pcl.setup.py" of "boxdemo" to "python-pcl"

2. delete "setup.py" of "python-pcl" and rename "python_pcl.setup.py" as "setup.py"

Then, install python-pcl in python3 for unstack algo
```
python3 setup.py build_ext -i
sudo python3 setup.py install
```
(Optional) If you want to use send_data_tester, you need install python-pcl in python2, too.
```
python setup.py build_ext -i
sudo python setup.py install
```

### 4. Download the 2D segmentation network model for unstacking, and add it to the project
1. download from https://drive.google.com/file/d/1TuWlr8fkwJ_zGBGp6HkjWGbv-g-bepTP/view?usp=sharing
2. create a folder boxdemo/segmentation/models/, and put the downloaded model into it, as like: boxdemo/segmentation/models/model_0090999.pth


### 5. runtime cuda
```
wget https://developer.download.nvidia.com/compute/cuda/10.2/Prod/local_installers/cuda_10.2.89_440.33.01_linux.run
```
```
sudo sh cuda_10.2.89_440.33.01_linux.run
```
Untick the nvidia driver when installing and only install cuda* related       

Then add following lines into the beginning of ".bashrc":
```
PATH="/usr/local/cuda-10.2/bin:$PATH"
LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/cuda-10.2/lib64"
```

## Usage 
Before run the test, you need to replace "andylee" with your own usr name in "parser.add_argument('--model', type=str, default='/home/andylee/HKCLR-Depalletizing-Demo/depalletizing_ws/src/box_node/boxdemo/model.yml')" in "boxdemo/config.py"
```
python3 ~/HKCLR-Depalletizing-Demo/depalletizing_ws/src/box_node/boxdemo/example_unstack.py
```
there will be a segment effect photo showing firstly and after you close it, a 3d point cloud with coordinates will show up.  
###Installation process ends here. you can go back to where you from and keep going.

## Main Parameters in config.py
```
PREPROCESS_THRESHOLD_DEFAULT (default = 30): tuning this value for detecting edges in the image
filter_ground_plane_para: the parameters of the ground plane. We can crop the point cloud of the ground and pre-compute the paras and save it.
dist2plane: the distance threshold for filtering points near a plane (within dist2plane)
camera_intrinsics: intrinsic paras of camera for converting between point cloud and the corresponding detpth/grey image
```

## Device requirements
```
>= GeForce RTX 2080 Ti 
```
