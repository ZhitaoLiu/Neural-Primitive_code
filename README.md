# Neural-Primitive

## Prerequisites

* Ubuntu 20.04 LTS with ROS1 Noetic installation.
* CUDA already configured.

## Run the planner

**1. Download the code**

```bash 
git clone https://github.com/ZhitaoLiu/Neural-Primitive_code.git
```


**2. Configure LibTorch**

* Download the LibTorch at [PyTorch Website](https://pytorch.org/), select a version compatible with your CUDA version.

* Extract to obtain the libtorch folder and set the corresponding environment variables:

```bash
echo 'export LIBTORCH=<Your libtorch path>' >> ~/.bashrc   
echo 'export LD_LIBRARY_PATH=$LIBTORCH/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

* Modify the ```Torch REQUIRED PATHS``` in ```src/planner/src/plan_manage/CMakeLists.txt``` to your libtorch path:

```bash
find_package(Torch REQUIRED PATHS "<Your libtorch path>") 
```

**3. Compile the code**

```bash
cd src/planner
catkin_make
```

**4. Run**
```bash
source devel/setup.bash
roslaunch plan_manage run_in_sim.launch
```
Then use 2D Nav Goal to select the target point and trigger planning.

Note:

* You can also set ```flight_type``` to ```2``` in the ```run_in_sim.launch``` file to automatically fly to the preset waypoints.

* You can select different policy models in ```src/planner/src/plan_manage/launch/model.yaml```.


## Train the network

**1. Obtain datasets**

* Download the dataset at [dataSetFile](https://pan.baidu.com/s/1zlOlx1PjfsWNorGGpmbU8A?pwd=d3mj), or collect new datasets by referring to [Collected the dataset](#collected-the-dataset).

* Rename the dataset folder to ```dataSetFile``` and place it under ```src/planner_train```.


**2. Train**
```bash
cd src/planner_train
micromamba activate yourEnvName # Recommend using micromamba for virtual environments. Conda or others also work.
python3 train.py
```
After training is finished, place the generated ```policy_model.pt``` file into ```src/planner/src/plan_manage/nnModels``` for online end-to-end planning.

Note:

* Training runs on GPU if the terminal prints "Using device: cuda" at startup.

* The ```model_name``` in ```src/planner/src/plan_manage/launch/model.yaml``` must match the name of the policy model you obtained from training.

## Collect the dataset

**1. Install dependencies**

* Install HDF5 library for dataset storage:
```bash
sudo apt update
sudo apt install libhdf5-dev
```

* Install [OSQP](https://osqp.org/docs/index.html) for QP solving:
```bash
# Install osqp
git clone --depth 1 --branch v1.0.0.beta1 https://github.com/osqp/osqp.git 
cd osqp
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# Install OsqpEigen
cd ../..
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

**2. Compile the dataset collection code**

```bash
cd src/data_set_gen
catkin_make
```

**2. Run the dataset collection code**

```bash
source devel/setup.bash
roslaunch data_set_gen data_set_gen-1.launch # or data_set_gen-2.launch
```
The dataset will be generated in the ```dataSetFile``` folder.

Note: 

* Set the ```enable_parallel``` in the launch file to ```true``` for multi-threaded parallel computation, or to ```false``` for single-threaded computation with collection process visualization.

## Citation

Thank you for your citation if this repo helps your research:

```bibtex
@ARTICLE{11704699,
  author={Liu, Zhitao and Xu, Guangtong and Wang, Zihan and Hou, Jialiang and Xu, Chao and Gao, Fei},
  journal={IEEE Transactions on Industrial Informatics}, 
  title={Neural-Primitive: An Efficient End-to-End Local Planner With Primitive-Based Imitation Learning for Autonomous Flight}, 
  year={2026},
  volume={},
  number={},
  pages={1-12},
  doi={10.1109/TII.2026.3726983}}

```

