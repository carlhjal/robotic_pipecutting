# robotic_pipecutting

A robot-agnostic package for robotic pipe cutting and welding trajectory trialing

## Installation
``` bash
cd ~/ws_folder/src
git clone https://github.com/carlhjal/robotic_pipecutting.git
cd ..
```

Install dependencies:
``` bash
vcs import src < src/robotic_pipecutting/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
```

Build the repository
```
colcon build
```

### Python Environment

It is recommended to install additional python dependencies in a virtual environment:

``` bash
cd ~/ws_folder
python3 -m venv .venv
source .venv/bin/activate
pip install -r src/robotic_pipecutting/requirements.txt
```

### Running the path planning script

Make sure you have sourced the virtual environment and the workspace build files

``` bash
source .venv/bin/activate
source install/setup.bash
```

Launch the path planner

``` bash
python3 src/robotic_pipecutting/reach_planner/scripts/guitest_welding.py
```
