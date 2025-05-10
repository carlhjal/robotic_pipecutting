# robotic_pipecutting

A robot-agnostic package for robotic pipe cutting and welding trialing

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

``` bash
cd ~/ws_folder
python3 -m venv .venv
source .venv/bin/activate
pip install -r src/robotic_pipecutting/requirements.txt
```
