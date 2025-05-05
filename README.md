# robotic_pipecutting

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
