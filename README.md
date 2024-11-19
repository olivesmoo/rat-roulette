# rat-roulette

## Running the Turtlebot
### Step 1:
```
source ~/.bashrc
cd ros2_ws
colcon build
source install/setup.bash
```
### Step 2: 
#### Option 1 (in a separate terminal):
```
ros2 launch depthai_examples mobile_publisher.launch.py
ros2 run rat_roulette_pkg rat_roulette_node
```
#### Option 2:
```
ros2 launch rat_roulette_pkg roulette_launch.py
```

## Push Code
```
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519_turtwig
```
Passkey: Turtwig
