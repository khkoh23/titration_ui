# titration_ui

PySide2-based ROS 2 UI node for titration experiments:
- Node name: `titration_ui`
- Subscribes: `/ph` (std_msgs/Float32), `/temperature` (std_msgs/Float32), `/titration_vol` (std_msgs/UInt32)
- Publishes: `/pipette_cmd` (std_msgs/Int8) via UI buttons
- Real-time plot: pH vs titration volume (Y: 0..14, X: 0..max volume)
- Save Data: writes a timestamped CSV with columns `Time, pH, Temperature, Titration volume`

## Build

```bash
# In your colcon workspace
mkdir -p ~/titration_ws/src
cd ~/titration_ws/src
# Place the titration_ui/ package directory here
cd ..
colcon build
source install/setup.bash
