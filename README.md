# LuminousMsg

This package contains custom message definitions for the Luminous project.

## Messages

This section should list and describe the messages defined in the `msg/` directory.
You can inspect the `msg/` directory to see the available message files (e.g., `MyMessage.idl`).

For each message, provide:
- **Message Name:** (e.g., `MyMessage`)
- **Description:** A brief explanation of what the message represents and its fields.
- **Key Field:** If applicable, specify the field that acts as a key or unique identifier for the message. This is often denoted using the `@key` annotation within the message definition file (e.g., in an `.idl` file).

## Building

This package is a ROS 2 package and can be built using Colcon.

1.  **Source your ROS 2 environment:**
    ```bash
    source /opt/ros/<your_ros_distro>/setup.bash
    ```
2.  **Navigate to your workspace root:**
    ```bash
    cd /path/to/your/Luminous_ws
    ```
3.  **Build the package:**
    ```bash
    colcon build --packages-select luminous_msgs
    ```
4.  **Source the local setup file:**
    ```bash
    source install/local_setup.bash
    ```

Replace `<your_ros_distro>` with your ROS 2 distribution (e.g., humble, foxy).
Replace `/path/to/your/Luminous_ws` with the actual path to your Luminous workspace.

## Usage

After building and sourcing, these messages can be used in your ROS 2 nodes by including the appropriate header files and specifying the package dependency in your `package.xml`.

Example C++ include:
```cpp
#include "luminous_msgs/msg/my_message.hpp"
```

Example Python import:
```python
from luminous_msgs.msg import MyMessage
```
