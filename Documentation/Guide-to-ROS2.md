# Guide to ROS2
**Additional tips about ROS2 I have learned and documented while creating this repo.**

## Creating a ROS2 Workspace
*NOTE: this is not necessary if pulling from GitHub and is for informational purposes only.*
1) Install build tools (these instructions were from the crash course video linked on the main README): `sudo apt update`, `sudo apt install python3-colcon-extensions`
3) Navigate to where you want to make the ROS2 workspace and `mkdir meca_ws` (creates a folder named meca_ws).
4) Navigate into meca_ws and `mkdir src` inside.
5) ***Navigate into the meca_ws directory*** and `colcon build`. You should get a success message; if not, something is wrong.
6) `gedit ~/.bashrc` (or vim) and add line `source ~/meca_ws/install/setup.bash` (use the name of your workspace) and `source ~/.bashrc` **This is if you are using bashrc. I ended up making a shell script that can be pushed to GitHub and combines many commands (see README), but I still run source with the install/setup.bash every time.**
8) `ros2 pkg create meca_controller --build-type ament_python --dependencies rclpy` **This creates a ros2 package called meca_controller.**

## Adding On and Creating a New ROS2 Package
Here I am going to document the steps to make a new ROS2 package--in my case, I am making a ROS2 package for the motion planning algorithm I had implemented.

The current structure looks like this:
```
jessicamyers@MightyMeca:~/Jupyter/mecademic-ros2/meca_ws$ ls
build  install  log  mecavenv  src

jessicamyers@MightyMeca:~/Jupyter/mecademic-ros2/meca_ws/src$ ls
meca_controller
```

What we want to do here is create another package like meca_controller.

1) Make sure you are in `mecademic-ros2/meca_ws/src`
2) Run `ros2 pkg create meca_motion_planner --build-type ament_python --dependencies rclpy` Replace meca_motion_planner with whatever you want the package to be called. Now we have:

```
jessicamyers@MightyMeca:~/Jupyter/mecademic-ros2/meca_ws/src$ ls
meca_controller  meca_motion_planner
```
3) Follow the steps below to create a node.

## Making a Node:
Within the `src/meca_controller/meca_controller` (package) folder of your workspace, you can create a new python node with `touch my_first_node.py`. Then be sure to make the file executable by doing `chmod +x my_first_node.py`. (Obviously, name it whatever you want). There should be a `setup.py` file in your `src/meca_controller` folder. If you open it, you should see an entry_points section; you will have to define your node here in order to be able to ros2 run it like `ros2 run meca_controller meca_driver`. Within the console_scripts list, add your node; below, meca_driver is the name of the node, and then after the equal sign, you must put `package_name.file_name:function_name`.

```
entry_points={ # package name, file name, func
        'console_scripts': [
            "meca_driver = meca_controller.meca_driver:main"
        ],
    },
```

## Services vs topics vs actions
See [this link](https://docs.ros.org/en/humble/How-To-Guides/Topics-Services-Actions.html) for whether you should be using a topic, service, or action for message passing in the system. Also, if you are adding on, always question the current code because it is always possible we could be doing something the wrong way, and no need to propagate the errors forward. Also, if you are going to use services, there are additional dependencies you should specify when creating the package--but these can be added later as well in the package.xml file.

[How to create a simple service node](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html#create-a-package)


[How to prevent deadlocks - async vs sync service calls](https://docs.ros.org/en/humble/How-To-Guides/Sync-Vs-Async.html). When you call a service asynchronously, there are two ways in which you could choose what to do with the returned `future`:
1) Use a callback, which will be called once the service call completes:
    ```
    future = client.call_async(request)
    future.add_done_callback(partial(self.callback_get_motion_plan))
    ```
2) As in the link above, use a safe loop that keeps the node spinning but also keeps checking for the completion of the call:
   ```
   while rclpy.ok():
        rclpy.spin_once(node)
        if future.done():
            #Get response
   ```

## Creating a Custom Message or Service Format
Also, if you are curious about how to make custom message formats like sensor_msgs/JointState and geometry_msgs/Point, see [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).
1) First I made a new ros2 package: `ros2 pkg create --build-type ament_cmake custom_interfaces` *NOTE: this uses cmake--which is the only way for creating custom msg and srv, and is okay! See this following excerpt from the website: `tutorial_interfaces is the name of the new package. Note that it is, and can only be, a CMake package, but this doesn’t restrict in which type of packages you can use your messages and services. You can create your own custom interfaces in a CMake package, and then use it in a C++ or Python node, which will be covered in the last section.`* **Also, note that I had to `python3 -m pip install catkin_pkg` for this to work. I have added it to the earlier venv instructions.**
2) Following the instructions at the website, I did `mkdir msg srv`.
3) To make a custom message, inside the msg directory I did `touch MotionPlan.msg` and then edited it to make a data structure.
4) I wanted to make a 2D array of joint angles, but so far I don't think it is possible so I am going to use reshape and store width and height of the array like in [this post](https://answers.ros.org/question/364269/send-a-2d-array-through-topics-in-ros2/).
5) I followed the rest of the instructions for editing CMakeLists.txt and package.xml.
6) I also needed `python3 -m pip install empy lark`.
7) To test the GetMotionPlan service and MotionPlan msg, I continued with the instructions at the tutorial link, creating client and service nodes inside the meca_motion_planner package and updating dependencies accordingly in package.xml `<depend>custom_interfaces</depend>`. Note that you also need to edit setup.py like in [this link](https://automaticaddison.com/how-to-create-a-service-and-client-python-ros2-foxy/). Also, dealing with arrays/lists can be tricky--be careful of using the correct datatype (float), and Python lists seem to work instead of np arrays in my current understanding.

## ROS2 vs ROS1
Here I'll provide some links I came across explaining the differences:
- https://design.ros2.org/articles/changes.html
- https://roboticsbackend.com/ros1-vs-ros2-practical-overview/

