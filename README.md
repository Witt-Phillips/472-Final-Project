<h1 align="center">CPSC 472 Final Project</h1>
<h2 align="center"> Table of Contents </h2>

- [Getting Webots to work with PyCharm](#getting-webots-to-work-with-pycharm)
  - [Useful links:](#useful-links)
  - [Downloading PyCharm](#downloading-pycharm)
  - [Workspace Settings](#workspace-settings)
    - [1. Set PyCharm Project Settings](#1-set-pycharm-project-settings)
    - [2. Set up PyCharm Profiler](#2-set-up-pycharm-profiler)
    - [3. Set Webots Controller to External](#3-set-webots-controller-to-external)
    - [4. Running the Interface](#4-running-the-interface)
      - [4.1 Pycharm Scientific Mode](#41-pycharm-scientific-mode)
  - [Using the Interface](#using-the-interface)
- [Running Webots 2021b on arm64](#running-webots-2021b-on-arm64)
  - [TL;DR](#tldr)
    - [Explaination](#explaination)
  
---

## Getting Webots to work with PyCharm

### Useful links:
- [Webots, Using your IDE](https://cyberbotics.com/doc/guide/using-your-ide?tab-os=macos&tab-language=python)
- [Webots, Running and External Controller](https://cyberbotics.com/doc/guide/running-extern-robot-controllers?version=R2021b&tab-language=python&tab-os=macos)
  
### Downloading PyCharm
So you can get Pycharm from [JetBrains](https://www.jetbrains.com/community/education/#students) for free if you have a student email; I think there is just a slight email and verification process.

Once you have PyCharm installed you can open the project folder in Pycharm with the base directory being the repo folder from GitHub. The scripts that I have been working with are all in the lib/ folder and the binaries are in the bin/ folder.

### Workspace Settings

#### 1. Set PyCharm Project Settings
Go to PyCharm > Settings -> Project: 472-Final-Project
- Make sure that the project interpreter is set to Python 3.8 in 472-Final-Project/bin/python
![Alt text](refs/image.png)

#### 2. Set up PyCharm Profiler
Set up the Profiler; Run > Edit Configurations ... > + > Python Profiler
>- Name it whatever you want
>- Script: (the controller file that you want to run this with) e.g. im using it with 472-Final-Project/lib/Zombie World/controllers/calbick_controller_template/calbick_controller_template.py
>- Now we have to set some environment variables so that the controller can find the Webots library
>> Click the little icon to the right of the Environment Variables text box and a window like this will appear:
![Alt text](refs/image-3.png)
>>> We need to set these variables (**/Users/danielcalbick/science/pycharm_projects/cpsc472** should be replaced with your own path to the project folder):
>>> ```bash
>>> WEBOTS_HOME=/Applications/Webots.app
>>> LD_LIBRARY_PATH="/Applications/Webots.app/projects/robots/kuka/youbot/libraries/youbot_control:/Users/danielcalbick/science/pycharm_projects/cpsc472/lib/Zombie world/libraries/youbot_control:/Users/danielcalbick/science/pycharm_projects/cpsc472/lib/Zombie world/controllers/validate_inputs:/Applications/Webots.app/lib/controller"
>>> PYTHONPATH=/Applications/Webots.app/lib/controller/python38_brew
>>> PATH=/Applications/Webots.app/bin:$PATH
>>> QT_QPA_PLATFORM_PLUGIN_PATH=/Applications/Webots.app/lib/webots/qt/plugins
>>> ```
>- We need to set the Python interpreter to the **intell version of python we put in the Webots python interpreter path**: /usr/local/bin/python3.8
>- Set the working directory to the directory of the controller file you are running; I also checked the box to add content roots to PYTHONPATH, which I think helps with finding all the other zombie controller files
>- OK  <br>
![Alt text](refs/image-1.png) <br>
> NOTE: I saved this configuration by checking the "store as project file" box and saving it in the .idea/runConfigurations folder so that it can be shared with the rest of the team, you should just be able to add it but I wanted to let you know the process <br>
> ![Alt text](refs/image-2.png)
>> Also, my directory git is linked to "cpsc472/" so if you see this in a path it would be the "472-Final-Project/" folder in the repo if you just pulled straight from GitHub 
><br>

#### 3. Set Webots Controller to External
The last thing we need to change is the Webots controller within the Youbot note in the scene tree
>- Open the scene tree and click on the youbot node
>- In the properties window on the right, click on the "Controller" and in the smaller dropdown menu select **\<extern>**, this should already be selected if you loaded the world file from the lib/Zombie world/worlds/zombiesAndBerries.wbt but I'm not entirely sure ![Alt text](refs/image7.png)

#### 4. Running the Interface
Now you should be able to run the controller file with the profiler 
> - Run > Profile 'calbick_controller_template' or simply click the play button in the blue profiler box 
> > This will start a python console and run the code initializing python ![Alt text](refs/image-4.png)
> - You won't have control of the console until you click the play button in Webots to start the simulation
##### 4.1 Pycharm Scientific Mode
There is a mode called "Scientific Mode" (View > Scientific Mode) which is a useful way to run this for a couple reasons:
- It has a built in console that you can use to run commands and see the output
- It has a built in variable viewer that you can use to see the values of variables in the program
- You can run the program line by line or in code blocks like in matlab by insterting #%% in the code; this allows you to run the block of code by pressing the green play button next to the block declaration

### Using the Interface
Now you can run the controller in Webots in an interactive way
> - I have been using the sanbox I put at the bottom of the controller file to test things out
> ```python
>[591]    #%% ----------- Sandbox -----------
>[592]
>[593]     world_map = worldMapObject()
>[594]
>[595]     # Initialize youbot in world with sensors
>[596]     youbot   = init_youbot(world_map)
>[597]     robot    = youbot.wb_robot
>[598]     timestep = world_map.timestep
>[599]     
>[600]     passive_wait(0.1, robot, timestep)
>[601]     pc = 0
>[602]     timer = 0
>[603]     
>[604]     robot_node = robot.getFromDef("Youbot")
>[605]     trans_field = robot_node.getField("translation")
>[606]     
>[607]     get_all_berry_pos(robot)
>[608]     
>[609]     #%%
>[610]     
>[611]     tmp = robot.step(TIME_STEP)
>[612]     youbot.sensors["gps"].getValues()
>```
>- The first time the controller runs through it will move one time step in the simulation and you will have full control through the console. Running the code block at ```[609]``` is an example of a simple use case, runing a time step and getting the gps values from the youbot. You can run this block over and over again to see how the gps values change as the youbot moves around the world.
>- *Further*, you can still use the Webots interface to move the youbot around and interact with the world. The console will still be active and you can run the code block at ```[609]``` to see how the gps (or any other sensor) values change as you move the youbot around the world. This is how I plan to build the image processing piece because I can put the youbot right infront of whatever I want, and read the sensor value playing around with the data in PyCharm's console and variable viewer.
> <br>

---
<p>
Let me know if you guys have any issues with the setup or if you have any questions about how to use PyCharm. I think this will be a really useful tool for us to use to develop the controller and I hope you guys can get it working on your machines.
</p>

---

<p align="center">
Here's a video of me using it:
</p>

<p align="center">
  <img src="refs/Pychar-Webots.gif" alt="temp" width="75%">
</p>

---

## Running Webots 2021b on arm64
### TL;DR 

```bash
# 1. In terminal switch to x84 via 
> env /usr/bin/arch -x86_64 /bin/zsh --login
# 2. Install an x84 version of brew 
> /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
# 3. Install python w/ x84 brew 
> /usr/local/bin/brew install python@3.8
# 4. Set the binary in Webots to the intel version:
/usr/local/bin/python3.8
```

#### Explaination
If you're interested, the reason we need to do this is because Webots 2021b was not yet compatible with the new arm64 architecture. To get older versions of Webots to run on new arm64-based Apple computers there a few things we need to address. 

The main issues are that the Webots dependencies are looking for x86_64 based Python installations. Normally, the most common way to install Python is through homebrew in the terminal—-which also has a both a x86_64 and arm64 version. 

The arm64 version is the default on M1 Macs and is stored in a different binary directory (/opt/homebrew/bin/brew) and subsequently installs all versions of Python as arm64 and in a different location than the x86_64 Homebrew (/usr/local/bin/brew). 

Webots is looking for a compatible Python installation in the x86_64 Homebrew directory, so we need to install a x86_64 version of Homebrew and Python.