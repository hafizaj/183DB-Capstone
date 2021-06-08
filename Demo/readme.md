<p align="center">
  <a href="" rel="noopener">
 <img width=400px height=300px src="https://raw.githubusercontent.com/hafizaj/183DB-Capstone/main/Demo/busserbot.png" alt="Bot logo"></a>
</p>

<h3 align="center">Busserbot</h3>

---

<p align="center"> Autonomous busser for your business.
    <br> 
</p>

## 📝 Table of Contents
+ [About](#about)
+ [Demo / Working](#demo)
+ [How it works](#working)
+ [Getting Started](#getting_started)
+ [Deploying your own bot](#deployment)
+ [Built Using](#built_using)
+ [TODO](../TODO.md)
+ [Contributing](../CONTRIBUTING.md)
+ [Authors](#authors)
+ [Acknowledgments](#acknowledgement)

## 🧐 About <a name = "about"></a>
The Busserbot is a robotic busser designed by Team People for EC ENGR 183D/MAE 162D.

## 🎥 Demo / Working <a name = "demo"></a>
[![Full Cycle Simulation](https://raw.githubusercontent.com/hafizaj/183DB-Capstone/main/Demo/thumbnail/full-cycle.png)](https://www.youtube.com/watch?v=2HlgLPjuZLk "Full Cycle Simulation")
[![Single Stationary Simulation](https://raw.githubusercontent.com/hafizaj/183DB-Capstone/main/Demo/thumbnail/single.png)]({https://www.youtube.com/watch?v=q2SQm7otxc0&feature=youtu.be} "Single Stationary Simulation")
[![Storage Simulation](https://raw.githubusercontent.com/hafizaj/183DB-Capstone/main/Demo/thumbnail/storage.png)](https://www.youtube.com/watch?v=hxuN4XihSyQ&feature=youtu.be "Storage Simulation")


## 💭 How it works <a name = "working"></a>

The BusserBot utilizes the principles of trajectory planning to prescribes velocity of the joints.

The entire bot is written in Python 3.6

## 🏁 Getting Started <a name = "getting_started"></a>
These instructions will get you a copy of the project up and running on your machine to view the demo. 
---
### Prerequisites

What things you need to install the software and how to install them:

1. [Webots] (https://cyberbotics.com/)
2. [Anaconda] (https://www.anaconda.com/)
3. [brew] (https://brew.sh/)

### Installing

#### Windows

1. Install the latest version of [Webots] (https://cyberbotics.com/)
2. Install the latest version of [Anaconda] (https://www.anaconda.com/)
3. Upon installing, add directory of `anaconda3` to `PATH`, as follows `C:\Users\YOUR_USER_NAME\anaconda3\`
4. Launch Webots
5. Locate the directory of your `anaconda`'s version of python using `where python`
7. Under `Tools`, open preferences
8. Add` anaconda`'s version of python to the python parameter, e.g. `C:\Users\YOUR_USER_NAME\anaconda3\python.exe`

#### MacOS and Linux

1. Install [brew] (https://brew.sh/) from the website
2. Once brew is installed, execute the following code to install python:
```
brew install python
```
3. After python is installed, run the following code to view the directory of all python versions
```
ls -l /usr/local/bin/python*
```
4. You will see all the links created by brew to its Python install, similar to the following:

```
lrwxr-xr-x  1 username  admin  36 Oct  1 13:35 /usr/local/bin/python3@ -> ../Cellar/python/3.7.4_1/bin/python3
lrwxr-xr-x  1 username  admin  43 Oct  1 13:35 /usr/local/bin/python3-config@ -> ../Cellar/python/3.7.4_1/bin/python3-config
lrwxr-xr-x  1 username  admin  38 Oct  1 13:35 /usr/local/bin/python3.7@ -> ../Cellar/python/3.7.4_1/bin/python3.7
lrwxr-xr-x  1 username  admin  45 Oct  1 13:35 /usr/local/bin/python3.7-config@ -> ../Cellar/python/3.7.4_1/bin/python3.7-config
lrwxr-xr-x  1 username  admin  39 Oct  1 13:35 /usr/local/bin/python3.7m@ -> ../Cellar/python/3.7.4_1/bin/python3.7m
lrwxr-xr-x  1 username  admin  46 Oct  1 13:35 /usr/local/bin/python3.7m-config@ -> ../Cellar/python/3.7.4_1/bin/python3.7m-config
```
5. In the example above, the first row shows the `python3` symlink. To set is as the default `python` symlink, we run the following code
```
ln -s -f /usr/local/bin/python3 /usr/local/bin/python
```
6. Reload the shell using the following code
```
exec $SHELL -l
```
7. To verify that we have succesfully set our link, we can view our current definition of `python` as follows:

```
which python

---

/usr/local/bin/python
```
8. Execute the following code to install additional libraries
```
pip install scipy
```
9. Open Webots, and under the topleft Webots, go to `Preferences`
10. Change the python parameter to python's directory
---
## 🚀 Deploying your own bot <a name = "deployment"></a>
To see an example project on how to deploy your bot, please see my own configuration:

+ **Heroku**: https://github.com/kylelobo/Reddit-Bot#deploying_the_bot

## ⛏️ Built Using <a name = "built_using"></a>
+ Python
+ Webots

## ✍️ Authors <a name = "authors"></a>
+ [@hafizaj](https://github.com/hafizaj) - Coding/ Arm / Systems Integration Logic Design
+ [@semiragali](https://github.com/semiragali) - Data Analysis
+ [@shahrulkamil98](https://github.com/shahrulkamil98) - Storage Unit Logic Design
+ [@justincchandra](https://github.com/justincchandra) - Storage Unit Designer
+ [@anaravit](https://github.com/anaravit) - Robotic Arm Designer
+ [@rmartinez88](https://github.com/rmartinez88) - Vertical Lift/ Arm-Storage Integration Designer

## 🎉 Acknowledgements <a name = "acknowledgement"></a>
+ Ankur Mehta
+ Yusuke Tanaka
+ Jacob Rosen
+ Peter Ferguson
