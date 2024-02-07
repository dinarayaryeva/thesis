# Bachelor thesis repository

code - programming part of the thesis\
thesis - tex project with thesis

### Code structure:

├── **code** \
&emsp;├── **control_system**&emsp;# control module\
&emsp;&emsp;├── **assets**&emsp;# stl & xml models\
&emsp;&emsp;├── **config**&emsp;\
&emsp;&emsp;├── **control_system**\
&emsp;&emsp;&emsp;├── controllers.py&emsp;# model-based pid and sliding mode\
&emsp;&emsp;&emsp;├── srb_model.py\
&emsp;&emsp;&emsp;├── quat_routines.py\
&emsp;&emsp;&emsp;├── sympy_routines.py&emsp;# symbolic functions for srb\
&emsp;&emsp;├── **notebooks**&emsp;# ipynb\
&emsp;&emsp;├── **tests**\
&emsp;&emsp;&emsp;├── test_simulator.py&emsp;# test Simulator\
&emsp;&emsp;&emsp;├── test_bluerov.py&emsp;# test BluerovSim\
&emsp;&emsp;&emsp;├── test_sensors.py\
&emsp;&emsp;&emsp;├── test_control.py&emsp;# test sliding mode with BluerovSim\
&emsp;&emsp;├── setup.py&emsp;# <u>install before tests</u>\
&emsp;├── **mj_simulator**&emsp;# simulator module\
&emsp;&emsp;├── **mj_simulator**&emsp;# simulator module\
&emsp;&emsp;&emsp;├── **routines**\
&emsp;&emsp;&emsp;├── render.py\
&emsp;&emsp;&emsp;├── simulator.py\
&emsp;&emsp;&emsp;├── shared_types.py\
&emsp;&emsp;&emsp;├── bluerov.py&emsp;# bluerov simulator\
&emsp;&emsp;├── setup.py&emsp;# <u>install before tests</u>\
&emsp;├── enviroment.yml&emsp;# conda env