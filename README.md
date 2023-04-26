# robotic-agriculture
This repository contains code used to control the robotic agriculture system. The most important file is [robotic-agriculture.py](robotic-agriculture.py), which is the MicroPython code that controls the entire system.
## Setting up environment

First time:
```
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```
If you are running into issues where it seems like files/libraries are missing, run the second and third commands again to update the dependencies.

Each time after that
```
venv\Scripts\activate
```
before running any files, installing packages, etc.

## Putting files onto ESP32
We are using MicroPython. Use the commands in [this tutorial](https://docs.micropython.org/en/latest/esp32/tutorial/intro.html) to get it working. Ideally the flash should only need to happen once, and then after that we will be able to edit the functionality of the system by editing `boot.py` using [ampy](https://github.com/scientifichackers/ampy)
```
ampy --help
```

When I want to just run the file, I (Caleb) run this on the command line:
```
ampy --port COM10 run -n main.py
```
You may need to replace `COM10` with whatever port the ESP32 shows up as on your computer

