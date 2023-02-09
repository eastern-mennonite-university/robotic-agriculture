# robotic-agriculture


a;slkdfgh;dflkgjsd;flgkjs

## Setting up environment

First time:
```
python -m venv venv
pip install -r requirements.txt
venv\Scripts\activate
```

Each time after that
```
venv\Scripts\activate
```
before running any files, installing packages, etc.

## Putting files onto ESP32
We are using MicroPython. Use the commands in [this tutorial](https://docs.micropython.org/en/latest/esp32/tutorial/intro.html) to get it working. Ideally the flash should only need to happen once, and then after that we will be able to edit the functionality of the system by editing `boot.py` using [ampy](https://github.com/scientifichackers/ampy)
```
ampy --help
``