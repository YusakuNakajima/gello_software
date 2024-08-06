
# Get offset for Gello robot
## for UR
python3 scripts/gello_get_offset.py --start-joints 0 -1.57 1.57 -1.57 -1.57 0 --joint-signs 1 1 -1 1 1 1  --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISUQE-if00-port0
## for cobotta
python3 scripts/gello_get_offset.py --start-joints 0 0 1.57 0 1.57 0 --joint-signs 1 1 -1 1 -1 1  --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISUQE-if00-port0

## Check serial port
```
ls /dev/serial/by-id/
```
## Output
```
best offsets               :  ['9.425', '4.712', '3.142', '4.712', '1.571', '1.571']
best offsets function of pi: [6*np.pi/2, 3*np.pi/2, 2*np.pi/2, 3*np.pi/2, 1*np.pi/2, 1*np.pi/2 ]
gripper open (degrees)        113.091015625
gripper close (degrees)       71.291015625
```

