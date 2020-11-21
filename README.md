# raspi-hand-detector
An image processing algorithm using OpenCV that read camera images, estimate hand and fingers positions to send velocities commands to a remote wheeled robot.

Bachelor final project of Robotics and Automation Engineering Course - Federal University of ABC
* Monograph title: Computer Vision Applied to Emdedded Systems

## Installation
Clone this repository to your Raspberry Pi or Ubuntu/Linux.
Check required lib versions:
- OpenCV 3.0.0+
- Python 2.7+

## Usage example
run `tg3.py` with proper arguments:
* --display
    * **1** - Show display
    * **0** - Don't show display
* --raspberry
    * **1** - Use raspberry pi configs and libs
    * **0** - Use only ubuntu configs and libs
* --send
    * **1** - Send commmands to remote host
    * **0** - Don't send commmands to remote host

Ex: Running on raspberry pi, sending commands to the remote host and showing results on raspberry pi display
```
python tg3.py --display 1 --raspberry 1 --send 1
```

## License
Distributed under the MIT license. See ``LICENSE`` for more information.

## Contributing
1. Fork it (<https://github.com/rodriguesrenato/raspi-hand-detector/fork>)
2. Create your feature branch (`git checkout -b feature/contribuition`)
3. Commit your changes (`git commit -am 'About your contribuitions'`)
4. Push to the branch (`git push origin feature/contribuition`)
5. Create a new Pull Request


