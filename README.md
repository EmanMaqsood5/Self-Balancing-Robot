# Self-Balancing Robot

A two-wheeled robot that balances itself on an ESP32. Made for the Control Systems Lab at UET Lahore.

It stands up using an MPU-6050 (to sense tilt) and two motors. I tried a few different ways of controlling it, and each one has its own folder.

## What's inside

**`simple-balancing/`** – the main one. The robot balances with a PID controller.
- `balance_robot_code.ino` – the robot code
- `lab14_model_identification.m` – works out the robot's physical model (moment of inertia) from the data I recorded

**`lfr/`** – balances *and* follows a light.
- `lfr_light_following_robot.ino`

**`lead-lag-controller/`** – same robot, but using a lead-lag controller instead of PID.
- `leadlag_balance_robot.ino` – the robot code
- `lab15_lead_lag.m` – designs the controller from the Lab 14 model and gives the numbers used in the code

## How to run it

1. Open an `.ino` file in the Arduino IDE.
2. Install the **ESPAsyncWebServer** library (and its AsyncTCP dependency).
3. Pick your ESP32 board, upload, and put the robot upright.
4. The robot makes its own Wi-Fi network so you can watch live data from your phone or laptop.

For the MATLAB files, just open them and hit Run. Lab 14 needs a data file called `eman_data_phase3.csv` in the same folder (not included here).

## Things to know

- The balance angle (0.90°) and motor deadband were tuned for *my* robot. Yours will probably need different numbers.
- The model in Lab 14 is an estimate. The robot didn't balance cleanly for most of the recording, so it's a best guess, not a perfect fit.
- The Wi-Fi name and password are in the code. Change them if you care.
