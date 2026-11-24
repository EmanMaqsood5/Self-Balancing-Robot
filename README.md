# Self-Balancing Two-Wheel Robot (ESP32)


## What is this project?

We built a small two-wheeled robot that **stands up on its own** without falling over. It's like balancing a broom on your hand: the moment it starts to tilt, the wheels drive in the direction of the tilt to get back under it.

The project has four parts, each building on the one before:

| Part | What we did | Lab |
|------|-------------|-----|
| 1 | Built the robot and made it balance using a **PID controller** | Lab 12 |
| 2 | Made it also **follow a light** while balancing (two controllers working together) | Lab 13 |
| 3 | Worked out the robot's **physical model** (its inertia) from recorded data | Lab 14 |
| 4 | Designed a better controller on paper using that model (**Lead-Lag**) and put it on the robot | Lab 15 |

---

## How the robot works

1. **Sense the tilt.** The MPU6050 chip measures how fast the robot is rotating (gyroscope) and which way gravity pulls (accelerometer). Neither is perfect alone: the gyro drifts over time and the accelerometer is jittery. So we blend them with a **complementary filter** (mostly gyro for quick changes, a little accelerometer to stop drift).
2. **Decide what to do.** A controller looks at the tilt error (how far we are from upright) and calculates how hard to drive the motors.
3. **Move the wheels.** The ESP32 sends that command to the L298N motor driver, which spins the motors forward or backward.
4. **Repeat 200 times every second.**

---

## Hardware

| Part | What it does |
|------|--------------|
| ESP32 dev board | The brain. Runs everything, and also makes its own WiFi network |
| MPU6050 | Tilt sensor (gyro + accelerometer) |
| L298N | Motor driver (takes ESP32 signals, powers the motors) |
| 2 × DC gear motors | Drive the wheels |
| Battery (7.4 V LiPo or 9–12 V pack) | Powers the motors, and the ESP32 through a regulator |
| Voltage regulator (L7805 / L7807 type) | Gives the ESP32 a steady supply so motor spikes don't reset it |
| 2 × LDR + 10 kΩ resistors | Light sensors (Part 2 only) |

### Pin connections

| Signal | ESP32 pin |
|--------|-----------|
| MPU6050 SDA | GPIO 21 |
| MPU6050 SCL | GPIO 22 |
| MPU6050 VCC / GND | 3.3 V / GND |
| MPU6050 AD0 | GND (address 0x68) |
| Motor A speed (PWMA / ENA) | GPIO 25 |
| Motor A direction (AIN1 / IN1) | GPIO 26 |
| Motor A direction (AIN2 / IN2) | GPIO 27 |
| Motor B speed (PWMB / ENB) | GPIO 32 |
| Motor B direction (BIN1 / IN3) | GPIO 33 |
| Motor B direction (BIN2 / IN4) | GPIO 14 |
| Left LDR (Part 2) | GPIO 34 |
| Right LDR (Part 2) | GPIO 35 |

**Power wiring in short:** battery + goes to the L298N VS pin and the regulator input. Regulator output goes to the ESP32 VIN. **All grounds must be joined together.**

**Tip:** Mount the heavy stuff (battery) high up. A higher centre of mass gives the motors more leverage and makes balancing easier.

---

## The WiFi dashboard

The ESP32 makes its own WiFi network, so no router is needed.

- **Network name:** `ESP_burners`, **password:** `password123`
- **Open in your browser:** `http://192.168.4.1`

From the page you can see the live tilt angle and **change the controller gains while the robot is running** (no re-uploading code). There is also an **Emergency Stop** button. In Part 2 it also shows the light sensor readings.

---

## Part 1 – PID balancing

**PID** has three pieces:
- **P (Proportional):** push back in proportion to how far we've tilted.
- **I (Integral):** fixes a small leftover lean that never goes away.
- **D (Derivative):** calms things down so the robot doesn't wobble.

Safety features in the code:
- If the tilt goes past the limit, the motors stop (robot has fallen).
- It re-enables itself when you stand the robot back up near balance.
- The integral term is limited so it can't "wind up" and cause a lurch.
- A **deadband** adds a minimum motor power, because motors don't move at very low PWM.
- The gyro is calibrated at startup. **Hold the robot still when you power it on.**

### How we built it (step by step)
We tested each piece alone before putting everything together:
1. Tested the L298N and motors directly from the battery.
2. Blinked an LED to confirm the ESP32 works.
3. Drove the motors from the ESP32.
4. Read the MPU6050 and checked the angle on the Serial Monitor.
5. Assembled everything and tuned PID over USB.
6. Added the regulator, ran on battery, and tuned wirelessly over WiFi.

---

## Part 2 – Balancing + following light

Now the robot has **two jobs** at once:

| Loop | Job | Speed |
|------|-----|-------|
| Inner (fast) | Stay balanced | 200 Hz (every 5 ms) |
| Outer (slow) | Turn toward the light | 10 Hz (every 100 ms) |

Because the turning loop is 20× slower than the balance loop, they don't fight each other. The balance loop sorts out any wobble before the turning loop even notices.

**How the light steering works:**
- Each LDR is in a voltage divider with a 10 kΩ resistor, read by the ESP32's 12-bit ADC (0–4095).
- Light error = left reading − right reading.
- A small PID turns that error into a steering command (`u_dir`).
- The motors get: **left = balance + steering**, **right = balance − steering**.

Problems we ran into:
- **Noise on the ADC** from the motor PWM, which made the robot twitch. Fixed by reading the sensors only at 10 Hz and clamping the steering.
- **Too much steering** made the robot fall (it knocks the balance). Fixed by lowering the gain and limiting the steering.
- **Robot kept turning after a fall** because the old steering value was still stored. Fixed by resetting it to 0 on a fall.
- **The robot's body shadowed one sensor** in sharp turns. Fixed by mounting the LDRs higher up.

---

## Part 3 – Finding the robot's physical model

Up to now we tuned PID by trial and error. In this part we worked out the maths behind the robot: it's an **inverted pendulum**.

```
tilt acceleration = (m·g·h / J) × tilt  +  (1 / J) × motor torque
```

- `m` = mass (1.10 kg), `h` = height of centre of mass above axle (0.12 m), `g` = 9.81
- `J` = moment of inertia: how hard the robot is to rotate. This was the unknown.

The plan was to push the robot, record the wobble, and match it to the model. But our recordings weren't clean: the robot was stuck leaning for much of the session and only balanced briefly. So we used two methods:
1. **Physics guess** for J from the dimensions.
2. **Frequency matching:** take the wobble frequency from the short balanced stretch (using an FFT), then find the J that gives that same frequency in the model.

**Result:** J = **0.0654 kg·m²**, giving the plant model:

```
G(s) = 15.2898 / (s² − 19.7990)
```

This has one unstable pole at +4.45 rad/s, which is the mathematical way of saying "it falls over by itself."

The report also describes a cleaner method (record an impulse response with a buffer in the firmware, send it to the PC, and fit it). The code for that is included in the snippet files below.

---

## Part 4 – Lead-Lag controller

Using the model from Part 3, we designed a controller from the frequency response instead of guessing:

- **Lead part** adds phase (stability): zero at 6.67 rad/s, pole at 63.2 rad/s.
- **Lag part** boosts low-frequency gain: zero at 2.57 rad/s, pole at 0.32 rad/s.
- **Gain K = 88.2176**, chosen so the crossover is at about 20.5 rad/s.

It was converted to a form the ESP32 can run (Tustin method at 200 Hz):

```
u[k] = 77.88543924·e[k] − 152.22080282·e[k-1] + 74.36797451·e[k-2]
       + 1.72548152·u[k-1] − 0.72591912·u[k-2]
```

(This is the same as `u0 = b0·e0 + b1·e1 + b2·e2 − a1·u1 − a2·u2` in the code.)

**Simulation results**

| | Lead-Lag |
|---|---|
| Phase margin | 47.77° |
| Overshoot | 37.32 % |
| Settling time | 0.6256 s |
| Rise time | 0.0501 s |
| Closed-loop poles | all stable |

In the simulation, the PID from Lab 12 came out unstable under the same conditions, which is the point the report makes for model-based design.

---

## Files in this project

| File | What it is |
|------|------------|
| `balance_robot_code.ino` | Part 1 PID balance code with the web tuner (see the warning below) |
| `lfr_light_following_robot.ino` | Part 2 full code: balance + light following + dashboard |
| `lab14_model_identification.m` | Part 3 MATLAB: finds J from the data |
| `cep3_matlab_snippets.m` | Part 3 MATLAB snippets from the report (import, fit, plots, filter) |
| `cep3_datalogging_snippets.ino` | Part 3 ESP32 snippets: buffer, disturbance trigger, data dump |
| `capture_impulse.py` | Optional Python script to save the serial data to a CSV |
| `lab15_lead_lag.m` | Part 4 MATLAB: designs the lead-lag controller and plots results |
| `leadlag_balance_robot.ino` | Part 4 full Arduino code using the lead-lag controller |
| `leadlag_update_snippet.c` | Small C example of the lead-lag difference equation |

---

## How to run it

**Robot code (Arduino IDE)**
1. Install the **ESP32 board package**.
2. Install the **ESPAsyncWebServer** library (and AsyncTCP, which it needs).
3. Select board **ESP32 Dev Module** (or NodeMCU-32S).
4. Open the `.ino` you want, upload over USB.
5. **Keep the robot still while it boots**, since this is when the gyro calibrates.
6. Connect your phone to the ESP32's WiFi, open `192.168.4.1`, and tune.

**MATLAB**
- `lab15_lead_lag.m` runs on its own (needs the Control System Toolbox).
- `lab14_model_identification.m` needs the data file `eman_data_phase3.csv` in the same folder (serial log lines containing `Angle:`, `Target Setpoint:`, `PID:`).

---

## Tuning values

| | Part 2 code | Part 2 report text |
|---|---|---|
| Kp / Ki / Kd | 50 / 0.8 / 3 | 30 / 0 / 0.5 |
| Setpoint | 0.9° | −5.0° |
| Safety limit | 25° | 35° |
| Light Kp / Ki / Kd | 0.25 / 0.008 / 0.08 | 0.05 / 0 / 0.01 |
| Steering limit | ±110 | ±50 |

Tuning tips:
- Raise **Kp** until it reacts firmly, but not so much that it shakes.
- Raise **Kd** to calm the shaking.
- Use **Ki** only if there's a slow leftover lean.
- Find the **setpoint** by holding the robot at its natural balance point and reading the angle.

---

## Things to know

- **`balance_robot_code.ino` is incomplete.** It was copied from screenshots in the Part 1 report, and a few lines were not visible. It is missing the start of the HTML string, one line in `loop()`, and the declarations of `isEnabled`, `P`, `D` and `previousError`. Fill these in from your original sketch before compiling.
- **Code and report disagree in places** (see the table above). The Part 2 report text lists different gains from the ones in the code. Decide which is your final version and make them match.
- **The Part 1 report mentions a `BalanceBot` network**, but the code uses `ESP_burners`.
- **The Part 3 data was poor.** J came from the short balanced window, so it is an estimate, not a precise measurement.
- **The lead-lag step response overshoots by about 37 %.** The phase margin target was met, but this is worth mentioning in discussion.


---

## References

1. Espressif Systems, *ESP32 Technical Reference Manual* (2023)
2. InvenSense/TDK, *MPU-6000 and MPU-6050 Product Specification*, Rev 3.4
3. STMicroelectronics, *L298 Dual Full-Bridge Driver Datasheet*
4. Åström & Hägglund, *Advanced PID Control*, ISA (2006)
5. Arduino reference pages for `analogWrite()` and the Wire library
