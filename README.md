# IRobot‑Create3‑Navigation
Autonomous navigation and delivery behaviors for the **iRobot Create 3** educational robot, written in Python with the `irobot‑edu‑sdk`.

---

## Project Overview
This repo contains two complementary behaviors:

| Script | Purpose | Core idea |
|--------|---------|-----------|
| **`MazeSolver.py`** | Explore and solve an _arbitrarily‑sized_ grid maze, then drive to a user‑defined goal | Dynamic flood‑fill: the robot updates a cost map in real time and always heads for the neighbour cell with the lowest cost|
| **`AutonomousDelivery.py`** | Point‑to‑point “last‑meter” delivery with reactive obstacle avoidance | Continually realigns to its target while using 7 IR proximity sensors to follow walls when needed|

Both scripts were battle‑tested on a corn‑maze–style course and achieved **≈99 % success over 100 runs** during our senior‑design demo.

---

## Hardware / Software Requirements
| Item | Notes |
|------|-------|
| iRobot **Create 3** | Firmware ≥ 0.4.0 recommended |
| Bluetooth‑capable host (laptop / RPi) | Tested on macOS 14 & Ubuntu 22.04 |
| Python 3.9 + | 3.11 works too |
| [`irobot‑edu‑sdk`](https://pypi.org/project/irobot-edu-sdk/) | Included in `requirements.txt` |
| Optional | 3 × 3 m maze tiles or any open space |

---

## Quick Start

```bash
# 1) Clone
git clone https://github.com/dylanjcurran/IRobot-Create3-Navigation.git
cd IRobot-Create3-Navigation

# 2) Install deps
python -m venv .venv && source .venv/bin/activate   # (optional)
pip install -r requirements.txt                     # contains irobot-edu-sdk

# 3) Power on your Create 3 and note its Bluetooth name
#    (the default is usually “iRobot‑[xxxx]” or your custom name)

# 4) Run either behavior
python MazeSolver.py          # default: start (0,0) ➜ goal (2,2)
python AutonomousDelivery.py  # default: drive to (0 cm, 120 cm)
