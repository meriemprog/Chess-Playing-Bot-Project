YOUPI Chess Arm ♟️🦾

An autonomous chess-playing robot built on a repurposed Bras-Yuppi (YOUPI) commercial robotic arm kit. The system combines a custom-designed gripper, computer vision for board and piece detection, inverse kinematics for motion control, and AI-based move decision-making — bringing perception, planning, and control together in a single pipeline.

The robot watches the physical board, decides on (or receives) a move, and physically executes it by picking up and placing chess pieces.


✨ Features


Autonomous pick & place — full pick sequence (rise → travel → descend → grip → rise → return) for moving chess pieces on the board
Custom inverse kinematics solver — closed-form 3-DOF IK with elbow-up / elbow-down solution selection and joint-limit checking
Waypoint-based homing — every move is logged so the arm can retrace its exact path back to a saved home position
Custom gripper — designed and built in-house to reliably grasp chess pieces of varying shapes/sizes
Computer vision — board and piece detection (state recognition of the physical board)
AI move engine — move decision-making for autonomous play
Stepper motor control — synchronized multi-axis motion using a Bresenham-style interpolation algorithm



🏗️ System Overview

┌───────────────┐     ┌───────────────┐     ┌────────────────────┐     ┌───────────────┐
│  Vision System │ --> │   AI / Move   │ --> │  Inverse Kinematics │ --> │  Motor Control │
│ (board+piece   │     │   Engine      │     │  (angle solver)     │     │  (stepper      │
│  detection)    │     │               │     │                     │     │   drivers)     │
└───────────────┘     └───────────────┘     └────────────────────┘     └───────────────┘
                                                                                │
                                                                                ▼
                                                                        ┌───────────────┐
                                                                        │  YOUPI Arm +   │
                                                                        │  Custom Gripper│
                                                                        └───────────────┘


The vision system captures the current state of the physical board.
The AI engine determines the next move.
The move's target (X, Y) coordinates are converted to joint angles via inverse kinematics.
The motion controller drives the three stepper motors in a synchronized fashion to execute the move, including a full pick-and-place sequence.



🔧 Hardware


Base kit: Bras-Yuppi (YOUPI) commercial robotic arm
End effector: Custom in-house gripper, designed for chess piece handling
Microcontroller: Raspberry Pi Pico (MicroPython)
Actuation: 3× stepper motors (CLK/step + CW/direction driver interface)


MotorFunctionSTEP pinDIR pinSteps / revolutionM1Base rotation (θ1)GP10GP115900 (≈180°)M2Shoulder (θ2)GP13GP123150 (≈90°)M3Elbow (θ3)GP14GP153250 (≈90°)

Arm geometry

ParameterValueUpper arm length (L1)162 mmForearm length (L2)162 mmShoulder height80 mmMax reach324 mm

CAD files for the arm/gripper assembly are included (Assem1.SLDASM, SolidWorks).


🧠 Inverse Kinematics

The IK solver (solve_ik) takes a target (x, y, z) and returns joint angles (t1, t2, t3):


θ1 (base): computed directly via atan2(y, x).
θ2 / θ3 (shoulder/elbow): solved using the law of cosines on the shoulder-to-target distance, giving two candidate configurations — elbow-up and elbow-down.
Each candidate is checked against joint limits before being accepted.
Mode-dependent solution priority:

mode="move" (picking/placing) → tries elbow-down first (natural downward reach onto the board)
mode="home" (travel/safe moves) → tries elbow-up first (keeps the arm clear of obstacles)





If a target is unreachable (out of range, or violates joint limits in both configurations), the solver returns None and the move is safely aborted.

Motion & Homing


move_to_angles() drives all three motors simultaneously using a Bresenham-like interpolation so they start and finish together, regardless of how many steps each one needs.
Every executed move is pushed onto a waypoint log. return_home() replays this log in reverse to retrace the arm's exact path back to the saved home position — important since the arm has no absolute position feedback (open-loop stepper control).
The log is capped (MAX_WAYPOINTS = 100) to protect the Pico's limited RAM.


Pick Sequence

pick_sequence(x, y) performs a full pick-and-place cycle:


Rise to safe height at the current position
Move to safe height above the target
Descend to pick height (elbow-down preferred)
Grip (hold delay)
Rise back to safe height
Return home (retrace waypoints)
Move to a parking position (base rotated +30°)



🚀 Getting Started

Requirements


Raspberry Pi Pico (or compatible board) running MicroPython
machine, math, and utime modules (built into MicroPython)
Stepper drivers wired per the pin table above


Running


Flash MicroPython onto the Pico.
Copy the control script onto the board (e.g. as main.py).
Power on — the arm homes automatically to (80, 0, SAFE_Z).
Interact over the serial console:


80 0        # pick sequence targeting X=80, Y=0
home        # retrace path and return to home position
pos         # print current X/Y/Z position


📂 Repository Contents

FileDescriptionAssem1.SLDASMSolidWorks assembly of the arm + custom gripperInverse-kinematics-*.pyMicroPython firmware: IK solver, motion control, pick sequence


Vision and AI move-decision modules are part of the broader project and may live in separate files/repos — update this section with links as they're added.




🗺️ Roadmap


 Vision-based board/piece state recognition integration
 AI move engine integration (chess engine interface)
 Closed-loop position feedback / calibration routine
 Automated opponent move confirmation (illegal move detection)



https://github.com/user-attachments/assets/d14841c2-93cd-44d1-9f84-836eece63526



https://github.com/user-attachments/assets/3cf8b165-4b86-42b6-9791-ea7a03b91e5b



https://github.com/user-attachments/assets/12a544c9-9a42-4aeb-83b0-646884c86f1e

