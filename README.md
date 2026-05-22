# Cloth and Large-Step Particle System Simulation - 2IMV15 Project 1

In this repository, users can find a real-time cloth and particle simulation project developed for the TU/e course **2IMV15 Simulation in Computer Graphics** using **C++**, **OpenGL**, and **GLUT**.

The project implements multiple numerical integration schemes and interactive physical constraints for simulating deformable systems, such as cloth meshes and particle networks.

---

## Features

- Real-time cloth simulation
- Multiple numerical solvers:
  - Euler
  - Midpoint
  - Runge-Kutta 4 (i.e., RK4)
- Rod constraints and spring systems
- Mouse interaction with configurable stiffness and damping
- Wind force simulation
- Toggleable cloth fixation
- Frame dumping support
- Interactive runtime parameter controls

---

## Repository

Project repository can be accessed at:

https://github.com/KonstantinGeorgiev0/2IMV15-Project1.git

---

# Prerequisites

This project is built using **C++** and uses **OpenGL/GLUT** for rendering. It supports environments with a **C++** compiler and the necessary **OpenGL/GLUT** framework development libraries.


You will need:

- A C++ compiler supporting modern C++
- OpenGL development libraries
- GLUT/freeGLUT development libraries
- `make`

Supported operating systems:

- macOS
- Linux
- Windows

---

# Platform Notes

The repository is currently pushed in a **macOS-ready configuration**.

For setup instructions specific to your operating system, please refer to the provided platform-specific `.txt` files in the repository, i.e., compile_linux.txt, compile_mac.txt, or compile_win10.txt.

### Important

As indicated in the platform-specific '.txt' files mentioned above, for **Windows** (and potentially some Linux environments), make sure to:

- replace `GLUT` includes/usages with `GL`
- verify linker/library settings for OpenGL and GLUT/freeGLUT

This may be necessary depending on your local graphics/toolchain configuration.

---

# Compilation

From the project root directory/folder, open a shell window and type the following commands for build, one at a time:

```bash
make clean
make
```

The compilation process generates the executable binary inside the target binaries `bin/` folder.

---

# Running the Simulation

To launch the application, run the executable from the project root directory:

```bash
./bin/project1.exe
```

---

# Runtime Controls

The simulation initializes in a **paused configuration**. Use the keys mentioned below to control the scenes and interactions, respectively:


## General Controls

| Key | Action |
|---|---|
| `space` | Toggle simulation/construction mode |
| `q` | Quit application |
| `c` | Clear/reset simulation |
| `s` | Switch between scenes |

---

## Solver Controls

| Key | Action |
|---|---|
| `1` | Euler solver |
| `2` | Midpoint solver |
| `3` | RK4 solver |

---

## Simulation Parameters

| Key | Action |
|---|---|
| `p` / `o` | Increase/decrease timestep (`dt`) |
| `i` / `u` | Increase/decrease mouse spring stiffness |
| `k` / `j` | Increase/decrease mouse damping |

---

## Physics Toggles

| Key | Action |
|---|---|
| `r` | Toggle between sqrt and squared formula for `RodConstraint` |
| `w` | Toggle wind force |
| `f` | Toggle fixing top row of cloth |

---

## Debug / Utility

| Key | Action |
|---|---|
| `v` | Open parameter modification menu in console |
| `d` | Toggle frame dumping |

---

# Project Structure

```text
.
├── bin/                # Compiled executables
├── include/            # Header files
├── lib/                # External libraries and framework dependencies
├── obj/                # Intermediate object files generated during compilation
├── src/                # Source files
├── Makefile
├── compile_linux.txt
├── compile_mac.txt
├── compile_win10.txt
└── README.md
```

---

# Numerical Integration Methods

For context, the simulation supports three integration methods:

### Euler
Simple and fast, but less stable for stiff systems.

### Midpoint
Improved stability and accuracy over Euler.

### RK4 (i.e., Runge-Kutta 4)
Most accurate integrator implemented in the project, at higher computational cost.

---

# Interaction

The simulation allows direct mouse interaction with particles and cloth vertices through configurable spring forces and damping parameters.

This enables interactive deformation and testing of constraint behavior in real time.

---

# Known Notes

- Simulation stability depends heavily on timestep size (`dt`)
- RK4 provides the most stable behavior for complex cloth scenes
- Wind and rod constraints can significantly affect performance and stability

---

# Authors

The project has been developed for the TU/e course **2IMV15 Simulation in Computer Graphics**.

Contributors:
- Konstantin Georgiev
- Komal Jha
- Cristiana Cărbunaru

---

# License

This project is intended for educational purposes.
