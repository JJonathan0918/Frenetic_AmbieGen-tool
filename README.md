# 🚗 Frenetic_AmbieGen: Genetic Algorithm-Based Road Topology Generator

This is a **genetic algorithm-based road topology generator**, built on the AmbieGEN framework and Frenetic's road representation approach. The goal is to produce diverse and realistic road layouts and evaluate their effectiveness using both preliminary and simulator-based control models.

---

## 🧠 Overview

- 🚧 **Road Generation Framework**: Combines **AmbieGen** (a procedural generation framework) with **Frenetic** (Frenet-based road representation).
- 🧬 **Optimization Method**: Road samples are generated and optimized using a **genetic algorithm** (GA).
- 📈 **Evaluation Workflow**:
  - **Stage 1**: Preliminary validation using a **kinematic vehicle model** and **lane-following controller**.
  - **Stage 2**: Realistic simulation using **BeamNG.tech** with **BeamNG.AI** and the **Dave-2 neural controller**.

---

## 📁 Project Structure

```
.
├── frenetic_ambiegen_preliminary/ # Preliminary road generation and evaluation using a kinematic model and lane controller
├── frenetic_ambiegen_simulation/ # Simulation evaluation scripts using BeamNG.tech with Dave-2 and BeamNG.AI
├── results/ # Evaluation results including plots, logs, and images
├── compare.py # Script to compare results between Frenetic_AmbieGen and the baseline
├── config.py # Configuration file for GA settings, environment parameters, and simulation settings
├── demo.py # Entry point to generate and evaluate a single road sample
├── optimize.py # Genetic algorithm logic for road generation and optimization
├── requirements.txt # Python dependencies
└── LICENSE # MIT License
```
## 🚀 Quick Start

### Competition Background

The [SBST Workshop](https://sbft24.github.io/) offers a challenge for software testers who want to work with self-driving cars in the context of the usual [tool competition](https://sbft24.github.io/tools/).
[![Video by BeamNg GmbH](https://user-images.githubusercontent.com/93574498/207164554-3f3d9478-3970-4c08-b1e3-2b656313ae33.webm)]([https://github.com/BeamNG/BeamNGpy/raw/master/media/steering.gif](https://user-images.githubusercontent.com/93574498/207164554-3f3d9478-3970-4c08-b1e3-2b656313ae33.webm))

>Note: BeamNG GmbH, the company developing the simulator, kindly offers it for free for researcher purposes upon registration (see [Installation](documentation/INSTALL.md)).

### 🔧 Installation

```bash
git clone https://github.com/JJonathan0918/frenetic_ambiegen-tool.git
cd frenetic_ambiegen

# (Optional) Create virtual environment
python -m venv venv
source venv/bin/activate  # or venv\Scripts\activate on Windows

# Install dependencies
pip install -r requirements.txt
```

## 🧪 Usage

### Run preliminary evaluation
```bash
python optimize.py --problem vehicle --algo random --runs 1
```
The algorithm part can use ga, random, nsga2.

### Run simulation evaluation

```bash
python competition.py --visualize-tests --time-budget 1800 --executor beamng --map-size 200 --module-name frenetic_ambiegen_formal.ambiegen_generator --class-name AmbieGenTestGenerator --beamng-home E:\BeamNG.tech.v0.26.2.0\BeamNG.tech.v0.26.2.0 --beamng-user E:\BeamNG.tech.v0.26.2.0_userpath
```

## 📜 License

This project includes code from [cps-tool-competition], which is licensed under the GPL v3 License.
