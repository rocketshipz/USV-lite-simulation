# USV Marine Plastic Collection Simulation

An Agent-Based Model (ABM) simulating a multi-agent Unmanned Surface Vehicle (USV) system designed to detect, collect, and transport floating marine plastic waste[cite: 2]. By leveraging autonomous agents and role specialization, the simulation demonstrates a highly efficient, continuous cleanup operation[cite: 2].

## Core System Mechanics

* **Role Specialization:** The workload is divided between distinct vehicle types to ensure continuous collection[cite: 2]. This allows the primary collection vehicle to remain in the field rather than repeatedly returning to base[cite: 2].
* **Dynamic Physics:** The travel speed of each USV dynamically scales and decreases based on the weight/volume of the trash it is currently carrying.
* **Inter-Agent Communication:** Agents share task status, storage capacities, and exact positional coordinates in real-time to coordinate hand-offs[cite: 2]. 

## Agent Profiles

### 1. Search-and-Collect (SC) USV
The primary forward-operating vehicle responsible for actively clearing the water surface[cite: 2].
* **Detection:** Utilizes simulated LiDAR and Radar sensor arrays to scan the environment and pinpoint floating plastic waste.
* **Operation:** Patrols the designated area autonomously until targets are identified, then moves toward the waste to collect it[cite: 2].
* **Capacity Protocol:** Operates with a strict physical storage limit. Once capacity is reached, it halts collection and transmits its exact coordinates to the Transport USV for offloading[cite: 2].

### 2. Transport (T) USV
The logistics and support vehicle that manages waste delivery to the base[cite: 2].
* **Coordination:** Maintains a communication link with the SC USV to monitor its capacity and current location[cite: 2].
* **Retrieval:** Navigates directly to the SC USV once alerted, retrieving the full payload of collected trash.
* **Disposal:** Transports the retrieved waste to a designated collection station or base, allowing the SC USV to immediately resume its patrol[cite: 2].

## Installation & Execution

Follow these steps to set up the environment and run the simulation locally.

**1. Create a virtual environment**
`python -m venv venv`

**2. Activate the virtual environment**
* On **Windows**:
  `venv\Scripts\activate`
* On **macOS / Linux**:
  `source venv/bin/activate`

**3. Install dependencies**
`pip install -r requirements.txt`

**4. Run the simulation**
`python main.py`
