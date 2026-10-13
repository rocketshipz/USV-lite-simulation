import mesa
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Simulation Parameters
AREA_SIZE = 100.0
NUM_PLASTICS = 15
SPAWN_MARGIN = 5.0  # Margin from edges to spawn plastics
SC_CAPACITY = 3
SPEED_SC = 1.5
SPEED_T = 2.0
BASE_STATION = np.array([10.0, 10.0])

# Trash Weight & Movement Parameters
PLASTIC_WEIGHT_MIN = 1.0  # kg
PLASTIC_WEIGHT_MAX = 5.0  # kg
WEIGHT_SLOWDOWN_FACTOR = 0.08  # Drag factor: higher weight -> slower movement
MIN_SPEED = 0.3  # Minimum drone speed floor to avoid stalling

# Lidar / Sensor Parameters
SENSOR_RADIUS = 25.0  # SC USV detection radius

# Search Patrol Waypoints for systematic area coverage when no trash is detected
PATROL_WAYPOINTS = [
    np.array([20.0, 20.0]),
    np.array([20.0, 80.0]),
    np.array([50.0, 80.0]),
    np.array([50.0, 20.0]),
    np.array([80.0, 20.0]),
    np.array([80.0, 80.0]),
]


class PlasticAgent(mesa.Agent):
    def __init__(self, model, weight=None):
        super().__init__(model)
        self.collected = False
        if weight is None:
            self.weight = round(float(np.random.uniform(PLASTIC_WEIGHT_MIN, PLASTIC_WEIGHT_MAX)), 1)
        else:
            self.weight = float(weight)


class SC_USV_Agent(mesa.Agent):
    def __init__(self, model):
        super().__init__(model)
        self.capacity = 0
        self.current_weight = 0.0
        self.state = "SEARCHING"  # SEARCHING, WAITING, COMPLETED
        self.target_plastic = None
        self.patrol_index = 0

    @property
    def current_speed(self):
        # Drone moves slower the more trash weight it carries
        return max(MIN_SPEED, SPEED_SC / (1.0 + WEIGHT_SLOWDOWN_FACTOR * self.current_weight))

    def move_towards(self, target_pos, speed):
        current_pos = np.array(self.pos)
        direction = target_pos - current_pos
        distance = np.linalg.norm(direction)
        if distance < speed:
            self.model.grid.move_agent(self, tuple(target_pos))
            return True
        else:
            new_pos = current_pos + (direction / distance) * speed
            self.model.grid.move_agent(self, tuple(new_pos))
            return False

    def step(self):
        if self.state == "SEARCHING":
            active_plastics = [
                a for a in self.model.agents 
                if isinstance(a, PlasticAgent) and not a.collected
            ]

            # If all plastics have been collected
            if not active_plastics:
                if self.capacity > 0:
                    self.state = "WAITING"
                else:
                    self.state = "COMPLETED"
                return

            # Detect plastics using Lidar / Radius sensor (only within SENSOR_RADIUS)
            detected_plastics = [
                p for p in active_plastics
                if np.linalg.norm(np.array(p.pos) - np.array(self.pos)) <= SENSOR_RADIUS
            ]

            # Reset target if it was already collected
            if self.target_plastic is not None and self.target_plastic.collected:
                self.target_plastic = None

            # If no target currently locked, lock onto the closest detected trash in sensor range
            if self.target_plastic is None and detected_plastics:
                distances = [
                    np.linalg.norm(np.array(p.pos) - np.array(self.pos))
                    for p in detected_plastics
                ]
                self.target_plastic = detected_plastics[np.argmin(distances)]

            if self.target_plastic is not None:
                reached = self.move_towards(np.array(self.target_plastic.pos), self.current_speed)
                if reached:
                    self.target_plastic.collected = True
                    self.model.grid.remove_agent(self.target_plastic)
                    self.capacity += 1
                    self.current_weight += self.target_plastic.weight
                    self.target_plastic = None
                    if self.capacity >= SC_CAPACITY:
                        self.state = "WAITING"
            else:
                # No trash detected within lidar radius -> patrol the area to find trash
                target_wp = PATROL_WAYPOINTS[self.patrol_index]
                reached = self.move_towards(target_wp, self.current_speed)
                if reached:
                    self.patrol_index = (self.patrol_index + 1) % len(PATROL_WAYPOINTS)


class T_USV_Agent(mesa.Agent):
    def __init__(self, model):
        super().__init__(model)
        self.state = "IDLE"  # IDLE, RETRIEVING, RETURNING
        self.carried_weight = 0.0

    @property
    def current_speed(self):
        # Transport drone also slows down when carrying collected trash back to base
        return max(MIN_SPEED, SPEED_T / (1.0 + WEIGHT_SLOWDOWN_FACTOR * self.carried_weight))

    def move_towards(self, target_pos, speed):
        current_pos = np.array(self.pos)
        direction = target_pos - current_pos
        distance = np.linalg.norm(direction)
        if distance < speed:
            self.model.grid.move_agent(self, tuple(target_pos))
            return True
        else:
            new_pos = current_pos + (direction / distance) * speed
            self.model.grid.move_agent(self, tuple(new_pos))
            return False

    def step(self):
        sc_agent = self.model.sc_agent
        if sc_agent.state == "WAITING" and self.state == "IDLE":
            self.state = "RETRIEVING"

        if self.state == "RETRIEVING":
            reached = self.move_towards(np.array(sc_agent.pos), self.current_speed)
            if reached:
                # Transfer collected trash load from SC drone to T drone
                self.carried_weight = sc_agent.current_weight
                sc_agent.current_weight = 0.0
                sc_agent.capacity = 0

                active_plastics = [
                    a for a in self.model.agents 
                    if isinstance(a, PlasticAgent) and not a.collected
                ]
                if active_plastics:
                    sc_agent.state = "SEARCHING"
                else:
                    sc_agent.state = "COMPLETED"

                self.state = "RETURNING"

        elif self.state == "RETURNING":
            reached = self.move_towards(BASE_STATION, self.current_speed)
            if reached:
                # Unload trash at base station
                self.carried_weight = 0.0
                self.state = "IDLE"


class MarineCollectionModel(mesa.Model):
    def __init__(self, num_plastics=NUM_PLASTICS):
        super().__init__()
        self.grid = mesa.space.ContinuousSpace(AREA_SIZE, AREA_SIZE, False)

        # Add SC-USV
        self.sc_agent = SC_USV_Agent(self)
        self.grid.place_agent(self.sc_agent, (50.0, 50.0))

        # Add T-USV
        self.t_agent = T_USV_Agent(self)
        self.grid.place_agent(self.t_agent, tuple(BASE_STATION))

        # Add Plastics with random positions and weights
        for _ in range(num_plastics):
            p = PlasticAgent(self)
            pos = tuple(np.random.uniform(SPAWN_MARGIN, AREA_SIZE - SPAWN_MARGIN, size=2))
            self.grid.place_agent(p, pos)

    def step(self):
        self.agents.shuffle_do("step")


# Start the simulation
# Instantiate Model
model = MarineCollectionModel()

# Prepare visualization data capture
history = []
MAX_FRAMES = 800

def capture_state():
    return {
        "sc_pos": np.copy(model.sc_agent.pos),
        "t_pos": np.copy(model.t_agent.pos),
        "sc_state": model.sc_agent.state,
        "t_state": model.t_agent.state,
        "sc_weight": model.sc_agent.current_weight,
        "sc_speed": model.sc_agent.current_speed,
        "t_weight": model.t_agent.carried_weight,
        "t_speed": model.t_agent.current_speed,
        "plastics": [
            {"pos": np.copy(a.pos), "weight": a.weight}
            for a in model.agents
            if isinstance(a, PlasticAgent) and not a.collected
        ]
    }

# Record initial frame
history.append(capture_state())

for frame in range(MAX_FRAMES):
    model.step()
    state = capture_state()
    history.append(state)

    # Check if all plastics are collected and T-USV has finished offloading at base
    if len(state["plastics"]) == 0 and state["t_state"] == "IDLE" and model.sc_agent.capacity == 0:
        # Linger for a few extra frames to show final completed state
        for _ in range(15):
            history.append(state)
        break

# Matplotlib Animation Setup
fig, ax = plt.subplots(figsize=(7, 7))

def update(frame):
    ax.clear()
    ax.set_xlim(0, AREA_SIZE)
    ax.set_ylim(0, AREA_SIZE)

    state_info = history[frame]
    ax.set_title(
        f"Mesa USV Marine Collection (Frame {frame}/{len(history)-1})\n"
        f"SC USV: [{state_info['sc_state']}] Load: {state_info['sc_weight']:.1f} kg | Speed: {state_info['sc_speed']:.2f}\n"
        f"T USV:  [{state_info['t_state']}] Load: {state_info['t_weight']:.1f} kg | Speed: {state_info['t_speed']:.2f}",
        fontsize=9
    )

    # Plot Base Station
    ax.plot(BASE_STATION[0], BASE_STATION[1], 'ks', markersize=10, label="Base Station")

    # Draw Lidar Sensor Radius Circle around SC USV
    sc_pos = state_info["sc_pos"]
    sensor_circle = plt.Circle(
        (sc_pos[0], sc_pos[1]),
        SENSOR_RADIUS,
        edgecolor='crimson',
        facecolor='mistyrose',
        linestyle='--',
        alpha=0.35,
        linewidth=1.2,
        label=f"SC Lidar (r={int(SENSOR_RADIUS)})"
    )
    ax.add_patch(sensor_circle)

    # Plot Plastics
    plastics = state_info["plastics"]
    if plastics:
        px = [p["pos"][0] for p in plastics]
        py = [p["pos"][1] for p in plastics]
        sizes = [25 + p["weight"] * 12 for p in plastics]
        ax.scatter(px, py, c='blue', s=sizes, marker='o', alpha=0.8, label=f"Plastic Waste ({len(plastics)} left)")
        for p in plastics:
            ax.text(p["pos"][0] + 1.2, p["pos"][1] + 1.2, f"{p['weight']:.1f}kg", fontsize=7, color='navy')
    else:
        ax.plot([], [], 'bo', label="Plastic Waste (Cleared!)")

    # Plot Agents
    ax.plot(sc_pos[0], sc_pos[1], 'ro', markersize=9, label=f"SC USV ({state_info['sc_weight']:.1f}kg)")
    ax.plot(state_info["t_pos"][0], state_info["t_pos"][1], 'go', markersize=9, label=f"T USV ({state_info['t_weight']:.1f}kg)")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, linestyle=':', alpha=0.4)
    ax.legend(loc="upper right", fontsize=7.5, framealpha=0.85)

ani = FuncAnimation(fig, update, frames=len(history), interval=35, repeat=False)
plt.show()