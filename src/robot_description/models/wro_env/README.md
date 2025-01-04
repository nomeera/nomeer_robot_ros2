# WRO Future Competition Simulation Environment

This repository contains the simulation environment for the WRO (World Robot Olympiad) Future competition. The environment is modeled with actual size and dimensions, making it an ideal setup for testing robot performance and strategies in a simulated space. The simulation is created using **Gazebo Harmonic** and **ROS 2 Humble**.

## Features

- **Accurate Competition Layout**: The environment model (`.sdf` file) represents the real-world competition space with exact measurements.
- **Real-Time Simulation**: Test your robotics solutions in real-time using ROS 2 Humble with the Gazebo Harmonic simulator.
- **Pre-configured Setup**: A plug-and-play environment that allows users to quickly get started with testing and simulation.

## Requirements

- **ROS 2 Humble**: Follow the instructions [here](https://docs.ros.org/en/humble/Installation.html) to install ROS 2 Humble.
- **Gazebo Harmonic**: You can install Gazebo Harmonic by following the official [installation guide](https://gazebosim.org/docs/harmonic/install_ubuntu/).
- **SDF Model**: The environment is defined using an SDF model file, compatible with Gazebo Harmonic.

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/khaledgabr77/WRO-Future-Simulation.git
   ```

2. Navigate to the directory:

   ```bash
   cd WRO-Future-Simulation
   ```

3. Launch the simulation environment with Gazebo:

   ```bash
   gz sim model.sdf
   ```

![alt text](</images/Screenshot from 2024-10-14 22-00-24.png>)

## Usage

1. Open Gazebo Harmonic, load the WRO Future environment, and connect it to your ROS 2 Humble setup.
2. Utilize the provided topics and services to control and monitor your robot within the environment.
3. The environment is designed for testing robotics solutions under real-world conditions found in the WRO competition.

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository.
2. Create a new branch (`git checkout -b feature/YourFeature`).
3. Commit your changes (`git commit -m 'Add some feature'`).
4. Push to the branch (`git push origin feature/YourFeature`).
5. Open a Pull Request.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

