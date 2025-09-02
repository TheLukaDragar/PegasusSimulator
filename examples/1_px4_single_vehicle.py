#!/usr/bin/env python
"""
| File: 1_px4_single_vehicle.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to build an app that makes 
| use of the Pegasus API to run a simulation with a single vehicle, controlled 
| using the MAVLink control backend.
"""

# Imports to start Isaac Sim from this script
import argparse
import carb
import os
from isaacsim import SimulationApp

# Check if Isaac Lab is available for AppLauncher
try:
    from isaaclab.app import AppLauncher
    HAS_ISAACLAB = True
except ImportError:
    HAS_ISAACLAB = False
print(f"HAS_ISAACLAB: {HAS_ISAACLAB}")

# Parse command line arguments to support kit args
parser = argparse.ArgumentParser(description="PX4 Single Vehicle Simulation")

if HAS_ISAACLAB:
    # Use Isaac Lab's AppLauncher which properly supports kit_args
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app
else:
    # Fallback to direct SimulationApp usage (without kit_args support)
    parser.add_argument(
        "--headless", 
        action="store_true", 
        help="Run in headless mode"
    )
    args = parser.parse_args()
    config = {
        "headless": args.headless
    }
    # Start Isaac Sim's simulation environment
    # Note: this simulation app must be instantiated right after the SimulationApp 
    # import, otherwise the simulator will crash as this is the object that will 
    # load all the extensions and load the actual simulator.
    simulation_app = SimulationApp(config)

# -----------------------------------
# The actual script should start here
# -----------------------------------
# These imports must come AFTER SimulationApp is instantiated
import omni.timeline  # noqa: E402
from omni.isaac.core.world import World  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
import numpy as np  # noqa: E402

# Import the Pegasus API for simulating drones
from ssrd.pegasus.simulator.params import ROBOTS  # noqa: E402
from ssrd.pegasus.simulator.logic.backends.px4_mavlink_backend import (  # noqa: E402
    PX4MavlinkBackend, 
    PX4MavlinkBackendConfig
)
# Add camera support imports
from ssrd.pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera  # noqa: E402
#from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend  # noqa: E402
from ssrd.pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from ssrd.pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402


class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple 
    Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the 
        simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, i.e, the singleton that controls that is a one stop 
        # shop for setting up physics, spawning asset primitives, etc.
        # Update world settings to support 120 FPS camera
        world_settings = self.pg._world_settings.copy()
        world_settings["rendering_dt"] = 1.0 / 120.0  # 120 Hz rendering for 120 FPS camera
        self.pg._world = World(**world_settings)
        self.world = self.pg.world

        # Load custom environment from learning_nvidia
        self.pg.load_environment("/home/luka/learning_nvidia/flightdela2.usd")
       #self.pg.load_environment("/home/luka/learning_nvidia/fri2.usdc")

        # Create the vehicle
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor = MultirotorConfig()
        # Create the multirotor configuration
        # Set DDS environment variables
        os.environ["PX4_UXRCE_DDS_NS"] = "px4_0"  # DDS namespace
        os.environ["PX4_UXRCE_DDS_PORT"] = "8888"  # DDS port
        os.environ["ROS_DOMAIN_ID"] = "0"  # DDS domain ID

        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": self.pg.px4_path,
            # CHANGE this line to 'iris' if using PX4 version below v1.14
            "px4_vehicle_model": self.pg.px4_default_airframe,
            # Network configuration for QGroundControl access
            "connection_ip": "0.0.0.0",  # Bind to all interfaces instead of localhost
            "connection_baseport": 4560,
            "connection_type": "tcpin"
        })
        
        # Configure backends: PX4 for flight control + ROS2 for camera data
        # ros2_config = {
        #     "namespace": 'drone', 
        #     "pub_sensors": False,
        #     "pub_graphical_sensors": True,  # Enable camera data publishing
        #     "pub_state": True,
        #     "pub_tf": False,
        #     "sub_control": False,
        # }
        config_multirotor.backends = [
            PX4MavlinkBackend(mavlink_config),
            #ROS2Backend(vehicle_id=1, config=ros2_config)
        ]
        
        # Add camera sensor with Walksnail Avatar HD Kit V2 specifications
        camera_config = {
            "frequency": 120.0,                            # 120 FPS (Walksnail Avatar V2 capability)
            "resolution": (1920, 1080),                    # 1080P HD resolution (4:3 native sensor)
            "position": [0.30, 0.0, -0.05],               # 30cm forward, 5cm down from body
            "orientation": [0.0, 0.0, 180.0],               # Forward-facing (ZYX euler angles)
            "diagonal_fov": 160.0,                         # 160° wide-angle FOV (Walksnail Avatar V2)
            "intrinsics": np.array([[960.0, 0.0, 960.0],   # Focal length fx=960, principal point cx=960
                                   [0.0, 960.0, 540.0],    # Focal length fy=960, principal point cy=540
                                   [0.0, 0.0, 1.0]]),      # For 1920x1080 resolution with 160° FOV
            "distortion_coefficients": np.array([0.2, -0.1, 0.0, 0.0]),  # OpenCV fisheye distortion (k1, k2, k3, k4)
            "depth": True                                   # Enable depth data
        }
        config_multirotor.graphical_sensors = [MonocularCamera("front_camera", config=camera_config)]

        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.05],  # Spawn 50 meters in the air
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Reset the simulation environment so that all articulations (aka robots) 
        # are initialized
        self.world.reset()
        
        # Debug: Print camera configuration to verify settings are applied
        print("=== Walksnail Avatar HD Kit V2 Camera Configuration ===")
        print(f"Frequency: {camera_config['frequency']} FPS")
        print(f"Resolution: {camera_config['resolution']}")
        print(f"FOV: {camera_config['diagonal_fov']}°")
        print(f"Position: {camera_config['position']}")
        print("Intrinsics Matrix:")
        print(camera_config['intrinsics'])
        print(f"Distortion Coefficients: {camera_config['distortion_coefficients']}")
        print("=====================================================")

        # Camera data will be published to ROS2 topics under the /drone namespace:
        # - /drone/front_camera/rgb (RGB image data)
        # - /drone/front_camera/depth (Depth image data) 
        # - /drone/front_camera/camera_info (Camera calibration info)
        # Use 'ros2 topic list' to see all available topics

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def run(self):
        """
        Method that implements the application main loop, where the physics 
        steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():

    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()


if __name__ == "__main__":
    main()
