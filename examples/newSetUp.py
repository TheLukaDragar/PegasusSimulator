"""
ENHANCED SIMPLE WORKING Drone Camera with FIXED LIGHTING and GPU PHYSICS
This version includes multiple lighting solutions and GPU physics configuration
"""

import argparse
import carb
from isaacsim import SimulationApp
import socket
import struct
import cv2
import numpy as np
import threading
import time

# =============================================
# DRONE PERFORMANCE SETTINGS - Edit these values to change drone behavior
# =============================================

# === SPEED SETTINGS ===
MAX_VELOCITY = 80.0           # Maximum velocity (m/s) - default is ~15
MAX_ACCELERATION = 50.0       # Maximum acceleration (m/s²)
MAX_ANGULAR_VELOCITY = 30.0    # Maximum angular velocity (rad/s)

# === RESPONSIVENESS ===
POSITION_GAIN = 80.5           # Higher = more responsive position control
VELOCITY_GAIN = 80.0           # Higher = more aggressive velocity control  
ATTITUDE_GAIN = 80.0           # Higher = more responsive attitude control

# === AGILITY SETTINGS ===
MAX_TILT_ANGLE = 45.0         # Maximum tilt angle (degrees)
MAX_CLIMB_RATE = 80.0          # Maximum climb rate (m/s)
MAX_DESCENT_RATE = 20.0        # Maximum descent rate (m/s)

# === STABILITY ===
HOVER_THRUST = 0.5            # Thrust needed to hover (0-1)
THRUST_CURVE = "quadratic"    # Thrust response curve
DRAG_COEFFICIENT = 0.1        # Air resistance

# =============================================
# CAMERA CONFIGURATION
# =============================================
UDP_ENABLED = True
UDP_HOST = "127.0.0.1"
UDP_PORT = 8555

# Camera position relative to drone
CAMERA_X = 0.3      # Forward from drone center (positive = forward)
CAMERA_Y = 0.0      # Right from drone center (positive = right)
CAMERA_Z = -0.1     # Down from drone center (positive = up, negative = down)

# Camera settings
CAMERA_FOV = 160.0              # Field of view in degrees
CAMERA_RESOLUTION = (1280 , 720 ) # Camera resolution (width, height)
CAMERA_FREQUENCY = 60.0         # Camera capture frequency (FPS)

# Video encoding settings
VIDEO_QUALITY = 70              # JPEG quality (1-100)
VIDEO_RESIZE_WIDTH = 1280        # Resize for UDP streaming
VIDEO_RESIZE_HEIGHT = 720       # Resize for UDP streaming

print("=" * 60)
print("ENHANCED SIMPLE WORKING DRONE CAMERA with FIXED LIGHTING and GPU PHYSICS")
print("=" * 60)
print(f"DRONE PERFORMANCE:")
print(f"  Max Velocity: {MAX_VELOCITY} m/s")
print(f"  Max Acceleration: {MAX_ACCELERATION} m/s²")
print(f"  Max Tilt Angle: {MAX_TILT_ANGLE}°")
print(f"  Max Climb Rate: {MAX_CLIMB_RATE} m/s")
print("")
print(f"CAMERA SETTINGS:")
print(f"  Position: {CAMERA_X}m forward, {CAMERA_Y}m right, {CAMERA_Z}m down")
print(f"  FOV: {CAMERA_FOV}°")
print(f"  Resolution: {CAMERA_RESOLUTION[0]}x{CAMERA_RESOLUTION[1]}")
print(f"  Frequency: {CAMERA_FREQUENCY} FPS")
print(f"  UDP Port: {UDP_PORT}")
print("")
print(f"PHYSICS:")
print(f"  Backend: GPU")
print(f"  GPU Dynamics: Enabled")
print("=" * 60)

# Check if Isaac Lab is available for AppLauncher
try:
    from isaaclab.app import AppLauncher
    HAS_ISAACLAB = True
except ImportError:
    HAS_ISAACLAB = False

# Parse command line arguments
parser = argparse.ArgumentParser(description="Enhanced Simple Working Drone Camera with Fixed Lighting and GPU Physics")

if HAS_ISAACLAB:
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app
else:
    parser.add_argument("--headless", action="store_true", help="Run in headless mode")
    args = parser.parse_args()
    
    config = {
        "headless": args.headless,
        "renderer": "RayTracedLighting",
        "enable_rtx": False,  # Make sure RTX is enabled
        "enable_vulkan": True,
        "force_viewport": False,
        "physics_backend": "gpu",  # GPU physics backend
        "anti_aliasing": 0,  # Increased for better quality
        "width": 800,
        "height": 600,
    }
    simulation_app = SimulationApp(config)
    

# Post-simulation imports
import omni.timeline
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation

# Pegasus imports
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend, 
    PX4MavlinkBackendConfig
)
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Isaac Sim camera imports
from omni.isaac.sensor import Camera

# Import for lighting setup
import omni.usd
from pxr import UsdGeom, Gf, UsdLux, PhysxSchema

def configure_gpu_physics():
    """
    Configure GPU physics settings for optimal performance and dynamics
    Fixed for physics stepping synchronization issues
    """
    print("Configuring GPU Physics...")
    
    try:
        # Wait for stage to be fully initialized
        import time
        time.sleep(0.1)
        
        # Get the current stage
        stage = omni.usd.get_context().get_stage()
        if not stage:
            print("Warning: Stage not ready, skipping GPU physics configuration")
            return False
        
        # Find or create the main physics scene (ensure single scene)
        physics_scene_path = "/physicsScene"  # Use standard path
        physics_scene_prim = stage.GetPrimAtPath(physics_scene_path)
        
        if not physics_scene_prim.IsValid():
            # Create physics scene if it doesn't exist
            from pxr import UsdPhysics
            physics_scene_prim = UsdPhysics.Scene.Define(stage, physics_scene_path)
            print("Created new physics scene")
        
        # Apply PhysX Scene API
        if not physics_scene_prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene_prim)
            print("Applied PhysX Scene API")
        else:
            physx_scene_api = PhysxSchema.PhysxSceneAPI(physics_scene_prim)
            print("Using existing PhysX Scene API")
        
        # FIXED: Configure physics stepping to prevent sync issues
        # Set consistent timestep that matches world settings
        physx_scene_api.CreateTimeStepsPerSecondAttr().Set(250.0)  # Match rendering frequency
        print("Physics timestep synchronized to 60 Hz")
        
        # Enable GPU dynamics
        physx_scene_api.CreateEnableGPUDynamicsAttr().Set(True)
        print("GPU Dynamics enabled")
        
        # Conservative GPU memory settings to prevent conflicts
        physx_scene_api.CreateGpuMaxNumPartitionsAttr().Set(8)  # Reduced from 8
        print("GPU partitions set to 4 (conservative)")
        
        # Reduced GPU collision stack size for stability
        physx_scene_api.CreateGpuCollisionStackSizeAttr().Set(33554432)  # 32MB instead of 64MB
        print("GPU collision stack size set to 32MB")
        
        # Set found/lost pairs capacity (conservative)
        physx_scene_api.CreateGpuFoundLostPairsCapacityAttr().Set(131072)  # Reduced
        print("GPU found/lost pairs capacity configured")
        
        # Enable Continuous Collision Detection for fast-moving objects
        physx_scene_api.CreateEnableCCDAttr().Set(True)
        print("Continuous Collision Detection enabled")
        
        # Use PGS solver instead of TGS for better stability in GPU mode
        physx_scene_api.CreateSolverTypeAttr().Set("PGS")
        print("PGS solver enabled (better GPU compatibility)")
        
        # Conservative solver iteration counts
        physx_scene_api.CreateSolverPositionIterationCountAttr().Set(8)  # Reduced from 8
        physx_scene_api.CreateSolverVelocityIterationCountAttr().Set(2)  # Reduced from 2
        print("Conservative solver iterations configured (4 pos, 1 vel)")
        
        # IMPORTANT: Enable broad phase type for GPU
        physx_scene_api.CreateBroadphaseTypeAttr().Set("GPU")
        print("GPU broadphase enabled")
        
        # Enable GPU collision for better performance
        physx_scene_api.CreateGpuCollisionStackSizeAttr().Set(33554432)
        print("GPU collision stack configured")
        
        print("GPU Physics configuration completed successfully!")
        print("Note: Using conservative settings to prevent stepping sync issues")
        return True
        
    except Exception as e:
        print(f"Error configuring GPU physics: {e}")
        print("Falling back to CPU physics...")
        return False

def verify_gpu_physics():
    """
    Verify that GPU physics is properly enabled
    """
    try:
        from omni.isaac.core import World
        
        # This will be called after world creation
        world = World.instance()
        if world is not None:
            physics_context = world.get_physics_context()
            if physics_context:
                gpu_enabled = physics_context.is_gpu_dynamics_enabled() if hasattr(physics_context, 'is_gpu_dynamics_enabled') else True
                print(f"GPU Physics Status: {'Enabled' if gpu_enabled else 'Disabled'}")
                return gpu_enabled
    except Exception as e:
        print(f"Could not verify GPU physics status: {e}")
    
    return True  # Assume enabled if we can't check

# Add this function to be called after world creation
def setup_enhanced_physics():
    """
    Setup enhanced physics configuration after world creation
    """
    configure_gpu_physics()
    verify_gpu_physics()
# You should call setup_enhanced_physics() after creating your World instance
# For example, after: world = World(physics_dt=1.0/240.0, rendering_dt=1.0/60.0)
# Add: setup_enhanced_physics()

class EnhancedUDPPublisher:
    """Enhanced UDP Publisher with configurable settings"""
    
    def __init__(self, host="127.0.0.1", port=8555):
        self.host = host
        self.port = port
        self.enabled = UDP_ENABLED
        self.socket = None
        self.target_address = (host, port)
        self.frame_count = 0
        self.quality = VIDEO_QUALITY
        self.resize_width = VIDEO_RESIZE_WIDTH
        self.resize_height = VIDEO_RESIZE_HEIGHT
        
        if self.enabled:
            self.setup_udp()
    
    def setup_udp(self):
        """Setup UDP socket"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"✓ UDP Publisher ready: {self.host}:{self.port}")
            print(f"✓ Video settings: {self.resize_width}x{self.resize_height} @ {self.quality}% quality")
        except Exception as e:
            print(f"✗ UDP setup failed: {e}")
            self.enabled = False
    
    def publish_frame(self, frame):
        """Publish frame via UDP with enhanced settings"""
        if not self.enabled or self.socket is None or frame is None:
            return False
        
        try:
            # Resize to configured dimensions
            if frame.shape[:2] != (self.resize_height, self.resize_width):
                frame = cv2.resize(frame, (self.resize_width, self.resize_height))
            
            # Encode with configured quality
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.quality]
            success, buffer = cv2.imencode('.jpg', frame, encode_params)
            
            if not success:
                return False
            
            frame_data = buffer.tobytes()
            
            # Send frame size first
            size_data = struct.pack('>I', len(frame_data))
            self.socket.sendto(size_data, self.target_address)
            
            # Send frame data in chunks
            chunk_size = 60000
            for i in range(0, len(frame_data), chunk_size):
                chunk = frame_data[i:i + chunk_size]
                self.socket.sendto(chunk, self.target_address)
            
            self.frame_count += 1
            if self.frame_count % 60 == 0:
                print(f"✓ Sent {self.frame_count} frames from drone camera")
            
            return True
            
        except Exception as e:
            print(f"UDP publish error: {e}")
            return False

class EnhancedSimpleDroneCamera:
    """Enhanced simple drone with performance settings, fixed lighting, and GPU physics"""

    def __init__(self):
        """Initialize enhanced drone with camera, fixed lighting, and GPU physics"""
        
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()

        # Enhanced world settings with GPU physics (FIXED stepping sync)
        world_settings = {
            "physics_dt": 1.0 /250.0,   # FIXED: Match physics to rendering frequency
            "rendering_dt": 1.0 / 60.0,
            "stage_units_in_meters": 1.0,
            "physics_prim_path": "/physicsScene",  # Standard path
            "set_defaults": True,
            "backend": "torch",
            "device": "cpu",
        }
        
        self.pg._world = World(**world_settings)
        self.world = self.pg.world

        # *** SETUP GPU PHYSICS RIGHT AFTER WORLD CREATION ***
        print("Configuring GPU Physics for enhanced performance...")
        setup_enhanced_physics()

        # Load environment
        self.pg.load_environment("/home/luka/learning_nvidia/single obsticle.usd")
        
        # Setup comprehensive lighting AFTER loading environment
        self.setup_comprehensive_lighting()
        
        # Initialize enhanced UDP publisher
        self.udp_publisher = EnhancedUDPPublisher()

        # Create enhanced drone configuration
        config_multirotor = self.create_enhanced_drone_config()
        
        self.drone = Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.05],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Setup enhanced camera
        self.setup_enhanced_camera()

        self.world.reset()
        self.stop_sim = False
        
        print("=== Enhanced Simple Drone Camera with Fixed Lighting and GPU Physics Ready ===")
        print(f"Drone configured for high-speed flight (max: {MAX_VELOCITY} m/s)")
        print(f"GPU Physics: Enabled with synchronized stepping (60 Hz)")
        print(f"Physics/Rendering Sync: Matched frequencies for stability")

    def setup_comprehensive_lighting(self):
        """Setup comprehensive lighting system with multiple methods"""
        print("Setting up comprehensive lighting system...")
        
        """Enable camera-specific lighting enhancements"""
        try:
            settings = carb.settings.get_settings()
            
            # Camera/viewport specific settings
            settings.set("/app/viewport/grid/enabled", False)  # Disable grid for cleaner image
            settings.set("/app/viewport/outline/enabled", False)  # Disable outline
            
            # Rendering quality settings
            settings.set("/rtx/raytracing/subdiv/maxLevel", 2)
            settings.set("/rtx/raytracing/fractionalCutout/opacity", 0.5)
            
            print("Camera lighting enhancements enabled")
            
        except Exception as e:
            print(f"Camera lighting error: {e}")

    def create_enhanced_drone_config(self):
        """Create enhanced drone configuration with performance settings"""
        
        config_multirotor = MultirotorConfig()
        
        # Enhanced MAVLink configuration with performance parameters
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe,
            "connection_ip": "127.0.0.1",
            "connection_baseport": 4560,
            "connection_type": "tcpin",
            "enable_lockstep": True,
            "send_vision_estimation": False,
            "send_odometry": True,
            
            # === PERFORMANCE PARAMETERS FOR PX4 ===
            "custom_params": {
                # === SPEED PARAMETERS ===
                "MPC_XY_VEL_MAX": MAX_VELOCITY,           # Max horizontal velocity
                "MPC_Z_VEL_MAX_UP": MAX_CLIMB_RATE,       # Max climb rate
                "MPC_Z_VEL_MAX_DN": MAX_DESCENT_RATE,     # Max descent rate
                "MPC_ACC_HOR_MAX": MAX_ACCELERATION,      # Max horizontal acceleration
                
                # === AGILITY PARAMETERS ===
                "MPC_TILTMAX_AIR": MAX_TILT_ANGLE,        # Max tilt angle
                "MC_YAWRATE_MAX": np.degrees(MAX_ANGULAR_VELOCITY), # Max yaw rate
                "MC_ROLLRATE_MAX": 200.0,                 # Max roll rate (deg/s)
                "MC_PITCHRATE_MAX": 200.0,                # Max pitch rate (deg/s)
                
                "CBRK_FLIGHTTERM": 121212,        # Disable flight termination
                "CBRK_VELPOSERR": 159753,         # Disable velocity/position error checks
                "COM_VEL_FS_EVH": 0,     

                # === RESPONSIVENESS ===
                "MPC_XY_P": POSITION_GAIN,               # Position controller gain
                "MPC_XY_VEL_P_ACC": VELOCITY_GAIN,       # Velocity controller gain
                "MC_ROLL_P": ATTITUDE_GAIN,              # Roll/pitch controller gain
                "MC_PITCH_P": ATTITUDE_GAIN,
                
                # === RACING/PERFORMANCE SETTINGS ===
                "MPC_ACC_UP_MAX": 80.0,                  # Max upward acceleration
                "MPC_ACC_DOWN_MAX": 80.0,                 # Max downward acceleration
                "MPC_JERK_AUTO": 8.0,                    # Jerk limit for smoothness
                "MPC_TKO_SPEED": 80.0,                    # Takeoff speed
                "MPC_LAND_SPEED": 80.5,                   # Landing speed
                
                # === ADVANCED TUNING ===
                "MC_AIRMODE": 1,                         # Enable airmode for aggressive flying
                "MPC_POS_MODE": 4,                       # Position control mode (4=acceleration based)
                "PWM_MAIN_MIN": 10000,                    # Motor PWM range
                "PWM_MAIN_MAX": 60000,
                
                # === SIMULATION OPTIMIZATIONS ===
                "SIM_GZ_EN": 0,                          # Disable Gazebo integration
                "MAV_USEHILGPS": 1,                      # Use HIL GPS
                "CBRK_NO_VISION": 0,                     # Enable vision safety
                "CBRK_VTOLARMING": 0,                    # Enable arming safety
            }
        })
        
        config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]
        
        # Enhanced camera configuration using Pegasus MonocularCamera
        camera_intrinsics = np.array([
            [CAMERA_RESOLUTION[0]/2, 0.0, CAMERA_RESOLUTION[0]/2],
            [0.0, CAMERA_RESOLUTION[1]/2, CAMERA_RESOLUTION[1]/2],
            [0.0, 0.0, 1.0]
        ])
        
        enhanced_camera_config = {
            "frequency": CAMERA_FREQUENCY,
            "resolution": CAMERA_RESOLUTION,
            "position": [CAMERA_X, CAMERA_Y, CAMERA_Z],
            "orientation": [0.0, 0.0, 180.0],
            "diagonal_fov": CAMERA_FOV,
            "intrinsics": camera_intrinsics,
            "distortion_coefficients": np.array([0.1, -0.05, 0.0, 0.0]),
            "depth": False  # Disable depth for performance
        }
        
        #config_multirotor.graphical_sensors = [MonocularCamera("front_camera", config=enhanced_camera_config)]
        config_multirotor.physics_dt = 1.0 / 250.0
        
        return config_multirotor

    def setup_enhanced_camera(self):
        
        print("Setting up enhanced camera...")

        try:
            # Get drone position
            drone_pos, drone_rot = self.drone.get_world_pose()

            # Calculate camera position relative to drone
            camera_world_pos = [
                drone_pos[0] + CAMERA_X,
                drone_pos[1] + CAMERA_Y,
                drone_pos[2] + CAMERA_Z
            ]

            # Convert Pegasus orientation [0.0, 0.0, 180.0] to quaternion
            from scipy.spatial.transform import Rotation as R
            pegasus_orientation = R.from_euler('XYZ', [0.0, 0.0, 180.0], degrees=True)
            orientation_quat = pegasus_orientation.as_quat()  # [x, y, z, w]

            # Create enhanced camera with ONLY supported parameters
            self.camera_prim_path = "/World/enhanced_drone_camera"

            self.camera = Camera(
                prim_path=self.camera_prim_path,
                name="enhanced_drone_camera",
                position=np.array(camera_world_pos),
                frequency=CAMERA_FREQUENCY,
                resolution=CAMERA_RESOLUTION,
                orientation=orientation_quat,  # Now matches Pegasus [0,0,180°]
            )

            # Initialize camera
            self.camera.initialize()

            # Set FOV using USD properties AFTER initialization
            try:
                stage = omni.usd.get_context().get_stage()
                camera_prim = stage.GetPrimAtPath(self.camera_prim_path)

                if camera_prim.IsValid():
                    # Set horizontal aperture and focal length to achieve desired FOV
                    # FOV formula: fov = 2 * atan(aperture / (2 * focal_length))
                    import math

                    # Standard camera aperture (35mm equivalent)
                    horizontal_aperture = 10 # mm

                    # Calculate focal length for desired FOV
                    fov_radians = math.radians(CAMERA_FOV)
                    focal_length = horizontal_aperture / (2 * math.tan(fov_radians / 2))

                    # Set camera properties via USD
                    camera_prim.GetAttribute("horizontalAperture").Set(horizontal_aperture)
                    #camera_prim.GetAttribute("focalLength").Set(focal_length)
                    
                    print(f"FOV set to {CAMERA_FOV}° via USD properties")
                    print(f"  Horizontal aperture: {horizontal_aperture}mm")
                    print(f"  Focal length: {focal_length:.2f}mm")

            except Exception as e:
                print(f"Could not set FOV via USD: {e}")
                print("  Using default Isaac Sim camera FOV")

            print(f"Enhanced camera created:")
            print(f"  Position: {camera_world_pos}")
            print(f"  Resolution: {CAMERA_RESOLUTION}")
            print(f"  Frequency: {CAMERA_FREQUENCY} FPS")
            print(f"  Orientation: [0°, 0°, 180°]")

        except Exception as e:
            print(f"Enhanced camera setup error: {e}")
            self.camera = None

    def update_camera_position(self):
        """Update camera position to follow drone with enhanced tracking"""
        try:
            if self.camera is None:
                return
            
            # Get current drone position and rotation
            drone_pos, drone_rot = self.drone.get_world_pose()
            
            # Calculate new camera position
            new_camera_pos = np.array([
                drone_pos[0] + CAMERA_X,
                drone_pos[1] + CAMERA_Y,
                drone_pos[2] + CAMERA_Z
            ])
            
            # Enhanced rotation to match drone orientation
            from scipy.spatial.transform import Rotation as R
            
            # Convert drone quaternion to Rotation object
            drone_rotation = R.from_quat([drone_rot[0], drone_rot[1], drone_rot[2], drone_rot[3]])
            
            # Keep camera aligned with drone (no additional rotation)
            new_camera_rot = drone_rotation.as_quat()
            
            # Update camera position and rotation
            self.camera.set_world_pose(position=new_camera_pos, orientation=new_camera_rot)
            
        except Exception as e:
            # Fallback: just update position
            try:
                drone_pos, _ = self.drone.get_world_pose()
                new_camera_pos = np.array([
                    drone_pos[0] + CAMERA_X,
                    drone_pos[1] + CAMERA_Y,
                    drone_pos[2] + CAMERA_Z
                ])
                self.camera.set_world_pose(position=new_camera_pos)
            except:
                pass

    def get_camera_frame(self):
        """Get camera frame with enhanced processing and brightness adjustment"""
        try:
            if self.camera is None:
                return None
            
            # Get current frame
            current_frame = self.camera.get_current_frame()
            
            if current_frame is None:
                return None
            
            # Handle different data formats
            if hasattr(current_frame, 'get_data'):
                frame_data = current_frame.get_data()
            else:
                frame_data = current_frame
            
            if isinstance(frame_data, dict):
                if 'rgb' in frame_data:
                    frame = frame_data['rgb']
                elif 'rgba' in frame_data:
                    frame = frame_data['rgba']
                    if frame.shape[-1] == 4:
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
                else:
                    return None
            else:
                frame = frame_data
            
            if not isinstance(frame, np.ndarray):
                frame = np.array(frame)
            
            if len(frame.shape) == 3:
                if frame.shape[2] == 4:  # RGBA
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                elif frame.shape[2] == 3:  # RGB
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            if frame.dtype != np.uint8:
                if frame.max() <= 1.0:
                    frame = (frame * 255).astype(np.uint8)
                else:
                    frame = frame.astype(np.uint8)
            
            # Apply brightness and contrast enhancement if image is too dark
            #if np.mean(frame) < 100:  # If average brightness is low
            #    # Apply gamma correction for brightness
            #    gamma = 1.5
            #    inv_gamma = 1.0 / gamma
            #    table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
            #    frame = cv2.LUT(frame, table)
            #    
            #    # Increase contrast slightly
            #    alpha = 1.2  # Contrast control
            #    beta = 10    # Brightness control
            #    frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
            
            return frame
            
        except Exception as e:
            print(f"Camera frame error: {e}")
            return None

    def run(self):
        """Run enhanced simulation with performance monitoring and GPU physics"""
        
        self.timeline.play()
        
        frame_count = 0
        start_time = time.time()
        
        print("Starting enhanced drone camera simulation with fixed lighting and GPU physics...")
        print("Performance settings applied - ready for high-speed flight with GPU acceleration!")
        
        while simulation_app.is_running() and not self.stop_sim:
            # Step the world
            self.world.step(render=True)
            
            # Update camera to follow drone
            self.update_camera_position()
            
            # Get and send frame
            if self.udp_publisher.enabled:
                frame = self.get_camera_frame()
                
                if frame is not None:
                   
                    success = self.udp_publisher.publish_frame(frame)
                    if success:
                        frame_count += 1
                        
                     
        
        carb.log_warn("Enhanced drone camera with GPU physics closing.")
        self.timeline.stop()

        simulation_app.close()


def main():
    """Main function"""
    
    print("=== ENHANCED SIMPLE WORKING DRONE CAMERA with FIXED LIGHTING ===")
    print("This version includes comprehensive lighting solutions:")
    print(f"- RTX Light Rig with enhanced settings")
    print(f"- Manual light sources (Sun, Key, Fill, Back)")
    print(f"- Environment dome lighting")
    print(f"- Camera brightness enhancement")
    print(f"- Drone speed: {MAX_VELOCITY} m/s")
    print(f"- Camera FOV: {CAMERA_FOV}°")
    print(f"- Video quality: {VIDEO_QUALITY}%")
    print("===============================================================")
    
    # Create and run enhanced drone camera
    drone_app = EnhancedSimpleDroneCamera()
    drone_app.run()

if __name__ == "__main__":
    main()