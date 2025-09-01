# Using Kit Arguments with PX4 Single Vehicle Simulation

This document explains how to pass kit arguments to Isaac Sim when running the PX4 single vehicle simulation.

## Overview

The updated `examples/1_px4_single_vehicle.py` script now supports kit arguments similar to Isaac Lab's AppLauncher. This allows you to:

- Enable specific extensions
- Configure livestreaming
- Set rendering parameters
- Control Isaac Sim's behavior directly

## Command Line Arguments

### Basic Arguments

- `--headless`: Run in headless mode (no GUI)
- `--livestream {0,1,2}`: Enable livestreaming (0=disabled, 1=native, 2=webrtc)
- `--enable_cameras`: Enable cameras when running without GUI
- `--kit_args "..."`: Pass kit arguments directly to Isaac Sim

### Kit Arguments Format

Kit arguments should be passed as a single string with space-separated arguments:

```bash
--kit_args "--enable extension.name --/setting/path=value"
```

## Usage Examples

### 1. Basic Run (GUI Mode)
```bash
ISAACSIM_PYTHON examples/1_px4_single_vehicle.py
```

### 2. Headless Mode
```bash
ISAACSIM_PYTHON examples/1_px4_single_vehicle.py --headless
```

### 3. With Livestreaming
```bash
ISAACSIM_PYTHON examples/1_px4_single_vehicle.py \
  --livestream 2 \
  --kit_args "--/app/livestream/publicEndpointAddress=127.0.0.1 --/app/livestream/port=49100"
```

### 4. Enable Specific Extensions
```bash
ISAACSIM_PYTHON examples/1_px4_single_vehicle.py \
  --kit_args "--enable omni.isaac.sensor-13.0.7 --enable pegasus.simulator"
```

### 5. Custom Render Settings
```bash
ISAACSIM_PYTHON examples/1_px4_single_vehicle.py \
  --kit_args "--/renderer/enabled=rtx --/rtx/rendermode=PathTracing"
```

### 6. Complex Example (Like launch_isaac_with_extensions.sh)
```bash
export PUBLIC_IP=127.0.0.1
ISAACSIM_PYTHON examples/1_px4_single_vehicle.py \
  --livestream 2 \
  --kit_args "--/app/livestream/publicEndpointAddress=${PUBLIC_IP} --/app/livestream/port=49100 --enable omni.usd.schema.flow --enable omni.usd.schema.scene.visualization --enable omni.isaac.sensor-13.0.7 --enable pegasus.simulator"
```

## Common Kit Arguments

### Extensions
- `--enable omni.isaac.sensor-13.0.7`: Enable Isaac Sim sensors
- `--enable pegasus.simulator`: Enable Pegasus simulator extension
- `--enable omni.usd.schema.flow`: Enable USD schema flow
- `--enable omni.usd.schema.scene.visualization`: Enable scene visualization

### Livestreaming
- `--/app/livestream/publicEndpointAddress=IP`: Set public IP for streaming
- `--/app/livestream/port=PORT`: Set streaming port

### Rendering
- `--/renderer/enabled=rtx`: Enable RTX rendering
- `--/rtx/rendermode=PathTracing`: Set render mode to path tracing
- `--/rtx/rendermode=RayTracedLighting`: Set render mode to ray traced lighting

### Physics
- `--/physics/cudaDevice=0`: Set CUDA device for physics

## Help

To see all available arguments:
```bash
ISAACSIM_PYTHON examples/1_px4_single_vehicle.py --help
```

## Comparison with launch_isaac_with_extensions.sh

The original shell script used Isaac Lab's launcher:
```bash
~/IsaacLab/isaaclab.sh -p examples/0_template_app.py \
  --livestream 1 \
  --kit_args "..."
```

The updated Python script provides the same functionality directly:
```bash
ISAACSIM_PYTHON examples/1_px4_single_vehicle.py \
  --livestream 2 \
  --kit_args "..."
```

This gives you more control and eliminates the need for the Isaac Lab wrapper script. 