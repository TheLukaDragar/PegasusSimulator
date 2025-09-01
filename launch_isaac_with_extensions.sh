#!/bin/bash

# Simple Isaac Lab + Extensions Launcher
# Run this from anywhere - it will handle all setup automatically

set -e

echo "🚀 Starting Isaac Lab with USD Schema Extensions..."


current_dir=$(pwd)
# Navigate to Isaac Lab directory
cd ~/IsaacLab

# Activate conda environment and source Isaac Sim
echo "📦 Setting up environment..."
# source $HOME/miniconda/etc/profile.d/conda.sh
# conda activate env_isaac5
source ../isaacsim/_build/linux-x86_64/release/setup_python_env.sh

# Set environment variables for livestreaming
export LIVESTREAM=1
export PUBLIC_IP=${PUBLIC_IP:-100.112.57.52}

echo "🌐 Livestreaming will be available at: http://${PUBLIC_IP}:49100"
echo "🔧 Auto-enabling USD schema extensions..."

# Navigate back to the original directory
cd "$current_dir"

# Run Isaac Lab with all required arguments
~/IsaacLab/isaaclab.sh -p examples/0_template_app.py \
  --livestream 1 \
  --kit_args "--/app/livestream/publicEndpointAddress=${PUBLIC_IP} --/app/livestream/port=49100 --enable omni.usd.schema.flow --enable omni.usd.schema.scene.visualization --enable omni.isaac.sensor-13.0.7 --enable pegasus.simulator" 