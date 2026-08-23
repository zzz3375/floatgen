#!/usr/bin/env python3
"""
Generate a complete Gazebo world SDF with all wind turbines as dynamic models.

Reads turbine positions and per-turbine parameters from config/wind_farm.yaml,
generates a URDF per turbine via farm.xacro, converts each to SDF through
``gz sdf -p``, and embeds the resulting <model> blocks into the base world
(gz/worlds/wind_farm.sdf — which no longer contains static turbine <include>s).

Usage:
    python3 scripts/generate_world.py \
        --config config/wind_farm.yaml \
        --output /tmp/wind_farm_dynamic.sdf
"""
import argparse
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml


def resolve_params(turb, defaults):
    """Merge per-turbine overrides on top of global defaults."""
    return {k: float(turb.get(k, defaults.get(k, _HARD_DEFAULTS.get(k, 0.0))))
            for k in ('velocity', 'nacelle_yaw', 'blade_pitch', 'hub_position')}


_HARD_DEFAULTS = {'velocity': -2.0, 'nacelle_yaw': 0.0,
                  'blade_pitch': 0.0, 'hub_position': 0.0}


def generate_urdf(xacro_path, name, x, y, params):
    """Run xacro to produce a single-turbine URDF."""
    cmd = [
        'ros2', 'run', 'xacro', 'xacro', xacro_path,
        'nx:=1', 'ny:=1',
        f'x:={x}', f'y:={y}',
        'scale:=200.0', 'yaw:=0.0',
        f'velocity:={params["velocity"]}',
        f'nacelle_yaw:={params["nacelle_yaw"]}',
        f'blade_pitch:={params["blade_pitch"]}',
        f'hub_position:={params["hub_position"]}',
        f'turbine_name:={name}',
    ]
    return subprocess.check_output(cmd, text=True)


def urdf_to_sdf_model(urdf_text, model_name, x, y, yaw=0.0):
    """Convert URDF text → SDF <model> element via ``gz sdf -p``."""
    # Rewrite package:// URIs to model:// so gz resolves meshes through
    # GZ_SIM_RESOURCE_PATH (the wind_turbine model is symlinked into PX4).
    urdf_text = urdf_text.replace(
        'package://floatgen/meshes/', 'model://wind_turbine/meshes/')

    with tempfile.NamedTemporaryFile(suffix='.urdf', mode='w', delete=False) as f:
        f.write(urdf_text)
        urdf_path = f.name
    try:
        sdf_text = subprocess.check_output(
            ['gz', 'sdf', '-p', urdf_path], text=True)
    finally:
        os.unlink(urdf_path)

    elem = ET.fromstring(sdf_text)
    # gz sdf -p wraps output in <sdf><model>…</model></sdf>
    model = elem if elem.tag == 'model' else elem.find('.//model')
    if model is None:
        raise RuntimeError(f'No <model> in gz sdf output for {model_name}')
    model.set('name', model_name)

    # gz sdf -p absorbs the URDF world→tower fixed joint and places the
    # root link at model origin — the (x, y) offset is lost.  Restore it
    # by setting the model pose explicitly.
    pose_elem = model.find('pose')
    if pose_elem is None:
        pose_elem = ET.SubElement(model, 'pose')
    pose_elem.text = f'{x} {y} 0 0 0 {yaw}'

    return model


def main():
    ap = argparse.ArgumentParser(description='Generate dynamic wind-farm world SDF')
    ap.add_argument('--config', required=True, help='wind_farm.yaml path')
    ap.add_argument('--output', required=True, help='output SDF path')
    args = ap.parse_args()

    floatgen_src = os.environ.get(
        'FLOATGEN_SRC', os.path.expanduser('~/ros2_ws/src/floatgen'))
    xacro_path = os.path.join(floatgen_src, 'urdf', 'farm.xacro')
    base_world = os.path.join(floatgen_src, 'gz', 'worlds', 'wind_farm.sdf')

    with open(args.config) as f:
        config = yaml.safe_load(f)

    defaults = config.get('turbine_defaults', {})
    turbines = config['turbines']

    tree = ET.parse(base_world)
    root = tree.getroot()
    world = root.find('world')
    if world is None:
        print('ERROR: no <world> element in base SDF', file=sys.stderr)
        sys.exit(1)

    for turb in turbines:
        name = turb['name']
        x, y = turb['x'], turb['y']
        params = resolve_params(turb, defaults)
        print(f'  turbine {name}  pos=({x},{y})  vel={params["velocity"]} '
              f'nacelle_yaw={params["nacelle_yaw"]}  '
              f'blade_pitch={params["blade_pitch"]}  '
              f'hub_position={params["hub_position"]}')

        urdf_text = generate_urdf(xacro_path, name, x, y, params)
        model_elem = urdf_to_sdf_model(urdf_text, name, x, y)
        world.append(model_elem)

    # Rename the world element to match the output filename so that
    # PX4's gz_bridge can find the service endpoint (/world/<name>/...).
    world.set('name', os.path.splitext(os.path.basename(args.output))[0])

    ET.indent(tree, space='  ')
    tree.write(args.output, xml_declaration=True, encoding='UTF-8')
    print(f'World SDF written to {args.output}  '
          f'({len(turbines)} dynamic turbines)')


if __name__ == '__main__':
    main()
