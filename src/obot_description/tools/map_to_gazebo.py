#!/usr/bin/env python3
"""Convert a ROS occupancy-grid YAML/PGM pair into a Gazebo SDF world."""

import argparse
import math
from pathlib import Path
from xml.sax.saxutils import escape


def read_map_yaml(path):
    values = {}
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()

    image = Path(values["image"])
    if not image.is_absolute():
        image = path.parent / image
    origin = values["origin"].strip("[] ").split(",")
    return {
        "image": image,
        "resolution": float(values["resolution"]),
        "origin_x": float(origin[0]),
        "origin_y": float(origin[1]),
        "origin_yaw": float(origin[2]),
        "negate": int(values.get("negate", "0")),
        "occupied_thresh": float(values.get("occupied_thresh", "0.65")),
    }


def read_pgm(path):
    data = path.read_bytes()
    tokens = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and data[index:index + 1].isspace():
            index += 1
        if data[index:index + 1] == b"#":
            index = data.find(b"\n", index) + 1
            continue
        end = index
        while end < len(data) and not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[index:end].decode())
        index = end

    magic, width, height, max_value = tokens
    width = int(width)
    height = int(height)
    max_value = int(max_value)
    if magic == "P5":
        pixels = data[index:index + width * height]
    elif magic == "P2":
        pixels = bytes(int(value) for value in data[index:].split())
    else:
        raise ValueError(f"unsupported image format {magic}; expected P2 or P5")
    if max_value != 255:
        pixels = bytes(round(value * 255 / max_value) for value in pixels)
    return width, height, pixels


def occupied_runs(width, height, pixels, threshold):
    occupied = [pixel <= threshold for pixel in pixels]
    rectangles = []
    for row in range(height):
        col = 0
        while col < width:
            if not occupied[row * width + col]:
                col += 1
                continue

            length = 0
            while (
                col + length < width
                and occupied[row * width + col + length]
            ):
                length += 1

            run_height = 1
            while row + run_height < height:
                next_row = row + run_height
                if not all(
                    occupied[next_row * width + cell]
                    for cell in range(col, col + length)
                ):
                    break
                run_height += 1

            rectangles.append((row, col, length, run_height))
            for covered_row in range(row, row + run_height):
                for covered_col in range(col, col + length):
                    occupied[covered_row * width + covered_col] = False
            col += length
    return rectangles


def make_world(map_path, output_path, wall_height):
    metadata = read_map_yaml(map_path)
    width, height, pixels = read_pgm(metadata["image"])
    threshold = round(255 * (1.0 - metadata["occupied_thresh"]))
    runs = occupied_runs(width, height, pixels, threshold)
    resolution = metadata["resolution"]
    map_center_x = metadata["origin_x"] + width * resolution / 2.0
    map_center_y = metadata["origin_y"] + height * resolution / 2.0
    camera_distance = max(width, height) * resolution * 0.9
    camera_x = map_center_x - camera_distance
    camera_y = map_center_y - camera_distance
    camera_height = camera_distance

    boxes = []
    for row, start, length, run_height in runs:
        x = metadata["origin_x"] + (start + length / 2.0) * resolution
        y = metadata["origin_y"] + (height - row - run_height / 2.0) * resolution
        boxes.append(
                        """        <collision name="wall_collision_{index}">
        <pose>{x:.6f} {y:.6f} {z:.6f} 0 0 {yaw:.6f}</pose>
                <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
            </collision>
            <visual name="wall_visual_{index}">
                <pose>{x:.6f} {y:.6f} {z:.6f} 0 0 {yaw:.6f}</pose>
                <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
                <material><ambient>0.55 0.55 0.58 1</ambient><diffuse>0.65 0.65 0.68 1</diffuse></material>
            </visual>""".format(
                index=len(boxes),
                x=x,
                y=y,
                z=wall_height / 2.0,
                yaw=metadata["origin_yaw"],
                sx=length * resolution,
                sy=run_height * resolution,
                sz=wall_height,
            )
        )

    world = """<?xml version="1.0" ?>
<sdf version="1.9">
  <!-- Generated from {source}. Occupied map cells are static collision boxes. -->
  <world name="obot_map_world">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
        <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>
        <scene>
            <ambient>0.8 0.8 0.8</ambient>
            <background>0.75 0.75 0.78</background>
            <grid>true</grid>
        </scene>
        <gui>
            <plugin filename="MinimalScene" name="3D View">
                <engine>ogre2</engine>
                <scene>scene</scene>
                <ambient_light>0.6 0.6 0.6</ambient_light>
                <background_color>0.75 0.75 0.78</background_color>
                <camera_pose>{camera_x:.6f} {camera_y:.6f} {camera_z:.6f} 0 0.55 0.785398</camera_pose>
            </plugin>
        </gui>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="map_walls">
      <static>true</static>
        <link name="walls">
    {boxes}
        </link>
    </model>
        <model name="ground_plane">
            <static>true</static>
            <link name="link">
                <collision name="collision">
                    <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
                </collision>
                <visual name="visual">
                    <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
                    <material><ambient>0.28 0.28 0.30 1</ambient><diffuse>0.42 0.42 0.45 1</diffuse></material>
                </visual>
            </link>
        </model>
  </world>
</sdf>
""".format(
        source=escape(str(map_path)),
        boxes="\n".join(boxes),
        camera_x=camera_x,
        camera_y=camera_y,
        camera_z=camera_height,
    )
    output_path.write_text(world)
    print(f"{map_path}: {width}x{height}, {len(boxes)} wall boxes -> {output_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_yaml", type=Path)
    parser.add_argument("output_sdf", type=Path)
    parser.add_argument("--wall-height", type=float, default=1.0)
    args = parser.parse_args()
    make_world(args.map_yaml, args.output_sdf, args.wall_height)


if __name__ == "__main__":
    main()