#!/usr/bin/env python3
"""Convert a map PGM and YAML + a points YAML into a PNG (binary) and a JSON file.

Usage example:
  python convert_map_and_points.py --map-yaml "galata_3_cleaned.yaml" \
    --points "points.yaml" \
    --out "galata.json" \
    --out-image "galata.png"
"""
import argparse
import json
import os
from pathlib import Path
import math
import base64

import numpy as np
from PIL import Image
import yaml


def load_map_yaml(yaml_path):
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data


def load_points_yaml(points_path):
    with open(points_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data


def convert_pgm_to_binary_png(pgm_path, negate, occupied_thresh, free_thresh, out_png_path):
    """
    Convert a PGM grayscale map into a PNG using the 'trinary' method used by map_server.
    - If negate is True, invert pixel values (255 - value).
    - occupied_thresh, free_thresh are floats between 0 and 1 (same semantics as map_server YAML).
    Output pixel values:
      - occupied  -> 0
      - free      -> 255
      - unknown   -> 205
    """
    img = Image.open(pgm_path).convert("L")
    arr = np.array(img).astype(np.uint8)

    # Convert pixel values to probabilities [0.0, 1.0]
    # Apply negate if requested
    if negate:
        p = arr / 255.0
    else:
        p = (255.0 - arr) / 255.0

    # Create output array initialized to unknown (205)
    out = np.full(p.shape, 205, dtype=np.uint8)

    # Occupied: pixel probability value >= occ_val -> 0 (black)
    out[p >= occupied_thresh] = 0

    # Free: pixel probability value <= free_val -> 255 (white)
    out[p <= free_thresh] = 255

    out_img = Image.fromarray(out, mode="L")
    out_img.save(out_png_path)
    return out_png_path


def crop_png_to_non_gray(png_path, out_path=None, unknown_value=205, pad_pixels=10):
    """
    Crop PNG so it contains only the area with pixels != unknown_value.
    Saves cropped image to out_path (or overwrites png_path if out_path is None).

    Returns:
      (bottom_left_x_px, bottom_left_y_px) - pixel coordinates of bottom-left corner
      of the cropped area relative to the original image.
    """
    img_local = Image.open(png_path).convert("L")
    arr = np.array(img_local, dtype=np.uint8)
    orig_h, orig_w = arr.shape[:2]

    mask = arr != unknown_value
    if not mask.any():
        save_path = out_path if out_path else png_path
        if save_path != png_path:
            img_local.save(save_path)
        # no crop: bottom-left corner is the original origin (0,0)
        return 0, 0

    ys, xs = np.where(mask)
    top, left = int(ys.min()), int(xs.min())
    bottom, right = int(ys.max()), int(xs.max())

    left_px = max(0, left - pad_pixels)
    upper_px = max(0, top - pad_pixels)
    right_px = min(orig_w, right + 1 + pad_pixels)
    lower_px = min(orig_h, bottom + 1 + pad_pixels)

    cropped = img_local.crop((left_px, upper_px, right_px, lower_px))
    save_path = out_path if out_path else png_path
    cropped.save(save_path)

    # bottom-left in pixel coordinates relative to original image:
    bottom_left_x_px = left_px
    bottom_left_y_px = orig_h - lower_px

    print(f"Cropped PNG image to center map area {cropped.size[0]}px x {cropped.size[1]}px")
    return bottom_left_x_px, bottom_left_y_px


def build_output_json(map_yaml, points_yaml, map_data_url, size, origin, target_resolution=0.025):
    """Build JSON ensuring output resolution = target_resolution (m/pixel).

    waypoints coordinates are returned in pixels computed with target_resolution.
    """
  
    width, height = int(size[0]), int(size[1])

    waypoints = []
    locs = points_yaml.get("Locations") if points_yaml else None
    if isinstance(locs, list):
        for item in locs:
            if not isinstance(item, dict):
                continue
            for name, body in item.items():
                pos = body.get("position", {})
                ox = float(pos.get("x", 0.0))
                oy = float(pos.get("y", 0.0))

                # convert meters -> pixels using target_resolution (m/pixel)
                px = (ox - origin[0]) / target_resolution
                py = (oy - origin[1]) / target_resolution

                # image coordinate system: origin at top-left, y down
                py_img = abs(height - py)

                # orientation -> yaw degrees (Z)
                ori = body.get("orientation", {})
                qx = float(ori.get("x", 0.0))
                qy = float(ori.get("y", 0.0))
                qz = float(ori.get("z", 0.0))
                qw = float(ori.get("w", 1.0))

                # yaw (around Z) from quaternion
                yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                yaw_deg = -math.degrees(yaw_rad)

                waypoints.append({
                    "name": name,
                    "x": float(px),
                    "y": float(py_img),
                    "yaw": float(yaw_deg),
                })

    result = {
        "map": map_data_url,
        "waypoints": waypoints,
        "info": {
            "size": {"width": width, "height": height},
            "resolution": float(target_resolution),
            "origin": {"x": 0.0, "y": 0.0},  # move origin to (0,0) in output JSON
        },
    }
    return result


def main():
    p = argparse.ArgumentParser(description="Convert map PGM+YAML and points YAML into PNG and JSON.")
    p.add_argument("--map-yaml", required=True, help="Path to map yaml (contains image, resolution, thresholds)")
    p.add_argument("--points", required=True, help="Path to points YAML (required)")
    p.add_argument("--out", required=False, help="Output JSON file to generate")
    p.add_argument("--out-image", required=False, help="Output PNG path (if omitted, same name as map image with .png)")
    args = p.parse_args()

    map_yaml_path = Path(args.map_yaml)
    if not map_yaml_path.exists():
        raise SystemExit(f"map yaml not found: {map_yaml_path}")

    map_yaml = load_map_yaml(map_yaml_path)

    # determine pgm path relative to map yaml
    pgm_name = map_yaml.get("image")
    if not pgm_name:
        raise SystemExit("map yaml does not contain 'image' entry")

    pgm_path = (map_yaml_path.parent / pgm_name).resolve()
    if not pgm_path.exists():
        raise SystemExit(f"pgm file not found: {pgm_path}")

    # points file is required
    points_path = Path(args.points)
    if not points_path.exists():
        raise SystemExit(f"points yaml not found: {points_path}")

    points_yaml = load_points_yaml(points_path)
    print(f"Using points file: {points_path}")

    # output files: default to same folder/name as map if not provided
    out_image = Path(args.out_image) if args.out_image else pgm_path.with_suffix('.png')
    out_json = Path(args.out) if args.out else map_yaml_path.with_suffix('.json')

    # create binary PNG from PGM (keeps original pixel dimensions)
    png_path = convert_pgm_to_binary_png(
        pgm_path,
        bool(map_yaml.get("negate", 0)),
        float(map_yaml.get("occupied_thresh", 0.65)),
        float(map_yaml.get("free_thresh", 0.196)),
        str(out_image),
    )

    bl_x_px, bl_y_px = crop_png_to_non_gray(
        str(out_image),
        out_path=str(out_image),
        unknown_value=205,
        pad_pixels=10
    )

    # open generated PNG to read size and possibly rescale to target resolution
    img = Image.open(png_path)
    orig_width, orig_height = img.size

    original_resolution = float(map_yaml.get("resolution", 0.0))  # m/pixel
    target_resolution = 0.025  # desired output resolution m/pixel
    
    # It's the 2D pose of the lower-left pixel in the map, origin is in the middle of the map
    # it may be useful also to compute map resolution respect to image
    origin = list(map_yaml.get("origin", [0.0, 0.0, 0.0]))          
    origin_x = float(origin[0]) + bl_x_px * original_resolution
    origin_y = float(origin[1]) + bl_y_px * original_resolution

    #################################################### UNCOMMENT SECTION TO ENABLE RESCALING
    # compute real-world size (meters)
    real_w_m = orig_width * original_resolution
    real_h_m = orig_height * original_resolution

    # compute new pixel dimensions for target resolution
    new_width = int(round(real_w_m / target_resolution))
    new_height = int(round(real_h_m / target_resolution))

    if (new_width, new_height) != (orig_width, orig_height):
        # resize using nearest neighbor to preserve binary values
        img_resized = img.resize((new_width, new_height), resample=Image.NEAREST)
        img_resized.save(out_image)
        width, height = new_width, new_height
        print(f"Rescaled PNG from {orig_width}px x {orig_height}px to {width}px x {height}px to match resolution {target_resolution} m/px")
    else:
        width, height = orig_width, orig_height
    ####################################################
    #width, height = orig_width, orig_height # keep original size, do not rescale, Comment out rescaling for now

    # encode the PNG as a data URL and gather size for JSON
    with open(out_image, "rb") as _f:
        img_b64 = base64.b64encode(_f.read()).decode('ascii')
    data_url = f"data:image/png;base64,{img_b64}"

    output_data = build_output_json(map_yaml, points_yaml, data_url, (width, height), (origin_x, origin_y),target_resolution)

    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)

    print(f"Wrote PNG: {out_image}")
    print(f"Wrote JSON: {out_json}")


if __name__ == "__main__":
    main()
