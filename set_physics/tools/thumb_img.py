import os
import asyncio
from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": True})

import omni.usd
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file


input_usd_dir = "/ssd/tianshihan/target_69_new_bk/models/object/others/bicycle"
output_thumbnail_dir = "/ssd/tianshihan/target_69_new_bk/models_thumbnail"

os.makedirs(output_thumbnail_dir, exist_ok=True)


for root, dirs, files in os.walk(input_usd_dir):
    for file in files:
        if file.endswith(".usd"):
            usd_path = os.path.join(root, file)
            print(f"Opening: {usd_path}")
            omni.usd.get_context().open_stage(usd_path)
            stage = omni.usd.get_context().get_stage()

            viewport = get_active_viewport()
            
            thumbnail_png_name = "test"
            thumbnail_path = os.path.join(output_thumbnail_dir, f"{thumbnail_png_name}.png")
            capture_helper = capture_viewport_to_file(viewport, thumbnail_path)

            print(f"Thumbnail saved to: {thumbnail_path}")


simulation_app.close()