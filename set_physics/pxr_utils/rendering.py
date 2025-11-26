import omni
import omni.kit.commands

def toggle_visibility(path):
    if not isinstance(path, list):
        path = [path]
    omni.kit.commands.execute("ToggleVisibilitySelectedPrims", selected_paths=path)