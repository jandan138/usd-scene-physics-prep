"""
This script is used to save the usd file as flattened, 
then the flattened usd file can be opened normally whereever 
instead of export a bundle of referenced models and materials file.
NOTE: The size of the final flattened usd file may be large.
"""
from pxr import Usd


def save_flattened_usd(input_usd_path, output_usd_path):
    stage = Usd.Stage.Open(input_usd_path)
    if not stage:
        print(f"Can not open {input_usd_path}!")
        return

    flattened_stage = stage.Flatten()
    if not flattened_stage:
        print(f"Stage {input_usd_path} can not be saved as flattened!")
        return

    flattened_stage.Export(output_usd_path)
    print(f"Input usd file is saved as flattened to {output_usd_path}!")


input_usd_path = "/ssd/tianshihan/fixed/target/models_bk/object/others/sofachair/f8629cd8e21a3478271f0aadedb33ffe/instance.usd"
output_usd_path = "/ssd/tianshihan/test/instance_flattened.usd"
save_flattened_usd(input_usd_path, output_usd_path)