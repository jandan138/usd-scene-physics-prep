from pxr import Usd, Sdf
import json


def get_model_reference(prim):
    model_refs = set()
    references = prim.GetMetadata('references')
    for ref in references.GetAddedOrExplicitItems():
        if isinstance(ref, Sdf.Reference):
            model_refs.add(str(ref.assetPath))
    
    return model_refs


def get_material_reference(stage, prim):
    material_refs = set()
    material_bindings = prim.GetRelationship("material:binding").GetTargets()
    for material_path in material_bindings:
        material_prim = stage.GetPrimAtPath(material_path)
        if material_prim:
            for shader in material_prim.GetChildren():
                for shader_property in shader.GetProperties():
                    if shader_property.GetTypeName() == "asset":
                        material_refs.add(str(shader_property.Get().path))
    
    return material_refs


def get_model_and_material_references_set(stage):
    model_refs_list = set()
    material_refs_list = set()
    for prim in stage.Traverse():
        if prim.HasMetadata('references'):
            model_ref = get_model_reference(prim)
            if model_ref:
                model_refs_list.update(model_ref)
        
        if prim.HasRelationship("material:binding"):
            material_ref = get_material_reference(stage, prim)
            if material_ref:
                material_refs_list.update(material_ref)
        
        if prim.GetName() == "textures":
            for attr in prim.GetAttributes():
                if attr.GetTypeName() == "asset":
                    material_refs_list.add(str(attr.Get().path))
    
    material_refs_list.add("Materials/OmniUe4Base.mdl")
    material_refs_list.add("Materials/OmniUe4Function.mdl")
    material_refs_list.add("Materials/OmniUe4Translucent.mdl")
    material_refs_list.add("Materials/WorldGridMaterial.mdl")
    material_refs_list.add("Materials/DayMaterial.mdl")
    material_refs_list.add("Materials/KooPbr_maps.mdl")
    material_refs_list.add("Materials/KooPbr.mdl")
    
    return model_refs_list, material_refs_list


def parse_into_json_file(refs_list, json_file_path):
    with open(json_file_path, "w") as f:
        json.dump(refs_list, f, indent=4)


stage_path = "/ssd/tianshihan/fixed/target/scenes_test/MWAX5JYKTKJZ2AABAAAAACA8_usd/start_result_new.usd"
stage = Usd.Stage.Open(stage_path)
model_refs_list, material_refs_list = get_model_and_material_references_set(stage)

model_json_file_path = "/ssd/tianshihan/fixed/target/scenes_test/MWAX5JYKTKJZ2AABAAAAACA8_usd/model_refs.json"
parse_into_json_file(list(model_refs_list), model_json_file_path)

material_json_file_path = "/ssd/tianshihan/fixed/target/scenes_test/MWAX5JYKTKJZ2AABAAAAACA8_usd/material_refs.json"
parse_into_json_file(list(material_refs_list), material_json_file_path)