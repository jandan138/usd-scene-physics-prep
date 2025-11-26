from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, Vt, UsdShade
import os
import numpy as np
import tqdm
import hashlib

transforms_name_set = set([
    "xformOp:transform",
    "xformOp:translate",
    "xformOp:orient",
    "xformOp:scale",
    "xformOpOrder"
])

file_name_set = set()

def is_absolute(path):
    if path[0] == '/':
        return True
    return False

def copyfile(src, dest):
    srct = simplify_path(src)
    destt = simplify_path(dest)
    if not os.path.exists(srct):
        print(srct, "does not exist!", src)
        pass
    else:
        # f_name = destt.split('/')[-1]
        _d_list = destt.split('/')[:-1]
        if not os.path.exists('/'.join(_d_list)):
            os.makedirs(os.path.join(*_d_list))
        # if destt[-3:] in ['png', 'jpg', 'jpeg'] and not f_name in file_name_set:
        #     file_name_set.add(f_name)
        #     print('from', srct, 'to', destt)
        
        os.system(f"cp {srct} {destt}")
    return True

def simplify_path(path):
    # print(path)
    elements = path.split('/')
    simp = []
    for e in elements:
        if e == '':
            # continue
            simp.append(e)
        elif e == '.':
            if len(simp) == 0:
                simp.append(e)
            continue
        elif e == '..':
            if len(simp) != 0 and simp[-1] != '..' and simp[-1] != '.':
                simp = simp[:-1]
            else:
                simp.append(e)
        else:
            simp.append(e)
    new_path = '/'.join(simp)
    return new_path

def remove_parent_prefix(path):
    elements = path.split('/')
    for i, e in enumerate(elements):
        if e == '..':
            continue
        else:
            elements = elements[i:]
            new_path = os.path.join(*elements)
            return new_path
    

def get_prim_transform(prim):
    if prim.HasAttribute("xformOpOrder"):
        order = prim.GetAttribute("xformOpOrder").Get()

        if order is None:
            return None
        
        tf = Gf.Transform()

        for term in order:
            if term == "xformOp:transform":
                tf_b = prim.GetAttribute(term).Get()
                if not tf_b is None:
                    # if not copy_scale:
                    #     tf_b.SetScale([1.0,1.0,1.0])

                    tm = np.array(tf.GetMatrix())
                    tm_b = np.array(tf_b)
                    tm = np.matmul(tm,tm_b)
                    tm = Gf.Matrix4d(tm)
                    tf.SetMatrix(tm)
                    
            if term == "xformOp:translate":
                tl = prim.GetAttribute(term).Get()
                if not tl is None:
                    tl = Gf.Vec3d(tl)
                    tf.SetTranslation(tl)
            
            if term == "xformOp:orient":
                to = prim.GetAttribute(term).Get()
                if not to is None:
                    rotation = Gf.Rotation()
                    rotation.SetQuat(to)
                    tf.SetRotation(rotation)
            
            if term == "xformOp:scale":
                scale = prim.GetAttribute(term).Get()
                # print(scale, prim.GetAttribute(term).GetTypeName())
                if not scale is None:
                    scale = Gf.Vec3d(scale)
                    tf.SetScale(scale)

        return tf
    
    return None
        


def transform_to_rt(prim):
    if prim.HasAttribute("xformOpOrder"):
        order = prim.GetAttribute("xformOpOrder").Get()

        if order is None:
            return None

        new_order = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
        data_type = [Sdf.ValueTypeNames.Double3, Sdf.ValueTypeNames.Quatd, Sdf.ValueTypeNames.Double3]
        if "xformOp:transform" in order:
            tf_m = prim.GetAttribute("xformOp:transform").Get()
            tf = Gf.Transform()
            tf.SetMatrix(tf_m)

            translation = tf.GetTranslation()
            orientation = tf.GetRotation().GetQuat()
            scale = tf.GetScale()
            data = [translation, orientation, scale]
        

            for term, dt, d in zip(new_order, data_type, data):
                attr = prim.CreateAttribute(term, dt, custom=False)
                attr.Set(d)
        

            prim.GetAttribute("xformOpOrder").Set(new_order)
    
    for child in prim.GetChildren():
        transform_to_rt(child)



def recursive_copy(prim_a, prim_b, copy_transform=True, src_prepend='.', dest_prepend='.', ref_prepend='.'):
    '''
        There are three kinds of properties could appear in a prim:
        1. attribute: some attributes
        2. relationship: internal prim path
        3. reference: external usd file path
    '''
    looks_b_path = "/Root/Looks"
    others_b_path = "/Root/Others"
    stage_a = prim_a.GetStage()
    stage_b = prim_b.GetStage()
    # print(prim_a.GetName(), prim_a.GetReferences(), prim_a.HasAuthoredReferences())
    

    ########## process relationships
    rels_a = prim_a.GetRelationships()    
    for rel in rels_a:
        # print(rel)
        # targets = []
        # print(rel.GetName())
        # rel.GetTargets(targets)
        # print(targets)
        # print(rel.GetName(), rel.GetTargets())
        rel_name = rel.GetName()
        targets = rel.GetTargets()

        if len(targets) > 0:
            rel_b = prim_b.CreateRelationship(rel_name, custom=False)
        
        # print(rel_name, targets, prim_a.GetPath())
        if rel_name == 'material:binding':
            # print(rel.GetAttributes())
            # print(prim_a.GetProperties())
            # help(rel)
            # print(rel, rel.HasAuthoredTargets())
            for t in targets:
                material_a = stage_a.GetPrimAtPath(t)
                material_name = material_a.GetName()
                material_b_path = os.path.join(looks_b_path, material_name)
                material_b = stage_b.GetPrimAtPath(material_b_path)
                # print(Usd.Object.IsValid(p))
                if not Usd.Object.IsValid(material_b):
                    material_b = stage_b.DefinePrim(material_b_path, material_a.GetTypeName())
                    recursive_copy(material_a, material_b, src_prepend=src_prepend, dest_prepend=dest_prepend)
                
                rel_b.AddTarget(material_b_path)
                strength = UsdShade.MaterialBindingAPI.GetMaterialBindingStrength(rel)
                # print(strength)
                UsdShade.MaterialBindingAPI.SetMaterialBindingStrength(rel_b, strength)
                # print(material_b.GetTypeName())
                # binding = UsdShade.MaterialBindingAPI.Apply(prim_b)
                # binding.Bind(material_b, UsdShade.Tokens.strongerThanDescendants, UsdShade.Tokens.allPurpose)
                
                pass
        elif rel_name == 'physics:body0' or rel_name == 'physics:body1':
            for t in targets:
                rel_p_a = stage_a.GetPrimAtPath(t)
                # rel_p_a_name = rel_p_a.GetName()
                # rel_p_b_path = os.path.join(others_b_path, rel_p_a_name)
                # rel_p_b = stage_b.GetPrimAtPath(rel_p_b_path)
                # # print(Usd.Object.IsValid(p))
                # if not Usd.Object.IsValid(rel_p_b):
                #     rel_p_b = stage_b.DefinePrim(rel_p_b_path, rel_p_a.GetTypeName())
                #     recursive_copy(rel_p_a, rel_p_b)
                if not Usd.Object.IsValid(rel_p_a):
                    print("invalid target", t)
                    continue
                rel_p_a_name = rel_p_a.GetName()
                rel_p_b_path = os.path.join("/Root", "Instance", rel_p_a_name)
                rel_b.AddTarget(rel_p_b_path)
        else:
            for t in targets:
                rel_p_a = stage_a.GetPrimAtPath(t)
                rel_p_a_name = rel_p_a.GetName()
                rel_p_b_path = os.path.join(others_b_path, rel_p_a_name)
                rel_p_b = stage_b.GetPrimAtPath(rel_p_b_path)
                if not Usd.Object.IsValid(rel_p_b):
                    rel_p_b = stage_b.DefinePrim(rel_p_b_path, rel_p_a.GetTypeName())
                    recursive_copy(rel_p_a, rel_p_b, src_prepend=src_prepend, dest_prepend=dest_prepend)
                
                rel_b.AddTarget(rel_p_b_path)

    ref_prepend = '.'
    prim = prim_a
    while Usd.Object.IsValid(prim):
        if prim.HasAuthoredReferences():
            tmp_references_list = []
            for prim_spec in prim.GetPrimStack():
                tmp_references_list.extend(prim_spec.referenceList.prependedItems)
            # print('tmp_references_list',tmp_references_list, prim_a.GetPath(), prim.GetPath())
            tmp_ref_prepend_path = os.path.join(*(str(tmp_references_list[0].assetPath).split('/')[:-1]))
            ref_prepend = simplify_path(os.path.join(tmp_ref_prepend_path, ref_prepend))
        
        prim = prim.GetParent()
    # ref_prepend = '.'
    # if prim_a.HasAuthoredReferences():
    #     print(prim_a)
    #     references_list = []
    #     for prim_spec in prim_a.GetPrimStack():
    #         references_list.extend(prim_spec.referenceList.prependedItems)
    #     ref_prepend = os.path.join(*(str(references_list[0].assetPath).split('/')[:-1]))

    # src_prepend = simplify_path(os.path.join(src_prepend, ref_prepend))
    # dest_prepend = simplify_path(os.path.join(dest_prepend, ref_prepend))

    # if prim_a.HasAuthoredReferences():
    #     # help(prim_a)
    #     references_list = []
    #     for prim_spec in prim_a.GetPrimStack():
    #         references_list.extend(prim_spec.referenceList.prependedItems)
    #     # print(references_list)
    #     # print(references_list[0].assetPath)
    #     ref_prepend_path = os.path.join(*(str(references_list[0].assetPath).split('/')[:-1]))
    #     # ref_prepend = simplify_path(os.path.join(ref_prepend, ref_prepend_path))
    #     ref_prepend = ref_prepend_path
    #     parent = prim_a.GetParent()
    #     while Usd.Object.IsValid(parent):
    #         if parent.HasAuthoredReferences():
    #             tmp_references_list = []
    #             for prim_spec in prim_a.GetPrimStack():
    #                 tmp_references_list.extend(prim_spec.referenceList.prependedItems)
    #             # print(references_list)
    #             # print(references_list[0].assetPath)
    #             tmp_ref_prepend_path = os.path.join(*(str(tmp_references_list[0].assetPath).split('/')[:-1]))
    #             ref_prepend = 

    #     src_prepend = simplify_path(os.path.join(src_prepend, ref_prepend))
    #     dest_prepend = simplify_path(os.path.join(dest_prepend, ref_prepend))

    #     # print(prim_a.GetName(), "ref_prepend", ref_prepend)

    #     # print(references.GetPrim() == prim_a)
    #     # print(prim_a.GetName(), prim_a.GetTypeName())
    #     # for ref in references.GetReferences():
    #     #     print(ref)
    #     # ps = prim_a.GetProperties()
    #     # for p in ps:
    #     #     print(p.GetName())
    #     # print("\n\n\n")


    
    # print(prim_a.GetName(), "copy_transform:", copy_transform)
    if copy_transform:
        tf_a = get_prim_transform(prim_a)
        if not tf_a is None:
            order = ["xformOp:transform"]
            xform_order = prim_b.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray, custom=False)
            xform_order.Set(order)

            tf_b = prim_b.CreateAttribute("xformOp:transform", Sdf.ValueTypeNames.Matrix4d, custom=False)
            tf_b.Set(tf_a.GetMatrix())
           
                

    ########## process attributes
    attributes = prim_a.GetAttributes()
    skip_attrs = set()
    for attr in attributes:
        name = attr.GetName()
        typename = attr.GetTypeName()
        val = attr.Get()
        # if name in transforms_name_set:
        #     print(name, typename, val)
        
        if name in skip_attrs:
            continue
        
        # var = attr.GetVariability()
        connections = attr.GetConnections()
                
        # if prim_a.GetTypeName() in ['Material', 'Shader']:
        #     print(prim_a.GetName(), name, typename, val, attr.GetConnections(), attr.GetColorSpace(), type(attr.GetColorSpace()))

        # if not copy_transform:
        #     if name in transforms_name_set:
        #         continue
        if name in transforms_name_set:
            continue
        
        # if val is not None or len(connections) > 0:
        new_attr = prim_b.CreateAttribute(name, typename, custom=False)

        # new_attr.SetVariability(var)
        if prim_a.IsA(UsdPhysics.PrismaticJoint) or prim_a.IsA(UsdPhysics.RevoluteJoint):
            if name == 'physics:jointEnabled':
                val = 0

        if val is not None:
            new_attr.Set(val)
            input_attr_name = f"inputs:{name}"
            if prim_a.HasAttribute(input_attr_name):
                skip_attrs.add(input_attr_name)
                if prim_b.HasAttribute(input_attr_name):
                    new_input_attr = prim_b.GetAttribute(input_attr_name)
                else:
                    new_input_attr = prim_b.CreateAttribute(input_attr_name, typename, custom=False)
                
                new_input_attr.Set(val)
                

        if len(connections) > 0:
            for c in connections:
                attr_father_path = str(prim_a.GetPath())
                relative_path = str(c)[len(attr_father_path)+1:]

                dest_father_path = str(prim_b.GetPath())
                connection_b = os.path.join(dest_father_path, relative_path)
                # print('attr_father_path', attr_father_path)
                # print('relative_path_path', relative_path)
                # print('father b', dest_father_path)
                # print('connection', connection_b)
                new_attr.AddConnection(connection_b)

        if typename == 'asset':
            colorspace = attr.GetColorSpace()
            if len(colorspace) > 0:
                new_attr.SetColorSpace(colorspace)
            
            if val is None:
                continue
            # print(val.GetTypeName())
            
            # filepath = str(val)[1:-1]
            filepath = val.path
            # print(filepath)
            # ap = Sdf.AssetPath(filepath)  
            # print(ap.path)
            if is_absolute(filepath) or str(filepath).startswith("http://"):
                new_attr.Set(Sdf.AssetPath(filepath))
            else:
            
                # updated_filepath = simplify_path(os.path.join(ref_prepend, filepath))
                updated_filepath = remove_parent_prefix(filepath)
                # updated_filepath = filepath
                # print(prim_a.GetPath(), attr.GetName(), ref_prepend, filepath, updated_filepath, src_prepend)
                new_attr.Set(Sdf.AssetPath(updated_filepath))
                
                # print(prim_a.GetName(), name, src_prepend, filepath)
                source_file_path = os.path.join(src_prepend, ref_prepend, filepath)
                # dest_file_path = os.path.join(dest_prepend, ref_prepend, filepath)
                dest_file_path = os.path.join(dest_prepend, updated_filepath)
                # print(dest_prepend,'=====================', ref_prepend)
                success = copyfile(source_file_path, dest_file_path)

                # print('========================')
                # print('source_file_path', source_file_path)
                # print('dest_file_path', dest_file_path)
                # print('filepath', filepath)
                # print('updated_filepath', updated_filepath)
                #     import ipdb
                #     ipdb.set_trace()
                    # print(dest_prepend)

    ########## process geometry primvars
    if prim_a.IsA(UsdGeom.Mesh):
        api_a = UsdGeom.PrimvarsAPI(prim_a)
        api_b = UsdGeom.PrimvarsAPI(prim_b)
        primvars = api_a.GetPrimvars()
        for var in primvars:
            # print(var.GetName(), var.GetInterpolation())
            name = var.GetName()
            it = var.GetInterpolation()
            if it != 'constant':
                var_b = api_b.GetPrimvar(name)
                var_b.SetInterpolation(it)
        # print(primvars)

    ########## process children 
    children = prim_a.GetChildren()
    prim_b_path = str(prim_b.GetPath())
    for child in children:
        typename = child.GetTypeName()
        name = child.GetName()
        # print(name)
        child_path = os.path.join(prim_b_path, name)
        new_child = stage_b.DefinePrim(child_path, typename)
        recursive_copy(child, new_child, src_prepend=src_prepend, dest_prepend=dest_prepend)

    pass


def create_instance(prim, path, src_prepend, dest_prepend):
    stage = Usd.Stage.CreateNew(path)
    # print(prim.GetTypeName())
    typename = prim.GetTypeName()
    root = stage.DefinePrim("/Root", "Xform")
    instance = stage.DefinePrim("/Root/Instance", typename)
    looks = stage.DefinePrim("/Root/Looks", "Scope")
    others = stage.DefinePrim("/Root/Others", "Scope")
    stage.SetDefaultPrim(root)
    recursive_copy(prim, instance, False, src_prepend=src_prepend, dest_prepend=dest_prepend)

    tf = get_prim_transform(prim)
    if tf is None:
        scale = Gf.Vec3d((1,1,1))
    else:
        scale = tf.GetScale()
    tf = Gf.Transform()
    tf.SetScale(scale)

    order = ["xformOp:transform"]
    xform_order = instance.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray, custom=False)
    xform_order.Set(order)

    tf_b = instance.CreateAttribute("xformOp:transform", Sdf.ValueTypeNames.Matrix4d, custom=False)
    tf_b.Set(tf.GetMatrix())

    transform_to_rt(instance)
        # print(attr.GetName(), attr.GetTypeName(), attr.Get())
    # print(attributes)
    stage.GetRootLayer().Save()
    

def unique_id(prim, consider_transform=True):
    res = []
    if consider_transform:
        attributes = prim.GetAttributes()
        for attr in attributes:
            name = attr.GetName()
            typename = attr.GetTypeName()
            val = attr.Get()
            try:
                if name in transforms_name_set and name != "xformOpOrder" and val is not None:
                    if name == 'xformOp:orient':
                        v = [val.real] + [_ for _ in val.imaginary]
                        v = np.array(v).flatten()
                    else:
                        v = np.array(val).flatten()
                    v_str = ["{:.2f}".format(_) for _ in v]
                    res += v_str
                    pass
            except:
                print(name, val)

    if prim.HasAuthoredReferences():
        tmp_references_list = []
        for prim_spec in prim.GetPrimStack():
            tmp_references_list.extend(prim_spec.referenceList.prependedItems)

        for r in tmp_references_list:
            ap = str(r.assetPath).split('/')[-1]
            pp = str(r.primPath)
            f = ap + "@" + pp
            res.append(f)
    for child in prim.GetChildren():
        files = unique_id(child)
        res.extend(files)

    return res

def get_renference(prim):
    res = []
    if prim.HasAuthoredReferences():
        tmp_references_list = []
        for prim_spec in prim.GetPrimStack():
            tmp_references_list.extend(prim_spec.referenceList.prependedItems)

        for r in tmp_references_list:
            ap = str(r.assetPath).split('/')[-2]
            res.append(ap)
    return res


def parse_scene(filepath, dest_path, scene_name_given=None):
    filename = filepath.split('/')[-1]
    src_prepend = "/".join(filepath.split('/')[:-1])
    stage = Usd.Stage.Open(filepath)

    if scene_name_given is None:
        scene_name = filename.split('.')[0]
    else:
        scene_name = scene_name_given
        
    scene_folder = os.path.join(dest_path, "scenes", scene_name)
    material_path = os.path.join(dest_path, "Materials")
    texture_path = os.path.join(material_path, "Textures")
    model_path = os.path.join(dest_path, "models")
    if not os.path.exists(material_path):
        os.makedirs(material_path)
        os.makedirs(texture_path)

    if not os.path.exists(scene_folder):
        os.makedirs(scene_folder)
        # os.makedirs(os.path.join(scene_folder, "Materials"))
        # os.makedirs(os.path.join(scene_folder, "Materials", "Textures"))
        os.symlink("../../Materials", os.path.join(scene_folder, "Materials"))
        # os.symlink(texture_path, os.path.join(scene_folder, "Materials", "Textures"))
        os.symlink("../../models", os.path.join(scene_folder, "models"))
        
        
        
    mdl_files = [_ for _ in os.listdir(os.path.join(src_prepend, "Materials")) if _[-3:] == 'mdl']
    for f in mdl_files:
        f_src_path = os.path.join(src_prepend, "Materials", f)
        f_dest_path = os.path.join(scene_folder, "Materials", f)
        copyfile(f_src_path, f_dest_path)

    new_stage = Usd.Stage.CreateNew(os.path.join(scene_folder, filename))
    new_stage.GetRootLayer().customLayerData = stage.GetRootLayer().customLayerData
    UsdGeom.SetStageUpAxis(new_stage, UsdGeom.Tokens.z)

    root = stage.GetDefaultPrim()
    # print(root.GetPath())
    Meshes = root.GetPrimAtPath("Meshes")
    
    
    root_new = new_stage.DefinePrim(root.GetPath(), root.GetTypeName())
    new_stage.SetDefaultPrim(root_new)


    base_scopes = ["BaseAnimation", "Base"]
    articulated_scopes = ["BaseAnimation", "Animation"]
    # interactive_scopes = ["Animation"]
    ban_list = ["Meshes", "Looks", "PhysicsMaterial"]
    for child in root.GetChildren():
        new_child = new_stage.DefinePrim(child.GetPath(), child.GetTypeName())
        if not child.GetName() in ban_list:
            recursive_copy(child, new_child, src_prepend=src_prepend, dest_prepend=scene_folder)
            pass
    
    # print(Meshes.GetPath())
    for scope in Meshes.GetChildren():
        scopeName = scope.GetName()
        # print("scope name", scopeName)
        new_stage.DefinePrim(scope.GetPath(), scope.GetTypeName())
        categories = scope.GetChildren()
        if scopeName in base_scopes:
            scope_folder = "layout"
        else:
            # continue
            scope_folder = "object"

        if scopeName in articulated_scopes:
            articualted_folder = "articulated"
        else:
            articualted_folder = "others"
    
        for cate in categories:
            new_stage.DefinePrim(cate.GetPath(), cate.GetTypeName())
            cate_name = str(cate.GetPath()).split('/')[-1]
            
            # print(cate_name)
            dest_cate_path = os.path.join(dest_path, 'models', scope_folder, articualted_folder, cate_name)
            if not os.path.exists(dest_cate_path):
                os.makedirs(dest_cate_path)

            instances = cate.GetChildren()
            # if len(cate_name) > 15:
            #     print(dest_cate_path)
            for inst in instances:
                
                # print(inst.GetPath())
                # inst_name = str(inst.GetPath()).split('/')[-1]
                inst_name = str(inst.GetName())
                # if not inst_name == 'SM_93_6BELKCETDD52CO2VKE888888':
                #     continue


                model_list = unique_id(inst, False)
                model_name_joint = '_'.join(model_list)
                temp_name_hash = hashlib.new("md5", model_name_joint.encode('utf-8'))
                model_name = temp_name_hash.hexdigest()
                # if scopeName in base_scopes:

                model_path = os.path.join(str(inst.GetParent().GetPath()), "model_"+model_name)
                inst_idx = 0
                while True:
                    new_path = model_path + f"_{inst_idx}"
                    inst_idx += 1
                    tp = new_stage.GetPrimAtPath(new_path)
                    if not Usd.Object.IsValid(tp):
                        break

                new_inst = new_stage.DefinePrim(new_path, inst.GetTypeName())
                    
                #     # temp_name = scene_name + inst_name
                #     # temp_name_hash = hashlib.new("md5", temp_name.encode('utf-8'))
                #     # model_name = temp_name_hash.hexdigest()
                #     model_name = model_name + "_base"
                # else:
                #     model_name = ''
                #     for i,_ in enumerate(inst_name.split('_')[2:]):
                #         if i == 0:
                #             model_name += _
                #         else:
                #             model_name += '_' + _
                # # print(model_name)
                # model_name = inst_name # !!!!!!!!! formating the model is not applicable for now, so do not merge the instances yet
                dest_model_folder_path = os.path.join(dest_cate_path, model_name)
                if not os.path.exists(dest_model_folder_path):
                    os.makedirs(dest_model_folder_path)
                    os.symlink("../../../../../Materials", os.path.join(dest_model_folder_path, "Materials"))
                    # os.symlink(texture_path, os.path.join(dest_model_folder_path, "Materials", "Textures"))
                    # os.makedirs(os.path.join(dest_model_folder_path, "Materials"))
                    # os.makedirs(os.path.join(dest_model_folder_path, "Materials", "Textures"))
                
                

                dest_model_path = os.path.join(dest_model_folder_path, "instance.usd")
                dest_prepend = dest_model_folder_path
                # print(dest_inst_path)
                # ref_inst_path = os.path.join("../../models", scope_folder, cate_name, model_name, "instance.usd")
                ref_inst_path = os.path.join("models", scope_folder, articualted_folder, cate_name, model_name, "instance.usd")
                if not os.path.exists(dest_model_path):
                    for f in mdl_files:
                        f_src_path = os.path.join(src_prepend, "Materials", f)
                        f_dest_path = os.path.join(dest_model_folder_path, "Materials", f)
                        copyfile(f_src_path, f_dest_path)

                    create_instance(inst, dest_model_path, src_prepend=src_prepend, dest_prepend=dest_prepend)

                new_inst.GetReferences().AddReference(ref_inst_path)


                # move the scale to the model level, which makes the instances at the same scale
                tf_inst = get_prim_transform(inst)

                order = ["xformOp:transform"]
                xform_order = new_inst.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray, custom=False)
                xform_order.Set(order)

                tf_new = Gf.Transform()
                if not tf_inst is None: 
                    tf_new.SetMatrix(tf_inst.GetMatrix())
                tf_new.SetScale([1.0, 1.0, 1.0])

                tf_m = new_inst.CreateAttribute("xformOp:transform", Sdf.ValueTypeNames.Matrix4d, custom=False)

                tf_m.Set(tf_new.GetMatrix())

                # attributes = inst.GetAttributes()
                # for attr in attributes:
                #     attr_name = attr.GetName()
                #     typename = attr.GetTypeName()
                #     if attr_name in transforms_name_set:
                #         new_attr = new_inst.CreateAttribute(attr_name, typename, custom=False)
                #         val = attr.Get()
                #         if val is not None:
                #             new_attr.Set(val)

                pass

                # break
        
            # break

        # break



    new_stage.GetRootLayer().Save()

def get_inst_model_mapping(filepath):
    # filename = filepath.split('/')[-1]
    stage = Usd.Stage.Open(filepath)

    root = stage.GetDefaultPrim()
    # print(root.GetPath())
    Meshes = root.GetPrimAtPath("Meshes")

    mapping = {}

    for scope in Meshes.GetChildren():
        for category in scope.GetChildren():
            cate = str(category.GetName())
            for instance in category.GetChildren():
                inst_name = str(instance.GetName())
                ref = get_renference(instance)
                key = cate + "/" + inst_name
                value = cate + "/" + ref[0]
                mapping[key] = value
    
    return mapping
                


def count_object(filepath):
    res = {}
    scopes = ["BaseAnimation", "Animation", "Furnitures"]
    stage = Usd.Stage.Open(filepath)
    for scope_name in scopes:
        scope = stage.GetPrimAtPath(os.path.join(f"/Root/Meshes/{scope_name}"))
        categories = scope.GetChildren()
        scope_num = 0
        for cate in categories:
            for inst in cate.GetChildren():
                scope_num += 1
        res[scope_name] = scope_num
    return res
    


def test_clean():
    file_path = '../../demo_usd/demo_0000_merge.usda'
    # file_path = '../../kujiale_0000/kujiale_0000.usda'
    # file_path = '../../commercial/start.usda'
    dest_path = '../../dest_usd'
    # dest_path = '../../kujiale_dest_usd'
    parse_scene(file_path, dest_path)