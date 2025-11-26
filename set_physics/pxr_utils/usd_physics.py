from pxr import Usd, UsdPhysics, UsdGeom
# To install pxr, use `pip install usd-core`
import os

def is_absolute(path):
    if path[0] == '/':
        return True
    return False

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


def to_list(data):
    res = []
    if data is not None:
        res = [_ for _ in data]
    return res

"""
    Collider
"""
def set_collider_(mesh, approx=UsdPhysics.Tokens.convexDecomposition, init_state=True):
    """set collider for a mesh
    Args:
    - mesh: Usd.Prim -> Mesh
    - approx: Convex approximation method, e.g. ConvenDecomposition, ConvexHull
    - init_state: init state of collider(whether activate)
    """
    if mesh.GetTypeName() == "Mesh" and UsdPhysics.CollisionAPI.CanApply(mesh):
        mesh_ = UsdGeom.Mesh(mesh)
        facecount = mesh_.GetFaceCount()


        faceVertexCounts = to_list(mesh.GetAttribute("faceVertexCounts").Get())
        faceVertexIndices = to_list(mesh.GetAttribute("faceVertexIndices").Get())
        facecount_calc = sum(faceVertexCounts)
        list_length = len(faceVertexIndices)
        if facecount_calc == list_length:
            collider = UsdPhysics.CollisionAPI.Apply(mesh)
            meshcollider = UsdPhysics.MeshCollisionAPI.Apply(mesh)
            
            # facecount = mesh_.GetFaceCount()
            # print(facecount)

            # if facecount > 1000:
            meshcollider.GetApproximationAttr().Set(approx)
            # else:
            # meshcollider.GetApproximationAttr().Set(UsdPhysics.Tokens.none)

            collider.GetCollisionEnabledAttr().Set(init_state)
        else:
            print(mesh, facecount, facecount_calc, list_length)

def set_collider(item, approx=UsdPhysics.Tokens.convexDecomposition, init_state=True):
    """set collider for all meshes of an Usd.Prim
    Args:
    - item: Usd.Prim, e.g. Xform
    """
    if len(item.GetChildren()) == 0:
        set_collider_(item, approx, init_state=init_state)
    for i in item.GetChildren():
        set_collider(i, approx, init_state=init_state)


def remove_collider_(mesh):
    if mesh.GetTypeName() == "Mesh":
        # if mesh.HasAPI(UsdPhysics.CollisionAPI):
        # print("remove api")
        mesh.RemoveAPI(UsdPhysics.CollisionAPI)
        # if mesh.HasAPI(UsdPhysics.MeshCollisionAPI):
        #     print("remove api2")
        mesh.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        if mesh.GetAttribute("physics:collisionEnabled"):
            mesh.GetAttribute("physics:collisionEnabled").Clear()
        if mesh.GetAttribute("physics:approximation"):
            mesh.GetAttribute("physics:approximation").Clear()

def remove_collider(item):
    # if len(item.GetChildren()) == 0:
    remove_collider_(item)
    for i in item.GetChildren():
        # print(i)
        remove_collider(i)

def activate_collider_(mesh):
    if mesh.GetTypeName() == "Mesh" and mesh.GetAttribute("physics:collisionEnabled").Get() == 0:
        mesh.GetAttribute("physics:collisionEnabled").Set(1)

def activate_collider(item):
    if len(item.GetChildren()) == 0:
        activate_collider_(item)
    for i in item.GetChildren():
        activate_collider(i)

def deactivate_collider_(mesh):
    if mesh.GetTypeName() == "Mesh" and mesh.GetAttribute("physics:collisionEnabled").Get() == 1:
        mesh.GetAttribute("physics:collisionEnabled").Set(0)

def deactivate_collider(item):
    if len(item.GetChildren()) == 0:
        deactivate_collider_(item)
    for i in item.GetChildren():
        deactivate_collider(i)

"""
    Rigid Body
"""
def set_rigidbody(item, init_state=True):
    rigidbody = UsdPhysics.RigidBodyAPI.Apply(item)
    rigidbody.GetRigidBodyEnabledAttr().Set(init_state)

def remove_rigid_(prim):
    # print(prim)
    prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
    if prim.IsA(UsdPhysics.PrismaticJoint) or prim.IsA(UsdPhysics.RevoluteJoint):
        attr = prim.GetAttribute("physics:jointEnabled")
        # print(attr.Get())
        attr.Set(False)
    
def remove_rigid(item):
    # if len(item.GetChildren()) == 0:
    remove_rigid_(item)
    for i in item.GetChildren():
        # print(i)
        remove_rigid(i)
        

def activate_rigid(prim):
    if prim.IsA(UsdPhysics.PrismaticJoint) or prim.IsA(UsdPhysics.RevoluteJoint):
        attr = prim.GetAttribute("physics:jointEnabled")
        attr.Set(True)

    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        # print(prim)
        rigidbody = UsdPhysics.RigidBodyAPI.Apply(prim)
        rigidbody.GetRigidBodyEnabledAttr().Set(True)
    else:
        for i in prim.GetChildren():
            activate_rigid(i)

def deactivate_rigid(prim):
    if prim.IsA(UsdPhysics.PrismaticJoint) or prim.IsA(UsdPhysics.RevoluteJoint):
        attr = prim.GetAttribute("physics:jointEnabled")
        attr.Set(False)

    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigidbody = UsdPhysics.RigidBodyAPI.Apply(prim)
        rigidbody.GetRigidBodyEnabledAttr().Set(False)
    else:
        for i in prim.GetChildren():
            deactivate_rigid(i)