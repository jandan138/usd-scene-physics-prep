import pandas as pd

def read_physicsCsv(csv_path, valid_columns = ["Name", "Articulation", "Deformable", "Rigid_Collision", "Collision", "None", "Not Sure"]):
    df = pd.read_csv(csv_path).iloc[:,:10]
    df["Name"] = df["Class"] + "_" + df["Item"]
    df = df[valid_columns].fillna(0)

    physics_dict = dict()
    for i, row in df.iterrows():
        name = row["Name"]
        physics = row.index[row.eq(1)][0]
        physics_dict[name] = physics

    return physics_dict



if __name__ == "__main__":
    csv_path = "../info/physics_list.csv"
    physics_dict = read_physicsCsv(csv_path)
    print(physics_dict)