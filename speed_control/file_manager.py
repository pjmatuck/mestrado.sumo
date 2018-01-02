import _pickle

data_folder = "data/"
output_folder = "output/"

def save_object(object, file_name):
    with open(data_folder + file_name + ".file", "wb") as f:
        _pickle.dump(object, f)
    return

def load_object(file_name):
    with open(data_folder + file_name + ".file", "rb") as f:
        return _pickle.load(f)

def generate_output_file(object, file_name):
    with open(output_folder + file_name + ".xml", "w") as f:
        f.writelines(["%s\n" % item for item in object])
