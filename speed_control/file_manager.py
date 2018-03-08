import _pickle
import os
import datetime

data_folder = "data/"
output_folder = "output/"

def create_folder():
    directory = os.path.join(os.getcwd(),output_folder,
                             datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
    os.makedirs(directory, exist_ok=True)
    return directory

def save_object(object, file_name):
    with open(data_folder + file_name + ".file", "wb") as f:
        _pickle.dump(object, f)
    return

def load_object(file_name):
    with open(data_folder + file_name + ".file", "rb") as f:
        return _pickle.load(f)

def generate_output_file(object, file_name):
    with open(create_folder() + "/" + file_name + ".xml", "w") as f:
        f.writelines(["%s\n" % str(item) for item in object])
