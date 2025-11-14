import inspect
import os
import os.path


def get_path_to_this_file():
    return inspect.getfile(inspect.currentframe())


def get_asset_path(rel_path, check_existing=False):
    if rel_path.startswith("/"):
        full_path = rel_path
    else:
        full_path = os.path.join(os.path.dirname(__file__), "../../data/", rel_path)
    if check_existing and not os.path.exists(full_path):
        raise IOError("File %s does not exist" % full_path)

    return full_path
