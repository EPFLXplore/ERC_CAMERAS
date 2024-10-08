import yaml

class GetInterfaces:

    def get(name):
        path = '../../../custom_msg/config/hd_interface_names.yaml'
        with open(path, "r") as file:
            interfaces = yaml.safe_load(file)['/**']['ros__parameters']
        return interfaces[name]