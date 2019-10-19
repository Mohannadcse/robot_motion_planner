'''
Author: Chris Miller '20
'''

class ConfigNode:
    # each search node except the root has a parent node
    def __init__(self, config, parent=None, action=None):
        self.config = config
        self.parent = parent
        self.action = action

    def set_parent(self, parent):
        self.parent = parent

    def get_parent(self):
        return self.parent

    def __repr__(self):

        return "Node: " + str(self.config)
