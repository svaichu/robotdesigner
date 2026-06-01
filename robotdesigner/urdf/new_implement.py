

class Node:
    def __init__(self, data):
        self.data = data
        self.child = None

    def __repr__(self):
        return self.data

class RobotOrder:
    def __init__(self):
        self.head = None
        self.base = None
    
    def addLink(self, new_link_name):
        new_link = Node(new_link_name)
        if self.head is None:
            self.head = new_link
        else:
            self.head.child = new_link
            self.head = new_link
    def __index__(self, index):
        

    def __repr__(self):
        node = self.head
        nodes = []
        while node is not None:
            nodes.append(node.data)
            node = node.next
        nodes.append("None")
        return " -> ".join(nodes)