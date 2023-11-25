
class URDF():
    def __init__(self, file):
        self.header = """
<?xml version="1.0"?>
<robot name="robot">"""
        self.footer = """
    <material name="yellow">
        <color rgba="1 0.83 0 1"/>
    </material>
    
    <material name="blue">
        <color rgba="0.18 0.16 0.31 1"/>
    </material>
    
    <material name="pink">
        <color rgba="0.85 0.01 0.41 1"/>
    </material>
    
    <material name="purple">
        <color rgba="0.51 0.01 0.39 1"/>
    </material>
</robot>"""
    def links(self):

        with open(file, "w") as f:
            f.write()