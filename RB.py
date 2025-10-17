from robotengine import Engine
from robotengine import Node
from robotengine import SerialIO


class Robot(Node):
    def __init__(self, name):
        super().__init__(name)
        
    def _ready(self):
        pass

    def _process(self, delta):
        print(f"[{self.engine.get_frame()}], [{round(self.engine.get_timestamp(), 2)}]")

if __name__ == '__main__':
    root = Node('root')

    robot = Robot('robot')

    root.add_child(robot)

    engine = Engine(root, frequency=240)
    engine.print_tree()
    engine.run()

