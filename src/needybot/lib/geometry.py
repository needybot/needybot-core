class Bounds(object):

    def __init__(self, rect):
        self.x = rect.get("x", 0)
        self.y = rect.get("y", 0)
        self.width = rect.get("width", 0)
        self.height = rect.get("height", 0)

