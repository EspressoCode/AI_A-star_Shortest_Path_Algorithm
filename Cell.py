from Node import Node


class Cell:
    def __init__(self, cell_number=None, list_of_nodes= None, high_risk=False):
        self.cell_number = cell_number
        self.list_of_nodes = list_of_nodes
        self.high_risk = high_risk
