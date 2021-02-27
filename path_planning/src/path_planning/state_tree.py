# tree data structure
from graphviz import Digraph, Source


class Tree:
    """
        A class to represent the structure of a node

        ...

        Attributes
        ----------
        name : str
            Name of the state.
        nodeType : str
            Type of state machine - state, parallel state machine, sequential state machine.
        children : list
            A list of the children, or substates, of the state.
        daemon : list
            A list of the UNIMPORTANT children, or substates, of the state.
        depth : int
            Represents the depth of the node of the tree within the state machine.
        number : int
            A unique integer value assigned to each state to easily differentiate between states.

        Methods
        -------
        renderTree(path : str):
            Renders the structure of the state machine tree to the specified path in the form of a .pdf file.

        renderGraph(path : str):
            Renders the flowchart of the state machine to the specified path in the form of a .pdf file.

        size() -> int:
            returns the number of nodes within the state machine
        """
    def __init__(self, name, nodeType, children=[], daemon=[], depth=0, number=1):
        if nodeType != "state" and len(children) < 1:
            assert Exception("Parallel or Sequential State has no Children")
        self.name = name
        self.children = children
        self.daemon = daemon
        self.nodeType = nodeType
        self.depth = depth
        self.number = number
        self.__assign_nums()

    # assigns numbers to each node of the tree
    def __assign_nums(self, freqArr=[]):
        if len(freqArr) == 0:
            freqArr = [i for i in range(1, self.size() + 1)]

        self.number = freqArr[0]
        freqArr.pop(0)

        for child in self.children:
            child.__assign_nums(freqArr)
        for child in self.daemon:
            child.__assign_nums(freqArr)

    # returns handles for graph
    def __return_handle(self, parent="") -> str:
        if self.nodeType == "State":
            return parent.name + str(self.depth) + str(self.number)
        else:
            return 'cluster_{}'.format(str(self.depth)) + str(self.number)

    # create graph
    def __create_graph(self, parent: Digraph, unimportant=False):
        # lowest level state
        if self.nodeType == "State":
            # graph only has one node
            parent.node(self.name, label=self.name)

        # sequential or parallel state machine
        else:
            with parent.subgraph(name=self.__return_handle(parent)) as g:
                # is unimportant
                if unimportant:
                    g.attr('node', style='filled', fillcolor='#d67272')
                else:
                    g.attr('node', style='filled', fillcolor='white')

                g.attr(label=self.name)

                # declaring children nodes
                for child in self.children:
                    if child.nodeType == "State":
                        g.node(child.__return_handle(self), label=child.name)

                    elif child.nodeType == "SequentialState" or child.nodeType == "ParallelState":
                        child.__create_graph(g)

                # sequential state
                if self.nodeType == "SequentialState":
                    # connecting children nodes
                    for i in range(len(self.children) - 1):
                        child1 = self.children[i]
                        child2 = self.children[i + 1]
                        parent1 = self
                        parent2 = self

                        while child1.nodeType != "State":
                            parent1 = child1
                            child1 = child1.children[0]
                        while child2.nodeType != "State":
                            parent2 = child2
                            child2 = child2.children[0]

                        # we are going from a non-basic state to non-basic state
                        if self.children[i].nodeType != "State" and self.children[i + 1].nodeType != "State":
                            g.edge(child1.__return_handle(parent1), child2.__return_handle(parent2),
                                   ltail=parent1.__return_handle(), lhead=parent2.__return_handle())

                        # we are going from a basic state to non-basic state
                        elif self.children[i].nodeType == "State" and self.children[i + 1].nodeType != "State":
                            g.edge(child1.__return_handle(parent1), child2.__return_handle(parent2),
                                   lhead=parent2.__return_handle())

                        # we are going from a non-basic state to basic state
                        elif self.children[i].nodeType != "State" and self.children[i + 1].nodeType == "State":
                            g.edge(child1.__return_handle(parent1), child2.__return_handle(parent2),
                                   ltail=parent1.__return_handle())

                        # we are going from a basic state to basic state
                        else:
                            g.edge(child1.__return_handle(parent1), child2.__return_handle(parent2))

                # parallel state
                else:
                    # declaring unimportant nodes
                    for child in self.daemon:
                        if child.nodeType == "State":
                            g.attr('node', style='filled', fillcolor='#d67272')
                            g.node(child.__return_handle(self), label=child.name)
                            g.attr('node', style='filled', fillcolor='white')

                        elif child.nodeType == "SequentialState" or child.nodeType == "ParallelState":
                            child.__create_graph(g, unimportant=True)

    # renders only current tree
    def __render_curr_tree(self, dot: Digraph):
        for child in self.children:
            dot.node(child.__return_handle(self), label=child.name)
            dot.edge(self.__return_handle(), child.__return_handle(self))
            child.__render_curr_tree(dot=dot)
        for child in self.daemon:
            dot.node(child.__return_handle(self), label=child.name, style='filled', fillcolor='#d67272')
            dot.edge(self.__return_handle(), child.__return_handle(self))
            child.__render_curr_tree(dot=dot)

    # returns number of nodes
    def size(self) -> int:
        total = 1
        for child in self.children:
            total += child.size()
        for child in self.daemon:
            total += child.size()
        return total

    # renders tree structure
    def render_tree(self, path: str):
        dot = Digraph(name=self.name, comment='Structure', engine='dot')
        dot.node(self.__return_handle(), label=self.name)
        self.__render_curr_tree(dot)
        Source(dot.source).render(path, view=True)

    # render graph
    def render_graph(self, path: str):
        dot = Digraph(name=self.name, comment='Flowchart', engine='dot')
        self.__create_graph(dot)
        dot.attr('graph', compound='True', nodesep='1')
        Source(dot.source).render(path, view=True)
