from abc import ABC, abstractmethod
from path_planning.state_tree import Tree


class BaseState(ABC):
    """
    Each state should define how it interacts with the depth and orientation control systems, because those are
    persisted across state changes.
    """
    def __repr__(self):
        """
        All subclasses should override this function if they accept arguments in __init__.
        All of those arguments should be included in the string representation for repr,
        such that eval(repr(state)) == state.
        That way, repr will provide an unambiguous representation of the state.
        State machines should recursively call __repr__ for any child states.

        If the representation is too verbose, you can optionally also implement __str__ and provide a more human
        readable representation.
        As a rule of thumb, exclude any tolerance values or verbose flags from the __str__ representation.
        State machines should also exclude recursive calls to child states in __str__.
        """
        return f'{self.__class__.__name__}()'

    def initialize(self, t, controls, sub_state, world_state, sensors):
        """
        Set up anything that needs initializing
        Run EACH time the state is chosen as the next state
        process(...) will be called with the next available data
        Note the parameters passed into function may be the same as the ones first passed into process.
        """
        pass

    def process(self, t, controls, sub_state, world_state, sensors):
        """
        This function will be called at a regular rate.
        When the state has finished and wants to finalize, it should just ensure that has_completed returns True.
        Do not manually call finalize, or else it will be called twice!
        """
        pass

    def finalize(self, t, controls, sub_state, world_state, sensors):
        """
        Clean up anything necessary.
        Note the parameters passed into function may be the same as the last ones passed into process.
        Note this may be called before the state has completed. If this occurs,
        the state should clean up any resources like normal; the exit_code in this case is not important.
        """
        pass

    @abstractmethod
    def has_completed(self):
        """
        When this function returns True, the state should exit.
        Control may be passed to another state if this is part of a state machine.
        The reason for exiting should be documented and returned in the exit_code function.
        """
        pass

    def exit_code(self):
        """
        This function should only be called once has_completed returns True.
        Indicates the reason that the state has exited.
        The code 0 signifies that the state completed its objective and exited successfully.
        The code -1 signifies that ros was shutdown.
        Other codes should be numbered 1 and higher, and their explanations should be documented.
        """
        return 0

    def get_tree(self, depth=0, verbose=False):
        """
        :param depth:
        :param verbose: If True, then repr will be used for leaf states instead of str.
        """
        return Tree(name=repr(self) if verbose else str(self),
                    nodeType="State",
                    depth=depth)
