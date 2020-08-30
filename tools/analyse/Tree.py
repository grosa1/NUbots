import clang.cindex

import re
import itertools

# A structure for holding reactors and functions
class Tree:
    def __init__(self, diagnostics, reactors=[], functions={}):
        self.diagnostics = diagnostics
        self.reactors = reactors
        self.functions = functions


# A structure for holding information about on statements
class On:
    def __init__(self, node, dsl=None, callback=None):
        self.node = node
        self.dsl = dsl
        self.callback = callback


# A structure for holding data about emit statements
class Emit:
    existingUniqueRegex = re.compile(r"std::unique_ptr<(.*), std::default_delete<.*> >")
    makeUniqueRegex = re.compile(r"typename _MakeUniq<(.*)>::__single_object")
    nusightDataRegex = re.compile(
        r"std::unique_ptr<(.*)>"
    )  # generated by emit(graph("localisation ball pos", filter.get()[0], filter.get()[1]));

    def __init__(self, node, scope=None, tpe=None):
        self.node = node
        self.scope = scope
        self.type = tpe


# A structure for holding information about functions
class Function:
    def __init__(self, node, emit=[], on=[], calls=[], calledBy=[]):
        self.node = node
        self.emits = emit
        self.ons = on
        self.nodeCalls = [node] + calls
        self.calls = []


# A structure for holding information about reactors
class Reactor:
    def __init__(self, node, methods=[]):
        self.node = node
        self.methods = methods

    def getName(self):
        return self.node.type.spelling