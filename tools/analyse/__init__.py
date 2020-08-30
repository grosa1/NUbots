from .Tree import Tree, On, Emit, Function, Reactor

import clang.cindex

import re
import itertools

import os.path

# TODO put in a config file
libraryFile = "/usr/local/lib"  # llvm-config --libdir
clang.cindex.Config.set_library_path(libraryFile)
parseArgs = [
    "-I../build/shared",
    "-Ishared",
    "-Inuclear/message/include",
    "-I/usr/local/lib/clang/9.0.1/include",  # clang include path, clang -E -v -
    "-I/usr/local/include/eigen3",
    # "-Wall",
]

whitelist = [
    "/usr/local/include/nuclear_bits/",
]

# Creates a index for parsing
def createIndex():
    return clang.cindex.Index.create()


# Prints out one node
def printNode(node, tab=0):
    return "  " * tab + "{} {}#{} {}:{}:{}\n".format(
        node.kind.name, node.type.spelling, node.spelling, node.location.file, node.location.line, node.location.column
    )


# Print out a kinda readable tree
def printTree(node, tab=0):
    out = ""
    out += printNode(node, tab)
    for child in node.get_children():
        out += printTree(child, tab + 1)
    return out


# Creates a tree of reactors, on statements and emit statements
def createTree(index, f):
    translationUnit = index.parse(f, parseArgs)
    root = Tree(translationUnit.diagnostics)

    # Make sure that there are no errors in the parsing
    for diagnostic in translationUnit.diagnostics:
        if diagnostic.severity >= clang.cindex.Diagnostic.Error:
            print(diagnostic)
            return root

    # For the initial loop through make sure that the file the node comes from is interesting to us
    for child in translationUnit.cursor.get_children():
        good = not os.path.isabs(child.location.file.name)
        if not good:
            for path in whitelist:
                if os.path.commonpath([child.location.file.name, path]) == path:
                    good = True
                    break
        if not good:
            continue

        _treeNodeStuff(child, root)

    # Find the transitive calls
    for function in root.functions.values():
        for call in function.nodeCalls:
            try:
                function.calls.append(root.functions[call.displayname])
            except KeyError:
                pass  # A function that is not intresting

    return root


# Just loop through the tree
def _traverseTree(node, root):
    for child in node.get_children():
        _treeNodeStuff(child, root)


# Find interesting nodes in the tree
def _treeNodeStuff(node, root):
    if isFunction(node):
        function = makeFunction(node, root)
        if function:
            root.functions[function.node.displayname] = function
    elif isClass(node):
        try:
            if isInherited(next(node.get_children()), "NUClear::Reactor"):
                root.reactors.append(makeReactor(node, root))
            else:
                _traverseTree(node, root)
        except StopIteration as e:
            _traverseTree(node, root)  # Class was a forward declare with no inheritance
    else:
        _traverseTree(node, root)


# Find information about a function
def makeFunction(node, root):
    function = Function(node)

    _functionTree(node, function, root)

    # Find if the function was a method
    # You can't declare a method before declaring a class
    try:
        child = next(node.get_children())
        if isTypeRef(child):
            for reactor in root.reactors:
                if child.type.spelling == reactor.node.type.spelling:
                    reactor.methods.append(function)
    except StopIteration as e:
        pass  # Function was a forward declared and not a method

    # Check that the function is interesting
    if function.emits or function.ons or not function.nodeCalls == [node]:
        return function


# loop through the function finding interesting information
def _functionTree(node, function, root):
    for child in node.get_children():
        if isCall(child):
            if isOnCall(child):
                function.ons.append(makeOn(child, root))
            elif isEmitCall(child):
                function.emits.append(makeEmit(child))
            else:
                function.nodeCalls.append(child)
        else:
            _functionTree(child, function, root)


# Find information about an on node
def makeOn(node, root):
    on = On(node)
    children = node.get_children()

    # Find the dsl type by looping through the expected part of the tree
    try:
        dsl = ""
        for dslChild in next(next(next(next(children).get_children()).get_children()).get_children()).get_children():
            # Build up the full typename
            if dslChild.kind == clang.cindex.CursorKind.TEMPLATE_REF:
                dsl += "{}::".format(dslChild.spelling)
            elif dslChild.kind == clang.cindex.CursorKind.NAMESPACE_REF:
                dsl += "{}::".format(dslChild.spelling)
            elif dslChild.kind == clang.cindex.CursorKind.TYPE_REF:
                dsl += dslChild.type.spelling
        on.dsl = dsl
    except StopIteration as e:
        print("On DSL find StopIter:", e)

    # Find the callback for the function
    try:
        for callbackChild in children:
            if callbackChild.kind == clang.cindex.CursorKind.UNEXPOSED_EXPR:  # lambda
                callback = next(callbackChild.get_children())
                on.callback = makeFunction(callback, root)
            elif callbackChild.kind == clang.cindex.CursorKind.DECL_REF_EXPR:  # reference to predefined function
                for function in root.functions:
                    if function.node == callbackChild.referenced:
                        on.callback = function
    except StopIteration as e:
        print("On callback find StopIter:", e)

    return on


# Find information about an emit node
def makeEmit(node):
    emit = Emit(node)

    children = node.get_children()

    # Work out the scope of the node
    emit.scope = "Local"
    try:
        memberRefExpr = next(children)
        for part in memberRefExpr.get_children():
            if part.kind == clang.cindex.CursorKind.TEMPLATE_REF:
                emit.scope = part.spelling
    except StopIteration:
        print("Emit scope StopIter:", e)

    # Work out the type that is being emitted
    try:
        expr = next(children)
        if expr.kind == clang.cindex.CursorKind.UNEXPOSED_EXPR:  # The parameter is constructed in the emit statement
            regexed = re.findall(Emit.makeUniqueRegex, expr.type.spelling)
            if regexed:
                emit.type = regexed[0]
            else:
                emit.type = re.findall(Emit.nusightDataRegex, expr.type.spelling)[0]
        elif (
            expr.kind == clang.cindex.CursorKind.DECL_REF_EXPR
        ):  # The parameter is constructed outside the emit statement
            regexed = re.findall(Emit.existingUniqueRegex, expr.type.spelling)
            if regexed:
                emit.type = regexed[0]
            else:  # The parameter is not a unique pointer
                emit.type = expr.type.spelling
        elif expr.kind == clang.cindex.CursorKind.MEMBER_REF_EXPR:  # the parameter is a member of a class
            emit.type = expr.type.spelling
    except StopIteration:
        print("Emit type StopIter:", e)

    return emit


# Information about a reactor
def makeReactor(node, root):
    reactor = Reactor(node)

    # Remove forward declared reactors
    shift = 0
    last = len(root.reactors)
    for i in range(last):
        if root.reactors[i - shift].node.type.spelling == node.type.spelling:
            reactor.methods.extend(root.reactors[i - shift].methods)
            del root.reactors[i - shift]
            shift += 1

    # Find the methods in the reactor
    for child in node.get_children():
        if isCall(child):
            function = makeFunction(child)
            reactor.methods.append(function)
            root.functions[function.node.displayname] = function

    return reactor


# Checks all nodes types that could be functions
def isFunction(node):
    return (
        node.kind == clang.cindex.CursorKind.CXX_METHOD
        or node.kind == clang.cindex.CursorKind.CONSTRUCTOR
        or node.kind == clang.cindex.CursorKind.DESTRUCTOR
        or node.kind == clang.cindex.CursorKind.FUNCTION_DECL
        or node.kind == clang.cindex.CursorKind.CONVERSION_FUNCTION
    )


# Checks that a node is a type ref, the first child of a function that is a method will be a type ref.
# node the first child of a function outside a class
def isTypeRef(node):
    return node.kind == clang.cindex.CursorKind.TYPE_REF


# Is a function call
def isCall(node):
    return node.kind == clang.cindex.CursorKind.CALL_EXPR


# Check that a call is an on node
# Node must be a call
def isOnCall(node):
    return node.type.spelling == "NUClear::threading::ReactionHandle" and node.spelling == "then"


# Check that a call is an emit statement
# node must be a call
def isEmitCall(node):
    return node.spelling == "emit"


# Check that the node is a class decleration
def isClass(node):
    return node.kind == clang.cindex.CursorKind.CLASS_DECL


# The first child of a class decleration will have information about the parent if it is an child
def isInherited(node, name):
    return node.kind == clang.cindex.CursorKind.CXX_BASE_SPECIFIER and node.type.spelling == name