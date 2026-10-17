"""
Node subclasses must not shadow rclpy.node.Node members.

rclpy keeps its own state in attributes such as ``handle``, ``context`` or ``_clock``. A subclass
that defines a method or attribute with one of those names breaks the node at runtime, e.g.
``AttributeError: __enter__`` (a method called ``handle``) or
``'function' object has no attribute 'handle'`` (a method called ``context``).
"""

import ast
from pathlib import Path

# Set by rclpy.node.Node.__init__ (Humble and Jazzy), not visible through dir(Node).
NODE_INTERNALS = {
    '_allow_undeclared_parameters',
    '_clock',
    '_context',
    '_default_callback_group',
    '_descriptors',
    '_logger',
    '_parameter_event_publisher',
    '_parameter_overrides',
    '_parameter_service',
    '_parameters',
    '_rate_group',
    '_time_source',
}
# Public members, kept here so the test also works without ROS installed.
NODE_PUBLIC = {
    'clients',
    'context',
    'default_callback_group',
    'executor',
    'guards',
    'handle',
    'publishers',
    'services',
    'subscriptions',
    'timers',
    'waitables',
}
# Members a subclass may override on purpose (always calling super()).
ALLOWED = {'destroy_node'}


def node_members() -> set:
    """Return every name a Node subclass must not redefine."""
    names = NODE_PUBLIC | NODE_INTERNALS
    try:
        from rclpy.node import Node
    except ImportError:
        return names - ALLOWED
    return (names | {n for n in dir(Node) if not n.startswith('__')}) - ALLOWED


def is_node_class(cls: ast.ClassDef) -> bool:
    """Return True for `class X(Node)` and `class X(rclpy.node.Node)`."""
    for base in cls.bases:
        name = base.attr if isinstance(base, ast.Attribute) else getattr(base, 'id', '')
        if name == 'Node':
            return True
    return False


def defined_names(cls: ast.ClassDef):
    """Yield (name, line) for methods and self.<name> assignments of a class."""
    for item in cls.body:
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
            yield item.name, item.lineno
        elif isinstance(item, ast.Assign):
            for target in item.targets:
                if isinstance(target, ast.Name):
                    yield target.id, item.lineno
    for node in ast.walk(cls):
        targets = []
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, (ast.AnnAssign, ast.AugAssign)):
            targets = [node.target]
        for target in targets:
            for t in ast.walk(target):
                if (
                    isinstance(t, ast.Attribute)
                    and isinstance(t.value, ast.Name)
                    and t.value.id == 'self'
                ):
                    yield t.attr, node.lineno


def test_no_shadowed_node_members():
    """Every Node subclass in this package keeps rclpy's attributes intact."""
    forbidden = node_members()
    package = Path(__file__).resolve().parents[1]
    problems = []
    for path in sorted(package.glob('*/**/*.py')):
        if 'test' in path.relative_to(package).parts[:1]:
            continue
        tree = ast.parse(path.read_text(), filename=str(path))
        for cls in (n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)):
            if not is_node_class(cls):
                continue
            for name, line in defined_names(cls):
                if name in forbidden:
                    problems.append(f'{path.name}:{line} {cls.name}.{name} shadows rclpy Node')
    assert not problems, '\n'.join(sorted(set(problems)))
