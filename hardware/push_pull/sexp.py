"""Mini parser/serializador de S-expressions de KiCad."""
import re

TOK = re.compile(r'\s*(\(|\)|"(?:\\.|[^"\\])*"|[^\s()"]+)')


def parse(text):
    stack = [[]]
    for m in TOK.finditer(text):
        t = m.group(1)
        if t == '(':
            stack.append([])
        elif t == ')':
            x = stack.pop()
            stack[-1].append(x)
        else:
            stack[-1].append(t)
    return stack[0][0]


def dump(x, ind=0):
    if not isinstance(x, list):
        return x
    i = 0
    head = []
    while i < len(x) and not isinstance(x[i], list):
        head.append(x[i])
        i += 1
    if i == len(x):
        return '(' + ' '.join(head) + ')'
    out = '(' + ' '.join(head)
    for e in x[i:]:
        if isinstance(e, list):
            out += '\n' + '\t' * (ind + 1) + dump(e, ind + 1)
        else:
            out += ' ' + e
    return out + '\n' + '\t' * ind + ')'


def find(x, key):
    return [e for e in x if isinstance(e, list) and e and e[0] == key]


def get(x, key):
    r = find(x, key)
    return r[0] if r else None


_cache = {}


def load_lib(path):
    if path not in _cache:
        with open(path, encoding='utf-8') as f:
            _cache[path] = parse(f.read())
    return _cache[path]


def lib_symbol(path, name):
    for e in find(load_lib(path), 'symbol'):
        if e[1] == '"%s"' % name:
            return e
    raise KeyError(name)
