#!/usr/bin/env python3
"""
KiCad 8 schematic connectivity extractor (standard library only, no kicad-cli).

Coordinate convention used (verified against the schematic itself, see report):
  lib pin (x, y)  ->  (x, -y)                      (lib +y up, schematic +y down)
                  ->  rotate by symbol angle CCW as seen on screen:
                        X' =  X*cos(a) + Y*sin(a)
                        Y' = -X*sin(a) + Y*cos(a)
                  ->  (mirror x): Y' = -Y'   /  (mirror y): X' = -X'
                  ->  translate by symbol (at x y)
The `(at x y angle)` of a lib pin IS its electrical connection point.
"""
import sys, os, re, math
from collections import defaultdict, OrderedDict

# usage: kicad_netlist.py [top_schematic.kicad_sch] [output_dir]
#   The .kicad_pcb next to the schematic (same base name) is used to cross-check
#   the derived nets against KiCad's own pad-to-net assignment when present.
TOP_FILE = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else \
    "/home/emiel/Documents/ShiftRegWS2811/ShiftWS2811_PCB/KiCad/ShiftWS2811_PCB.kicad_sch"
SCH_DIR = os.path.dirname(TOP_FILE)
PCB_FILE = os.path.splitext(TOP_FILE)[0] + ".kicad_pcb"
OUT_DIR = os.path.abspath(sys.argv[2]) if len(sys.argv) > 2 else os.getcwd()
TOL = 0.01

# --------------------------------------------------------------------------- s-expr
class Sym(str):
    """bare (unquoted) token"""

def parse_sexpr(text):
    stack = [[]]
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c.isspace():
            i += 1
        elif c == '(':
            stack.append([]); i += 1
        elif c == ')':
            lst = stack.pop(); stack[-1].append(lst); i += 1
        elif c == '"':
            j = i + 1; buf = []
            while j < n:
                ch = text[j]
                if ch == '\\' and j + 1 < n:
                    buf.append(text[j + 1]); j += 2; continue
                if ch == '"':
                    break
                buf.append(ch); j += 1
            stack[-1].append(''.join(buf)); i = j + 1
        else:
            j = i
            while j < n and not text[j].isspace() and text[j] not in '()':
                j += 1
            stack[-1].append(Sym(text[i:j])); i = j
    if len(stack) != 1 or len(stack[0]) != 1:
        raise ValueError("unbalanced s-expression")
    return stack[0][0]

def kids(node, key):
    return [x for x in node[1:] if isinstance(x, list) and x and x[0] == key]

def kid(node, key):
    for x in node[1:]:
        if isinstance(x, list) and x and x[0] == key:
            return x
    return None

def prop(node, name):
    for p in kids(node, 'property'):
        if len(p) >= 3 and p[1] == name:
            return p[2]
    return None

def num(x):
    return float(x)

def has_flag(node, name):
    """(hide yes) / bare hide"""
    for x in node[1:]:
        if isinstance(x, list) and x and x[0] == name:
            return len(x) == 1 or str(x[1]) == 'yes'
        if not isinstance(x, list) and isinstance(x, Sym) and str(x) == name:
            return True
    return False

# --------------------------------------------------------------------------- lib symbols
class LibPin:
    __slots__ = ('name', 'number', 'x', 'y', 'angle', 'length', 'etype', 'hidden', 'unit', 'style')

def mk_pin(p, unit, style):
    lp = LibPin()
    lp.etype = str(p[1]) if len(p) > 1 and not isinstance(p[1], list) else '?'
    at = kid(p, 'at')
    lp.x, lp.y = num(at[1]), num(at[2])
    lp.angle = num(at[3]) if len(at) > 3 else 0.0
    ln = kid(p, 'length'); lp.length = num(ln[1]) if ln else 0.0
    nm = kid(p, 'name'); lp.name = nm[1] if nm else ''
    nb = kid(p, 'number'); lp.number = nb[1] if nb else ''
    lp.hidden = has_flag(p, 'hide')
    lp.unit, lp.style = unit, style
    return lp

def parse_lib_symbols(root):
    libs = {}
    ls = kid(root, 'lib_symbols')
    if not ls:
        return libs
    raw = OrderedDict((s[1], s) for s in kids(ls, 'symbol'))

    def pins_of(s):
        res = [mk_pin(p, 0, 0) for p in kids(s, 'pin')]
        for sub in kids(s, 'symbol'):
            m = re.match(r'^(.*)_(\d+)_(\d+)$', sub[1])
            unit, style = (int(m.group(2)), int(m.group(3))) if m else (0, 0)
            res += [mk_pin(p, unit, style) for p in kids(sub, 'pin')]
        return res

    for name, s in raw.items():
        pins = pins_of(s)
        ext = kid(s, 'extends')
        if ext:
            prefix = name.split(':')[0] + ':' if ':' in name else ''
            parent = raw.get(prefix + ext[1]) or raw.get(ext[1])
            if parent:
                pins = pins_of(parent) + pins
        libs[name] = {'pins': pins,
                      'power': kid(s, 'power') is not None,
                      'value': prop(s, 'Value')}
    return libs

def transform_pin(lp, sx, sy, angle, mirror):
    x, y = lp.x, -lp.y
    a = math.radians(angle)
    c, s = round(math.cos(a)), round(math.sin(a))          # angles are multiples of 90 deg
    X = x * c + y * s
    Y = -x * s + y * c
    if mirror == 'x':
        Y = -Y
    elif mirror == 'y':
        X = -X
    return (round(sx + X, 4), round(sy + Y, 4))

# --------------------------------------------------------------------------- design
class Node:
    __slots__ = ('id', 'kind', 'path', 'x', 'y', 'text', 'ref', 'pin', 'pinname', 'etype',
                 'power', 'sheet', 'uuid', 'value', 'lib', 'hidden', 'sym_uuid')
    def __init__(self, kind, path, x, y):
        self.kind, self.path, self.x, self.y = kind, path, x, y
        self.text = self.ref = self.pin = self.pinname = self.etype = self.power = None
        self.sheet = self.uuid = self.value = self.lib = self.sym_uuid = None
        self.hidden = False

class Design:
    def __init__(self):
        self.nodes = []
        self.parent = []
        self.wires = []          # (path, x1,y1,x2,y2, node_id)
        self.sheets = []         # dicts
        self.symbols = []        # dicts
        self.log = []
        self.sheet_names = {'': '<top>'}

    # union-find
    def add(self, node):
        node.id = len(self.nodes); self.nodes.append(node); self.parent.append(node.id); return node
    def find(self, a):
        while self.parent[a] != a:
            self.parent[a] = self.parent[self.parent[a]]; a = self.parent[a]
        return a
    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[max(ra, rb)] = min(ra, rb)

    # ------------------------------------------------------------------ load one sheet file
    def load_sheet(self, filename, path_key, root_uuid, sheet_display):
        text = open(filename, encoding='utf-8').read()
        root = parse_sexpr(text)
        libs = parse_lib_symbols(root)
        self.sheet_names[path_key] = sheet_display
        if root_uuid is None:
            root_uuid = kid(root, 'uuid')[1]
        want_path = '/' + root_uuid + path_key
        first_nodes = len(self.nodes)

        # symbols
        for s in kids(root, 'symbol'):
            lib_id = kid(s, 'lib_id')[1]
            ln = kid(s, 'lib_name'); lib_key = ln[1] if ln else lib_id
            at = kid(s, 'at'); sx, sy = num(at[1]), num(at[2]); ang = num(at[3]) if len(at) > 3 else 0.0
            mir = kid(s, 'mirror'); mirror = str(mir[1]) if mir else None
            u = kid(s, 'unit'); unit = int(u[1]) if u else 1
            bs = kid(s, 'body_style') or kid(s, 'convert'); style = int(bs[1]) if bs else 1
            uuid = kid(s, 'uuid')[1]
            ref = prop(s, 'Reference'); value = prop(s, 'Value')
            inst = kid(s, 'instances')
            if inst:
                for proj in kids(inst, 'project'):
                    for p in kids(proj, 'path'):
                        if p[1] == want_path:
                            r = kid(p, 'reference')
                            if r: ref = r[1]
                            uu = kid(p, 'unit')
                            if uu: unit = int(uu[1])
            lib = libs.get(lib_key)
            if lib is None:
                self.log.append(f"WARNING: lib symbol {lib_key} not found for {ref}")
                continue
            is_power = lib['power'] or lib_id.startswith('power:') or (ref or '').startswith('#PWR')
            pins = [lp for lp in lib['pins'] if lp.unit in (0, unit) and lp.style in (0, style)]
            sym = {'ref': ref, 'value': value, 'lib': lib_id, 'path': path_key, 'uuid': uuid,
                   'at': (sx, sy, ang), 'mirror': mirror, 'unit': unit, 'power': is_power, 'pins': []}
            for lp in pins:
                X, Y = transform_pin(lp, sx, sy, ang, mirror)
                nd = self.add(Node('pin', path_key, X, Y))
                nd.ref, nd.pin, nd.pinname, nd.etype, nd.hidden = ref, lp.number, lp.name, lp.etype, lp.hidden
                nd.value, nd.lib, nd.sym_uuid = value, lib_id, uuid
                if is_power:
                    nd.power = value or lp.name
                elif lp.hidden and lp.etype == 'power_in':
                    # KiCad: invisible power-input pins are implicitly connected to the net named after the pin
                    nd.power = lp.name
                    self.log.append(f"NOTE: {ref} pin {lp.number} ({lp.name}) is a hidden power_in pin -> implicit net '{lp.name}'")
                sym['pins'].append(nd.id)
            self.symbols.append(sym)

        # wires (and buses, kept separate)
        for w in kids(root, 'wire'):
            pts = kid(w, 'pts'); xy = kids(pts, 'xy')
            x1, y1, x2, y2 = num(xy[0][1]), num(xy[0][2]), num(xy[1][1]), num(xy[1][2])
            wn = self.add(Node('wire', path_key, x1, y1))
            e1 = self.add(Node('wire_end', path_key, x1, y1))
            e2 = self.add(Node('wire_end', path_key, x2, y2))
            self.union(wn.id, e1.id); self.union(wn.id, e2.id)
            self.wires.append((path_key, x1, y1, x2, y2, wn.id))
        for b in kids(root, 'bus'):
            self.log.append(f"NOTE: bus present on {sheet_display} (not netlisted)")

        # junctions
        for j in kids(root, 'junction'):
            at = kid(j, 'at'); self.add(Node('junction', path_key, num(at[1]), num(at[2])))
        for nc in kids(root, 'no_connect'):
            at = kid(nc, 'at'); self.add(Node('no_connect', path_key, num(at[1]), num(at[2])))

        # labels
        for kind, key in (('label', 'label'), ('glabel', 'global_label'), ('hlabel', 'hierarchical_label')):
            for l in kids(root, key):
                at = kid(l, 'at')
                nd = self.add(Node(kind, path_key, num(at[1]), num(at[2]))); nd.text = l[1]

        # sheets (hierarchical sheet instances)
        for sh in kids(root, 'sheet'):
            at = kid(sh, 'at'); size = kid(sh, 'size')
            name = prop(sh, 'Sheetname'); fil = prop(sh, 'Sheetfile'); uuid = kid(sh, 'uuid')[1]
            page = None
            inst = kid(sh, 'instances')
            if inst:
                for proj in kids(inst, 'project'):
                    for p in kids(proj, 'path'):
                        pg = kid(p, 'page')
                        if pg: page = pg[1]
            sd = {'name': name, 'file': fil, 'uuid': uuid, 'path': path_key, 'page': page,
                  'at': (num(at[1]), num(at[2])), 'size': (num(size[1]), num(size[2])), 'pins': []}
            for p in kids(sh, 'pin'):
                pat = kid(p, 'at')
                nd = self.add(Node('sheetpin', path_key, num(pat[1]), num(pat[2])))
                nd.text, nd.sheet, nd.uuid = p[1], name, uuid
                sd['pins'].append(nd.id)
            self.sheets.append(sd)

        # ---- geometric connectivity within this sheet
        new_nodes = self.nodes[first_nodes:]
        grid = defaultdict(list)
        for nd in new_nodes:
            if nd.kind in ('wire', 'junction'):
                continue
            grid[(round(nd.x * 100), round(nd.y * 100))].append(nd.id)
        for ids in grid.values():
            for i in ids[1:]:
                self.union(ids[0], i)
        wires_here = [w for w in self.wires if w[0] == path_key]
        for nd in new_nodes:
            if nd.kind in ('wire', 'junction', 'no_connect'):
                continue
            for (_, x1, y1, x2, y2, wid) in wires_here:
                if point_on_segment(nd.x, nd.y, x1, y1, x2, y2):
                    self.union(nd.id, wid)
        # local labels: same text on same sheet instance
        by_text = defaultdict(list)
        for nd in new_nodes:
            if nd.kind == 'label':
                by_text[nd.text].append(nd.id)
        for ids in by_text.values():
            for i in ids[1:]:
                self.union(ids[0], i)
        # hierarchical labels with the same text inside one sheet are the same net
        by_text = defaultdict(list)
        for nd in new_nodes:
            if nd.kind == 'hlabel':
                by_text[nd.text].append(nd.id)
        for ids in by_text.values():
            for i in ids[1:]:
                self.union(ids[0], i)
        # recurse into sub-sheets
        for sd in [s for s in self.sheets if s['path'] == path_key]:
            sub_key = path_key + '/' + sd['uuid']
            sub_file = os.path.join(os.path.dirname(filename), sd['file'])
            self.load_sheet(sub_file, sub_key, root_uuid, sd['name'])
            # connect sheet pins to hierarchical labels of the same name inside
            hl = defaultdict(list)
            for nd in self.nodes:
                if nd.kind == 'hlabel' and nd.path == sub_key:
                    hl[nd.text].append(nd.id)
            for pid in sd['pins']:
                pn = self.nodes[pid]
                if pn.text in hl:
                    for i in hl[pn.text]:
                        self.union(pid, i)
                else:
                    self.log.append(f"WARNING: sheet {sd['name']} pin {pn.text} has no matching hierarchical label inside {sd['file']}")

    def finish(self):
        # power nets: global by name; global labels: global by text
        by_power = defaultdict(list); by_gl = defaultdict(list)
        for nd in self.nodes:
            if nd.power:
                by_power[nd.power].append(nd.id)
            if nd.kind == 'glabel':
                by_gl[nd.text].append(nd.id)
        for ids in list(by_power.values()) + list(by_gl.values()):
            for i in ids[1:]:
                self.union(ids[0], i)
        # nets
        self.nets = defaultdict(list)
        for nd in self.nodes:
            self.nets[self.find(nd.id)].append(nd.id)
        self.netname = {}
        for root, ids in self.nets.items():
            self.netname[root] = self.name_net(ids)

    def name_net(self, ids):
        nodes = [self.nodes[i] for i in ids]
        pw = sorted({n.power for n in nodes if n.power})
        if pw: return pw[0]
        gl = sorted({n.text for n in nodes if n.kind == 'glabel'})
        if gl: return gl[0]
        # local labels / hierarchical labels / sheet pins: prefer the highest sheet level (shortest path)
        cands = []
        for n in nodes:
            if n.kind == 'label':
                cands.append((n.path.count('/'), 0, self.sheet_prefix(n.path) + n.text))
            elif n.kind == 'sheetpin':
                cands.append((n.path.count('/') + 1, 1, self.sheet_prefix(n.path + '/' + n.uuid) + n.text))
            elif n.kind == 'hlabel':
                cands.append((n.path.count('/'), 1, self.sheet_prefix(n.path) + n.text))
        if cands:
            return sorted(cands)[0][2]
        pins = [n for n in nodes if n.kind == 'pin']
        if len(pins) == 1:
            p = pins[0]; return f"unconnected-({p.ref}-{p.pinname}-Pad{p.pin})"
        if pins:
            p = sorted(pins, key=lambda n: (refkey(n.ref), n.pin))[0]
            return f"Net-({p.ref}-{p.pinname})"
        return "<no-pins>"

    def sheet_prefix(self, path):
        if path == '':
            return '/'
        parts = path.split('/')[1:]
        names = []
        for i in range(len(parts)):
            key = '/' + '/'.join(parts[:i + 1])
            names.append(self.sheet_names.get(key, parts[i]))
        return '/' + '/'.join(names) + '/'

    # helpers
    def net_of(self, node_id):
        return self.find(node_id)
    def net_nodes(self, node_id):
        return [self.nodes[i] for i in self.nets[self.find(node_id)]]
    def net_pins(self, node_id, exclude_power_symbols=True):
        return [n for n in self.net_nodes(node_id) if n.kind == 'pin' and not (exclude_power_symbols and n.ref.startswith('#'))]
    def net_labels(self, node_id, path=None):
        res = []
        for n in self.net_nodes(node_id):
            if n.kind in ('label', 'glabel', 'hlabel') and (path is None or n.path == path):
                res.append(n.text)
        return sorted(set(res))
    def connected(self, node_id):
        """a pin counts as connected if its net has anything else electrically meaningful"""
        others = [n for n in self.net_nodes(node_id) if n.id != node_id and n.kind in ('pin', 'label', 'glabel', 'hlabel', 'sheetpin', 'wire')]
        return len(others) > 0

def point_on_segment(px, py, x1, y1, x2, y2, tol=TOL):
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return abs(px - x1) <= tol and abs(py - y1) <= tol
    cross = dx * (py - y1) - dy * (px - x1)
    if abs(cross) / math.sqrt(L2) > tol:
        return False
    t = ((px - x1) * dx + (py - y1) * dy) / L2
    return -tol / math.sqrt(L2) <= t <= 1 + tol / math.sqrt(L2)

def refkey(r):
    m = re.match(r'([A-Za-z#]+)(\d+)', r or '')
    return (m.group(1), int(m.group(2))) if m else (r or '', 0)

def pinkey(p):
    return (0, int(p)) if str(p).isdigit() else (1, str(p))

# --------------------------------------------------------------------------- reporting helpers
def fmt_pins(pins, d, with_sheet=True, max_items=60):
    """compact 'REF.pin(name)' list grouped by ref"""
    by_ref = defaultdict(list)
    for p in pins:
        by_ref[(p.path, p.ref)].append(p)
    out = []
    for (path, ref), ps in sorted(by_ref.items(), key=lambda kv: (kv[0][0], refkey(kv[0][1]))):
        ps.sort(key=lambda p: pinkey(p.pin))
        sh = '' if (path == '' or not with_sheet) else f"[{d.sheet_names[path]}] "
        if len(ps) > 8:
            nums = [p.pin for p in ps]
            out.append(f"{sh}{ref} pins {compress_numbers(nums)} ({len(ps)} pins)")
        else:
            out.append(f"{sh}{ref} " + ", ".join(f"pin {p.pin} ({p.pinname})" for p in ps))
    if len(out) > max_items:
        out = out[:max_items] + [f"... and {len(out) - max_items} more"]
    return "; ".join(out)

def compress_numbers(nums):
    try:
        ns = sorted(int(x) for x in nums)
    except ValueError:
        return ",".join(nums)
    # detect odd/even runs
    if len(ns) >= 4 and all(b - a == 2 for a, b in zip(ns, ns[1:])):
        return f"{ns[0]},{ns[1]},...,{ns[-1]} (step 2)"
    if len(ns) >= 4 and all(b - a == 1 for a, b in zip(ns, ns[1:])):
        return f"{ns[0]}..{ns[-1]}"
    return ",".join(str(x) for x in ns)

def sym_by_ref(d, ref, path=''):
    for s in d.symbols:
        if s['ref'] == ref and s['path'] == path:
            return s
    return None

def pin_nodes(d, sym):
    return sorted((d.nodes[i] for i in sym['pins']), key=lambda n: pinkey(n.pin))

def describe_net(d, node, path_for_labels=None, with_sheet=True):
    name = d.netname[d.find(node.id)]
    labels = d.net_labels(node.id, path_for_labels)
    others = [p for p in d.net_pins(node.id) if p.id != node.id]
    return name, labels, others

# --------------------------------------------------------------------------- report sections
def section_a(d, out):
    out.append("=" * 100)
    out.append("(a) TOP SHEET: Teensy 4.1 (U1) pin connections")
    out.append("=" * 100)
    u1 = sym_by_ref(d, 'U1')
    out.append(f"U1 = {u1['value']} ({u1['lib']}) at {u1['at']} mirror={u1['mirror']} unit={u1['unit']}")
    out.append("")
    out.append(f"{'Pin#':<5} {'Pin name':<22} {'Net name':<22} {'Top-sheet labels':<22} Other pins on the net")
    out.append("-" * 100)
    unconnected = []
    for p in pin_nodes(d, u1):
        if not d.connected(p.id):
            unconnected.append(p); continue
        name, labels, others = describe_net(d, p, path_for_labels='')
        out.append(f"{p.pin:<5} {p.pinname:<22} {name:<22} {','.join(labels):<22} {fmt_pins(others, d)}")
    out.append("")
    out.append("U1 pins with NO connection: " + ", ".join(f"{p.pin} ({p.pinname})" for p in unconnected))
    out.append("")

def buffer_pairs(sym_pins, kind):
    """returns list of (input_pin, output_pin) node pairs by name pattern"""
    by_name = {p.pinname: p for p in sym_pins}
    pairs = []
    if kind == '240':
        for n, p in by_name.items():
            m = re.match(r'^(\d)A(\d)$', n)
            if m:
                y = by_name.get(f"{m.group(1)}Y{m.group(2)}")
                if y: pairs.append((p, y))
    else:
        for n, p in by_name.items():
            m = re.match(r'^A(\d)$', n)
            if m:
                b = by_name.get(f"B{m.group(1)}")
                if b: pairs.append((p, b))
    return sorted(pairs, key=lambda t: t[0].pinname)

DS240 = {'2': '1A1', '4': '1A2', '6': '1A3', '8': '1A4', '18': '1Y1', '16': '1Y2', '14': '1Y3', '12': '1Y4',
         '17': '2A1', '15': '2A2', '13': '2A3', '11': '2A4', '3': '2Y1', '5': '2Y2', '7': '2Y3', '9': '2Y4',
         '1': '1OE', '19': '2OE', '10': 'GND', '20': 'VCC'}
DS245 = {str(2 + k): f'A{k + 1}' for k in range(8)}
DS245.update({str(18 - k): f'B{k + 1}' for k in range(8)})
DS245.update({'1': 'DIR', '19': 'OE', '10': 'GND', '20': 'VCC'})

def section_b(d, out):
    out.append("=" * 100)
    out.append("(b) TOP SHEET: buffers U2 (74AHCT240) and U3, U4, U5 (74AHCT245, drawn with 74xx:74HC245 symbol)")
    out.append("=" * 100)
    for ref in ('U2', 'U3', 'U4', 'U5'):
        s = sym_by_ref(d, ref)
        pins = pin_nodes(d, s)
        kind = '240' if '240' in s['lib'] else '245'
        ds = DS240 if kind == '240' else DS245
        out.append("")
        out.append(f"--- {ref} = {s['value']} ({s['lib']}) at {s['at']} mirror={s['mirror']} ---")
        out.append(f"{'Pin#':<5} {'KiCad name':<11} {'Datasheet':<10} {'Net name':<20} {'Top labels':<20} Other pins on net")
        for p in pins:
            if not d.connected(p.id):
                out.append(f"{p.pin:<5} {p.pinname:<11} {ds.get(p.pin,''):<10} {'*** UNCONNECTED ***'}")
                continue
            name, labels, others = describe_net(d, p, path_for_labels='')
            out.append(f"{p.pin:<5} {p.pinname:<11} {ds.get(p.pin,''):<10} {name:<20} {','.join(labels):<20} {fmt_pins(others, d)}")
        out.append("")
        if kind == '240':
            out.append("  Inverting buffer channels (input -> /output), 1OE/2OE control (active-low enables):")
            for a, y in buffer_pairs(pins, '240'):
                out.append(f"    {a.pinname} (pin {a.pin}, {ds[a.pin]}) = {d.netname[d.find(a.id)]:<18} --> NOT --> {y.pinname} (pin {y.pin}, {ds[y.pin]}) = {d.netname[d.find(y.id)]}")
            for n in ('1OE', '2OE', 'VCC', 'GND'):
                p = [q for q in pins if q.pinname == n][0]
                out.append(f"    {n} (pin {p.pin}) tied to {d.netname[d.find(p.id)]}")
        else:
            out.append("  Transceiver channels A<->B; DIR (pin 1, 'A->B' in KiCad) and OE (pin 19, 'CE' in KiCad, active low):")
            for a, b in buffer_pairs(pins, '245'):
                out.append(f"    {a.pinname} (pin {a.pin}, {ds[a.pin]}) = {d.netname[d.find(a.id)]:<18} <-> {b.pinname} (pin {b.pin}, {ds[b.pin]}) = {d.netname[d.find(b.id)]}")
            for n in ('A->B', 'CE', 'VCC', 'GND'):
                p = [q for q in pins if q.pinname == n][0]
                label = {'A->B': 'DIR', 'CE': '!OE'}.get(n, n)
                out.append(f"    {n} (pin {p.pin}, {label}) tied to {d.netname[d.find(p.id)]}")
            aside = sorted({d.netname[d.find(a.id)] for a, b in buffer_pairs(pins, '245')})
            bside = sorted({d.netname[d.find(b.id)] for a, b in buffer_pairs(pins, '245')})
            out.append(f"    A-side nets: {', '.join(aside)}")
            out.append(f"    B-side nets: {', '.join(bside)}")
    out.append("")

def section_c(d, out):
    out.append("=" * 100)
    out.append("(c) TOP SHEET: hierarchical sheet instances of OutputBlock.kicad_sch and their pin -> top-level net mapping")
    out.append("=" * 100)
    top_sheets = sorted([s for s in d.sheets if s['path'] == ''], key=lambda s: (s['at'][0], s['at'][1]))
    out.append(f"Found {len(top_sheets)} sheet instances (each one contains 2x 74AHCT595 = 16 outputs):")
    for s in top_sheets:
        out.append(f"  '{s['name']}' page {s['page']} file={s['file']} at={s['at']} size={s['size']} uuid={s['uuid']}")
    for s in top_sheets:
        out.append("")
        sub_key = '/' + s['uuid']
        # references inside this instance
        refs = sorted({sym['ref'] for sym in d.symbols if sym['path'] == sub_key and not sym['power']}, key=refkey)
        out.append(f"--- Sheet '{s['name']}' (page {s['page']}) -- components inside this instance: {', '.join(refs)} ---")
        out.append(f"{'Sheet pin':<11} {'Top net':<22} {'Top label(s) at pin':<22} Other top-level connections (pins on same net, top sheet + other instances)")
        for pid in sorted(s['pins'], key=lambda i: pinkey_name(d.nodes[i].text)):
            pn = d.nodes[pid]
            name = d.netname[d.find(pid)]
            labels = d.net_labels(pid, path='')
            others = [p for p in d.net_pins(pid) if p.path != sub_key]
            inside = [p for p in d.net_pins(pid) if p.path == sub_key]
            out.append(f"{pn.text:<11} {name:<22} {','.join(labels):<22} {fmt_pins(others, d)}")
            out.append(f"{'':<11} {'':<22} {'inside the block:':<22} {fmt_pins(inside, d, with_sheet=False)}")
    out.append("")

def pinkey_name(t):
    m = re.match(r'^OUT_(\d+)$', t)
    order = {'DATA': 0, 'SHIFT_CLK': 1, 'STORE_CLK': 2, 'OE': 3, 'COMMON': 4}
    if m: return (10, int(m.group(1)))
    return (order.get(t, 5), 0)

ALT595 = {'SER': 'DS/SER', 'SRCLK': 'SHCP/SRCLK', 'RCLK': 'STCP/RCLK', '~{OE}': '!OE', '~{SRCLR}': '!MR/!SRCLR',
          'QA': 'Q0', 'QB': 'Q1', 'QC': 'Q2', 'QD': 'Q3', 'QE': 'Q4', 'QF': 'Q5', 'QG': 'Q6', 'QH': 'Q7', "QH'": "Q7S/QH'"}

def section_d(d, out):
    out.append("=" * 100)
    out.append("(d) OutputBlock.kicad_sch (sub-sheet) full connectivity -- shown for the first instance; all 8 instances are identical")
    out.append("    (references per instance are listed in section (c); net names are given as seen INSIDE the block)")
    out.append("=" * 100)
    first = sorted([s for s in d.sheets if s['path'] == ''], key=lambda s: (s['at'][0], s['at'][1]))[0]
    key = '/' + first['uuid']
    syms = sorted([s for s in d.symbols if s['path'] == key and not s['power']], key=lambda s: refkey(s['ref']))
    # map of hierarchical labels
    def inside_name(node):
        hl = sorted({n.text for n in d.net_nodes(node.id) if n.kind == 'hlabel' and n.path == key})
        pw = sorted({n.power for n in d.net_nodes(node.id) if n.power})
        if pw: return pw[0]
        if hl: return 'HL:' + hl[0]
        pins = [p for p in d.net_pins(node.id) if p.path == key]
        if len(pins) <= 1: return '(unconnected)'
        p = sorted(pins, key=lambda n: (refkey(n.ref), pinkey(n.pin)))[0]
        return f"Net-({p.ref}-{p.pinname})"
    for s in syms:
        if '595' not in s['lib']:
            continue
        out.append("")
        out.append(f"--- {s['ref']} = {s['value']} ({s['lib']}) at {s['at']} mirror={s['mirror']} ---")
        out.append(f"{'Pin#':<5} {'KiCad name':<10} {'Alt name':<12} {'Net inside block':<22} {'Top-level net':<24} Other pins inside block")
        for p in pin_nodes(d, s):
            nm = inside_name(p)
            others = [q for q in d.net_pins(p.id) if q.id != p.id and q.path == key]
            topname = d.netname[d.find(p.id)]
            out.append(f"{p.pin:<5} {p.pinname:<10} {ALT595.get(p.pinname,''):<12} {nm:<22} {topname:<24} {fmt_pins(others, d, with_sheet=False)}")
    out.append("")
    out.append("--- Resistors (pin 1 / pin 2 of Device:R) ---")
    out.append(f"{'Ref':<5} {'Value':<6} {'pin1 net':<14} {'pin2 net':<14} {'595 output driving pin1/pin2':<32} Notes")
    for s in syms:
        if s['lib'] != 'Device:R':
            continue
        p1, p2 = pin_nodes(d, s)
        n1, n2 = inside_name(p1), inside_name(p2)
        drv = []
        for p in (p1, p2):
            for q in d.net_pins(p.id):
                if q.path == key and '595' in (q.lib or ''):
                    drv.append(f"pin{p.pin}<-{q.ref}.{q.pinname}(pin {q.pin})")
        out.append(f"{s['ref']:<5} {s['value']:<6} {n1:<14} {n2:<14} {', '.join(drv):<32}")
    out.append("")
    out.append("--- Capacitors ---")
    for s in syms:
        if s['lib'] != 'Device:C':
            continue
        p1, p2 = pin_nodes(d, s)
        out.append(f"{s['ref']} {s['value']}: pin1={inside_name(p1)}  pin2={inside_name(p2)}")
    out.append("")
    out.append("--- Wires inside OutputBlock (only 4; everything else is connected by hierarchical labels placed directly on pin ends) ---")
    for w in d.wires:
        if w[0] == key:
            out.append(f"  wire ({w[1]},{w[2]}) -> ({w[3]},{w[4]})  net: {inside_name(d.nodes[w[5]]) if True else ''}")
    out.append("")
    out.append("--- Hierarchical labels inside OutputBlock (name: count) ---")
    cnt = defaultdict(int)
    for n in d.nodes:
        if n.kind == 'hlabel' and n.path == key:
            cnt[n.text] += 1
    out.append("  " + ", ".join(f"{k}:{v}" for k, v in sorted(cnt.items(), key=lambda kv: pinkey_name(kv[0]))))
    out.append("")

def section_e(d, out):
    out.append("=" * 100)
    out.append("(e) TOP SHEET: connectors")
    out.append("=" * 100)
    for ref in ('J1', 'J2', 'J3', 'J4', 'J5'):
        s = sym_by_ref(d, ref)
        if not s: continue
        out.append("")
        out.append(f"--- {ref} = {s['value']} ({s['lib']}) at {s['at']} mirror={s['mirror']} ---")
        pins = pin_nodes(d, s)
        rows = []
        for p in pins:
            name = d.netname[d.find(p.id)] if d.connected(p.id) else '*** UNCONNECTED ***'
            rows.append((p.pin, name))
        # compress runs: group by net for GND
        gnd = [r[0] for r in rows if r[1] == 'GND']
        if len(gnd) > 4:
            out.append(f"  pins {compress_numbers(gnd)} -> GND")
        for pin, name in rows:
            if name == 'GND' and len(gnd) > 4:
                continue
            others = [q for q in d.net_pins(pin_by_num(pins, pin).id) if q.ref != ref]
            out.append(f"  pin {pin:<3} -> {name:<26} {fmt_pins(others, d)}")
    out.append("")

def pin_by_num(pins, n):
    return [p for p in pins if p.pin == n][0]

def section_misc(d, out):
    out.append("=" * 100)
    out.append("(extra) TOP SHEET: remaining components (RC networks, power input, battery, mounting holes)")
    out.append("=" * 100)
    for s in sorted([s for s in d.symbols if s['path'] == '' and not s['power'] and s['ref'] not in ('U1','U2','U3','U4','U5','J1','J2','J3','J4','J5')], key=lambda s: refkey(s['ref'])):
        pins = pin_nodes(d, s)
        parts = []
        for p in pins:
            name = d.netname[d.find(p.id)] if d.connected(p.id) else '*** UNCONNECTED ***'
            parts.append(f"pin {p.pin} ({p.pinname}) = {name}")
        out.append(f"{s['ref']:<5} {s['value']:<14} {s['lib']:<32} " + "; ".join(parts))
    out.append("")

def section_nets(d, out):
    out.append("=" * 100)
    out.append("(full) Flattened netlist of the whole design (all 8 OutputBlock instances expanded), nets with >= 2 pins")
    out.append("=" * 100)
    rows = []
    for root, ids in d.nets.items():
        pins = [d.nodes[i] for i in ids if d.nodes[i].kind == 'pin' and not d.nodes[i].ref.startswith('#')]
        if len(pins) < 2:
            continue
        rows.append((d.netname[root], pins))
    rows.sort(key=lambda r: natkey(r[0]))
    for name, pins in rows:
        out.append(f"{name:<34} ({len(pins):3d} pins) {fmt_pins(pins, d)}")
    out.append("")

def natkey(s):
    return [int(t) if t.isdigit() else t.lower() for t in re.split(r'(\d+)', s)]

def section_sanity(d, out):
    out.append("=" * 100)
    out.append("(sanity) Parser self-checks")
    out.append("=" * 100)
    # 1. power pins of 74xx and 595s
    ok = True
    for s in d.symbols:
        if s['power']: continue
        for i in s['pins']:
            p = d.nodes[i]
            if p.etype == 'power_in' and p.pinname in ('VCC', 'GND'):
                name = d.netname[d.find(p.id)]
                expected = {'VCC': '+5V', 'GND': 'GND'}[p.pinname]
                if name != expected:
                    ok = False
                    out.append(f"  POWER CHECK FAIL: {d.sheet_names[s['path']]} {s['ref']} pin {p.pin} ({p.pinname}) is on net '{name}', expected {expected}")
    out.append(f"  Power-pin check (all VCC->+5V, GND->GND on every IC incl. 8x2 595s): {'PASS' if ok else 'FAIL'}")
    # 2. dangling wire ends / labels
    dang = 0
    for n in d.nodes:
        if n.kind == 'wire_end':
            others = [m for m in d.net_nodes(n.id) if m.kind in ('pin', 'label', 'glabel', 'hlabel', 'sheetpin')]
            wires = {m.id for m in d.net_nodes(n.id) if m.kind == 'wire'}
            if not others and len(wires) < 2:
                dang += 1
                out.append(f"  dangling wire end at ({n.x},{n.y}) on {d.sheet_names[n.path]}")
    out.append(f"  Wire ends whose net has no pin/label at all: {dang}")
    for n in d.nodes:
        if n.kind in ('label', 'glabel', 'hlabel'):
            others = [m for m in d.net_nodes(n.id) if m.id != n.id and m.kind in ('pin', 'wire', 'sheetpin')]
            if not others:
                out.append(f"  label '{n.text}' at ({n.x},{n.y}) on {d.sheet_names[n.path]} touches nothing (only other labels)")
    # 3. unconnected pins per symbol
    out.append("")
    out.append("  Unconnected pins per symbol (top sheet and first OutputBlock instance):")
    first = sorted([s for s in d.sheets if s['path'] == ''], key=lambda s: (s['at'][0], s['at'][1]))[0]
    for s in sorted(d.symbols, key=lambda s: (s['path'], refkey(s['ref']))):
        if s['power'] or s['path'] not in ('', '/' + first['uuid']):
            continue
        un = [d.nodes[i] for i in s['pins'] if not d.connected(i)]
        total = len(s['pins'])
        if un:
            out.append(f"    {d.sheet_names[s['path']]:<12} {s['ref']:<5} {s['value']:<22} {len(un)}/{total} unconnected: " + ", ".join(f"{p.pin}({p.pinname})" for p in un))
        else:
            out.append(f"    {d.sheet_names[s['path']]:<12} {s['ref']:<5} {s['value']:<22} all {total} pins connected")
    # 4. hidden pins / notes
    out.append("")
    for l in d.log:
        out.append("  " + l)
    out.append("")

# --------------------------------------------------------------------------- PCB cross-check
def pcb_compare(d, out):
    out.append("=" * 100)
    out.append("(cross-check) Comparison with the pad->net assignments stored in ShiftWS2811_PCB.kicad_pcb (KiCad's own netlist from the last schematic->PCB update)")
    out.append("=" * 100)
    if not os.path.exists(PCB_FILE):
        out.append("  PCB file not found"); return
    root = parse_sexpr(open(PCB_FILE, encoding='utf-8').read())
    pcb = {}   # path -> (ref, {pad: net})
    for fp in kids(root, 'footprint'):
        ref = prop(fp, 'Reference')
        if ref is None:
            for t in kids(fp, 'fp_text'):
                if str(t[1]) == 'reference': ref = t[2]
        pth = kid(fp, 'path'); pth = pth[1] if pth else None
        pads = {}
        for pad in kids(fp, 'pad'):
            net = kid(pad, 'net')
            if net: pads[pad[1]] = net[2]
        pcb[pth] = (ref, pads)
    # schematic: path -> {pin: netroot}
    sch = {}
    for s in d.symbols:
        if s['power']: continue
        key = s['path'] + '/' + s['uuid']
        sch[key] = (s['ref'], {d.nodes[i].pin: d.find(i) for i in s['pins']})
    pcb_paths = set(pcb) ; sch_paths = set(sch)
    out.append(f"  footprints in PCB: {len(pcb_paths)}, non-power symbols in flattened schematic: {len(sch_paths)}")
    for k in sorted(pcb_paths - sch_paths): out.append(f"  PCB footprint {pcb[k][0]} path {k} has no schematic symbol")
    for k in sorted(sch_paths - pcb_paths): out.append(f"  schematic symbol {sch[k][0]} path {k} has no PCB footprint")
    refmis = [(pcb[k][0], sch[k][0]) for k in pcb_paths & sch_paths if pcb[k][0] != sch[k][0]]
    out.append(f"  reference designator mismatches (pcb vs sch): {refmis if refmis else 'none'}")
    # build mapping pcbnet -> set of schematic net roots
    pcbnet_to_sch = defaultdict(set); schnet_to_pcb = defaultdict(set)
    pads_checked = 0; missing = []
    for k in pcb_paths & sch_paths:
        ref, pads = pcb[k]; _, spins = sch[k]
        for pad, net in pads.items():
            if pad not in spins:
                missing.append(f"{ref} pad {pad}"); continue
            pads_checked += 1
            pcbnet_to_sch[net].add(spins[pad]); schnet_to_pcb[spins[pad]].add(net)
        for pin in spins:
            if pin not in pads:
                missing.append(f"{ref} sch pin {pin} has no PCB pad")
    out.append(f"  pads compared: {pads_checked}; pads/pins without counterpart: {missing if missing else 'none'}")
    bad = 0
    for net, roots in sorted(pcbnet_to_sch.items()):
        if net == '' : continue
        if len(roots) > 1:
            bad += 1
            out.append(f"  MISMATCH: PCB net '{net}' is split over schematic nets: {[d.netname[r] for r in roots]}")
    for r, nets in sorted(schnet_to_pcb.items(), key=lambda kv: d.netname[kv[0]]):
        nets2 = {n for n in nets if n != ''}
        if len(nets2) > 1:
            bad += 1
            out.append(f"  MISMATCH: schematic net '{d.netname[r]}' spans PCB nets: {sorted(nets2)}")
    # name comparison (informational)
    diffnames = []
    for net, roots in pcbnet_to_sch.items():
        if net == '' or len(roots) != 1: continue
        mine = d.netname[next(iter(roots))]
        if mine != net:
            diffnames.append(f"{net} ~ {mine}")
    out.append(f"  Partition mismatches: {bad}  -> {'IDENTICAL connectivity (every PCB net = exactly one schematic net and vice versa)' if bad == 0 else 'see above'}")
    out.append(f"  Net-name differences (same connectivity, only naming): {len(diffnames)}" + (": " + "; ".join(sorted(diffnames)[:40]) if diffnames else ""))
    out.append("")

# --------------------------------------------------------------------------- main
def main():
    d = Design()
    d.load_sheet(TOP_FILE, '', None, '<top>')
    d.finish()
    out = []
    out.append(f"KiCad schematic netlist derived from {TOP_FILE}")
    out.append(f"sheets loaded: " + ", ".join(f"{k or '/'}={v}" for k, v in d.sheet_names.items()))
    out.append(f"nodes: {len(d.nodes)}  symbols: {len(d.symbols)}  wires: {len(d.wires)}  nets: {len(d.nets)}")
    out.append("")
    section_a(d, out)
    section_b(d, out)
    section_c(d, out)
    section_d(d, out)
    section_e(d, out)
    section_misc(d, out)
    section_sanity(d, out)
    pcb_compare(d, out)
    section_nets(d, out)
    text = "\n".join(out)
    print(text)
    with open(os.path.join(OUT_DIR, 'netlist_report.txt'), 'w') as f:
        f.write(text + "\n")
    # also split files
    parts = re.split(r'\n(?==+\n\()', text)
    for part in parts[1:]:
        m = re.match(r'=+\n\((\w+)\)', part)
        if m:
            with open(os.path.join(OUT_DIR, f'section_{m.group(1)}.txt'), 'w') as f:
                f.write(part + "\n")

if __name__ == '__main__':
    main()
