"""Genera el esquemático de KiCad para mover el motor en ambos sentidos con el
ESP32: DAC (0-3.3 V) y Vref = 1.65 V -> amplificador diferencial -> push-pull.

    python generar_esquematico.py      -> push_pull_motor.kicad_sch / .kicad_pro

    Vm = G * (Vdac - Vref),  G = R4/R3 = R6/R5 = 47k/10k = 4.7
    DAC 0 V -> -7.8 V   |   DAC 1.65 V -> 0 V (parado)   |   DAC 3.3 V -> +7.8 V

Asignación del LM324 (U1):  A: diferencial   B: buffer de Vref   C, D: sin uso
"""
import math
import os
import sys
import uuid

sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from sexp import dump, find, get, lib_symbol  # noqa: E402

LIBDIR = r'C:/Users/PERSONAL/AppData/Local/Programs/KiCad/10.0/share/kicad/symbols/'
PROJECT = 'push_pull_motor'
ROOT_UUID = str(uuid.uuid5(uuid.NAMESPACE_URL, PROJECT))
uid = lambda: '"%s"' % uuid.uuid4()
q = lambda s: '"%s"' % s
num = lambda v: ('%.4f' % v).rstrip('0').rstrip('.')

G_TXT = '4.7'       # ganancia del diferencial (R4/R3)
RG = '47k'          # R4 y R6
RIN = '10k'         # R3 y R5

# ------------------------------------------------------------------ símbolos
LIBS = {}       # lib_id -> definición aplanada
PINS = {}       # lib_id -> {numero: (unidad, x, y)}


def usar(lib, name):
    lib_id = '%s:%s' % (lib, name)
    if lib_id in LIBS:
        return lib_id
    s = lib_symbol(LIBDIR + lib + '.kicad_sym', name)
    ext = get(s, 'extends')
    if ext:     # aplanar: unidades del padre + propiedades del hijo
        parent = ext[1].strip('"')
        base = lib_symbol(LIBDIR + lib + '.kicad_sym', parent)
        props = {p[1]: p for p in find(s, 'property')}
        flat = []
        for e in base:
            if isinstance(e, list) and e[0] == 'property' and e[1] in props:
                flat.append(props[e[1]])
            elif isinstance(e, list) and e[0] == 'symbol':
                sub = list(e)
                sub[1] = sub[1].replace(parent, name)
                flat.append(sub)
            else:
                flat.append(e)
        s = flat
    s = list(s)
    s[1] = q(lib_id)
    LIBS[lib_id] = s
    pins = {}
    for sub in find(s, 'symbol'):
        unit = int(sub[1].strip('"').split('_')[-2])
        for p in find(sub, 'pin'):
            at = get(p, 'at')
            pins[get(p, 'number')[1].strip('"')] = (unit, float(at[1]), float(at[2]))
    PINS[lib_id] = pins
    return lib_id


ITEMS = []


def pos_pin(lib_id, numero, x, y, rot=0, mirror=None):
    _, px, py = PINS[lib_id][numero]
    if mirror == 'x':
        py = -py
    elif mirror == 'y':
        px = -px
    a = math.radians(rot)
    px, py = (px * math.cos(a) - py * math.sin(a), px * math.sin(a) + py * math.cos(a))
    return (round(x + px, 4), round(y - py, 4))


def prop(nombre, valor, x, y, hide=False, size=1.27, justify=None, ang=0):
    eff = ['effects', ['font', ['size', num(size), num(size)]]]
    if justify:
        eff.append(['justify', justify])
    p = ['property', q(nombre), q(valor), ['at', num(x), num(y), str(ang)]]
    if hide:
        p.append(['hide', 'yes'])
    p.append(eff)
    return p


def simbolo(lib, name, ref, valor, x, y, unit=1, rot=0, mirror=None,
            ref_at=None, val_at=None, power=False, desc='', ref_just=None, val_just=None):
    lib_id = usar(lib, name)
    if rot in (90, 270):        # KiCad voltea el texto de los campos: se invierte la justificación
        swap = {'left': 'right', 'right': 'left', None: None}
        ref_just, val_just = swap[ref_just], swap[val_just]
    rx, ry = ref_at if ref_at else (x, y - 5.08)
    vx, vy = val_at if val_at else (x, y + 5.08)
    at = ['at', num(x), num(y), str(rot)]
    s = ['symbol', ['lib_id', q(lib_id)], at]
    if mirror:
        s.append(['mirror', mirror])
    s += [['unit', str(unit)], ['exclude_from_sim', 'no'], ['in_bom', 'no' if power else 'yes'],
          ['on_board', 'no' if power else 'yes'], ['dnp', 'no'], ['uuid', uid()],
          prop('Reference', ref, rx, ry, hide=power, justify=ref_just, ang=rot),
          prop('Value', valor, vx, vy, justify=val_just, ang=rot),
          prop('Footprint', '', x, y, hide=True),
          prop('Datasheet', '', x, y, hide=True),
          prop('Description', desc, x, y, hide=True)]
    for n, (u, _, _) in PINS[lib_id].items():
        if u in (0, unit):
            s.append(['pin', q(n), ['uuid', uid()]])
    s.append(['instances', ['project', q(PROJECT),
              ['path', q('/' + ROOT_UUID), ['reference', q(ref)], ['unit', str(unit)]]]])
    ITEMS.append(s)
    return {n: pos_pin(lib_id, n, x, y, rot, mirror)
            for n, (u, _, _) in PINS[lib_id].items() if u in (0, unit)}


_pwr = [0]


def potencia(name, x, y):
    _pwr[0] += 1
    val_at = (x, y - 3.81) if name in ('+12V', '+3V3', 'PWR_FLAG') else (x, y + 5.08)
    ref = '#FLG%02d' % _pwr[0] if name == 'PWR_FLAG' else '#PWR%02d' % _pwr[0]
    return simbolo('power', name, ref, name, x, y, power=True, val_at=val_at)


def cable(*pts):
    for a, b in zip(pts, pts[1:]):
        if a == b:
            continue
        ITEMS.append(['wire', ['pts', ['xy', num(a[0]), num(a[1])], ['xy', num(b[0]), num(b[1])]],
                      ['stroke', ['width', '0'], ['type', 'default']], ['uuid', uid()]])


def union(x, y):
    ITEMS.append(['junction', ['at', num(x), num(y)], ['diameter', '0'],
                  ['color', '0', '0', '0', '0'], ['uuid', uid()]])


def etiqueta(nombre, x, y, lado='der'):
    ang, just = ('0', 'left') if lado == 'der' else ('180', 'right')
    ITEMS.append(['label', q(nombre), ['at', num(x), num(y), ang],
                  ['effects', ['font', ['size', '1.524', '1.524'], 'bold'], ['justify', just, 'bottom']],
                  ['uuid', uid()]])


def texto(t, x, y, size=1.8, bold=True):
    font = ['font', ['size', num(size), num(size)]]
    if bold:
        font.append('bold')
    ITEMS.append(['text', q(t.replace('"', '\\"')), ['exclude_from_sim', 'no'],
                  ['at', num(x), num(y), '0'],
                  ['effects', font, ['justify', 'left', 'bottom']], ['uuid', uid()]])


def R_h(ref, valor, xc, y, lado='arriba'):
    """Resistencia horizontal centrada en (xc, y): pines en xc-3.81 / xc+3.81."""
    if lado == 'apilado':
        return simbolo('Device', 'R', ref, valor, xc, y, rot=90,
                       ref_at=(xc, y - 2.0), val_at=(xc, y + 2.9))
    dy, dx = (-2.0, 0) if lado == 'arriba' else (2.9, -1.6)
    return simbolo('Device', 'R', ref, valor, xc, y, rot=90,
                   ref_at=(xc - 1.2 + dx, y + dy), val_at=(xc + 0.4 + dx, y + dy),
                   ref_just='right', val_just='left')


def R_v(ref, valor, x, yc):
    """Resistencia vertical centrada en (x, yc): pines en yc-3.81 / yc+3.81."""
    return simbolo('Device', 'R', ref, valor, x, yc, ref_at=(x + 1.8, yc - 1.27),
                   val_at=(x + 1.8, yc + 1.27), ref_just='left', val_just='left')


def opamp(ref, unit, X, Y, invertido=True):
    """LM324; invertido=True: entrada (-) arriba (mirror x)."""
    p = simbolo('Amplifier_Operational', 'LM324', ref, 'LM324', X, Y, unit=unit,
                mirror='x' if invertido else None,
                ref_at=(X + 2.54, Y - 6.35), val_at=(X + 2.54, Y + 6.35))
    lib_id = 'Amplifier_Operational:LM324'
    names = {}
    for n, c in p.items():
        if PINS[lib_id][n][1] > 0:
            names['out'] = c
    entradas = sorted([c for c in p.values() if c != names['out']], key=lambda c: c[1])
    arriba, abajo = entradas
    if invertido:
        names['-'], names['+'] = arriba, abajo
    else:
        names['+'], names['-'] = arriba, abajo
    return names


# ------------------------------------------------------------------ circuito
def construir():
    # ===== (a) Vref = 1.65 V: divisor + buffer U1B =====
    X, Y = 66.04, 63.5
    texto('(a) Vref = 1.65 V  -  divisor + seguidor U1B (LM324)', X - 40, Y - 24)
    texto('Sin el seguidor, los 5k del divisor se suman a R3 y el punto de parada se corre a ~1.52 V',
          X - 40, Y - 20.5, size=1.3, bold=False)
    texto('Alimentar el divisor del MISMO 3.3 V que el MCP4725: si ese voltaje cambia, el cero no se mueve',
          X - 40, Y - 18, size=1.3, bold=False)
    o = opamp('U1', 2, X, Y, invertido=False)
    dx = X - 22.86
    R_v('R1', '10k', dx, Y - 6.35)
    R_v('R2', '10k', dx, Y + 1.27)
    cable((dx, Y - 10.16), (dx, Y - 12.7)); potencia('+3V3', dx, Y - 12.7)
    cable((dx, Y + 5.08), (dx, Y + 7.62)); potencia('GND', dx, Y + 7.62)
    cable((dx, Y - 2.54), o['+']); union(dx, Y - 2.54)
    cable(o['-'], (X - 10.16, Y + 2.54), (X - 10.16, Y + 7.62), (X + 10.16, Y + 7.62), (X + 10.16, Y))
    cable(o['out'], (X + 10.16, Y), (X + 17.78, Y)); union(X + 10.16, Y)
    etiqueta('VREF', X + 17.78, Y)

    # ===== (b) Diferencial U1A: Vm = G (DAC - VREF), realimentado desde los emisores =====
    X, Y = 175.26, 63.5
    texto('(b) Amplificador diferencial  -  U1A (LM324)', X - 40, Y - 24)
    texto('Vm = %s * (DAC - VREF)   (R4/R3 = R6/R5 = %s/%s)' % (G_TXT, RG, RIN),
          X - 40, Y - 20.5, size=1.3, bold=False)
    texto('R4 va a los EMISORES (VM), no a la salida del op-amp: compensa la caída de los transistores',
          X - 40, Y - 18, size=1.3, bold=False)
    o = opamp('U1', 1, X, Y)
    xn = X - 10.16
    R_h('R3', RIN, X - 17.78, Y - 2.54)
    cable((X - 13.97, Y - 2.54), (xn, Y - 2.54), o['-'])
    cable((X - 21.59, Y - 2.54), (X - 27.94, Y - 2.54)); etiqueta('VREF', X - 27.94, Y - 2.54, 'izq')
    R_h('R4', RG, X, Y - 10.16)
    cable((xn, Y - 2.54), (xn, Y - 10.16), (X - 3.81, Y - 10.16)); union(xn, Y - 2.54)
    cable((X + 3.81, Y - 10.16), (X + 20.32, Y - 10.16)); etiqueta('VM', X + 20.32, Y - 10.16)
    R_h('R5', RIN, X - 17.78, Y + 2.54, lado='abajo')
    cable((X - 13.97, Y + 2.54), (xn, Y + 2.54), o['+'])
    cable((X - 21.59, Y + 2.54), (X - 27.94, Y + 2.54)); etiqueta('DAC', X - 27.94, Y + 2.54, 'izq')
    R_v('R6', RG, xn, Y + 8.89)
    cable((xn, Y + 2.54), (xn, Y + 5.08)); union(xn, Y + 2.54)
    potencia('GND', xn, Y + 12.7)
    cable(o['out'], (X + 20.32, Y)); etiqueta('OA_OUT', X + 20.32, Y)

    # ===== (c) Push-pull TIP120 / TIP125 + motor =====
    X, Y = 45.72, 148.59
    texto('(c) Push-pull  -  TIP120 / TIP125 a +-12 V', X - 20, Y - 30)
    texto('Motor entre los emisores (VM) y GND: Vm > 0 gira hacia un lado, Vm < 0 hacia el otro',
          X - 20, Y - 26.5, size=1.3, bold=False)
    etiqueta('OA_OUT', X, Y, 'izq')
    R_h('R7', '1k', X + 7.62, Y)
    cable((X, Y), (X + 3.81, Y))
    BX = X + 17.78
    cable((X + 11.43, Y), (BX, Y))
    q1 = simbolo('Transistor_BJT', 'TIP120', 'Q1', 'TIP120', BX + 5.08, Y - 7.62,
                 ref_at=(BX + 12.7, Y - 8.89), val_at=(BX + 12.7, Y - 6.35))
    q2 = simbolo('Transistor_BJT', 'TIP125', 'Q2', 'TIP125', BX + 5.08, Y + 7.62, mirror='x',
                 ref_at=(BX + 12.7, Y + 6.35), val_at=(BX + 12.7, Y + 8.89))
    cable(q1['1'], (BX, Y - 7.62), (BX, Y), (BX, Y + 7.62), q2['1']); union(BX, Y)
    cable(q1['2'], (BX + 7.62, Y - 15.24)); potencia('+12V', BX + 7.62, Y - 15.24)
    cable(q2['2'], (BX + 7.62, Y + 15.24)); potencia('-12V', BX + 7.62, Y + 15.24)
    EX = BX + 7.62
    cable(q1['3'], (EX, Y), q2['3'])
    DX, MX = EX + 15.24, EX + 30.48
    cable((EX, Y), (EX + 7.62, Y), (DX, Y), (MX, Y))
    union(EX, Y); union(EX + 7.62, Y); union(DX, Y)
    cable((EX + 7.62, Y), (EX + 7.62, Y + 17.78)); etiqueta('VM', EX + 7.62, Y + 17.78)
    simbolo('Device', 'D', 'D1', '1N4007', DX, Y - 7.62, rot=270,
            ref_at=(DX + 2.5, Y - 8.89), val_at=(DX + 2.5, Y - 6.35), ref_just='left', val_just='left')
    simbolo('Device', 'D', 'D2', '1N4007', DX, Y + 7.62, rot=270,
            ref_at=(DX + 2.5, Y + 6.35), val_at=(DX + 2.5, Y + 8.89), ref_just='left', val_just='left')
    cable((DX, Y - 3.81), (DX, Y)); cable((DX, Y), (DX, Y + 3.81))
    cable((DX, Y - 11.43), (DX, Y - 15.24)); potencia('+12V', DX, Y - 15.24)
    cable((DX, Y + 11.43), (DX, Y + 15.24)); potencia('-12V', DX, Y + 15.24)
    m = simbolo('Motor', 'Motor_DC', 'M1', 'Motor DC', MX, Y + 5.08,
                ref_at=(MX + 4.0, Y + 3.81), val_at=(MX + 4.0, Y + 6.35), ref_just='left', val_just='left')
    cable(m['2'], (MX, Y + 15.24)); potencia('GND', MX, Y + 15.24)

    # ===== (d) U1C y U1D sin uso: seguidores con (+) a GND =====
    for k, unit in enumerate((3, 4)):
        X, Y = 180.34 + 38.1 * k, 132.08
        if k == 0:
            texto('U1C y U1D (LM324) sin uso: seguidores con (+) a GND', X - 15, Y - 12, size=1.3, bold=False)
        o = opamp('U1', unit, X, Y, invertido=False)
        cable(o['+'], (X - 10.16, Y - 2.54), (X - 10.16, Y + 7.62)); potencia('GND', X - 10.16, Y + 7.62)
        cable(o['-'], (X - 7.62, Y + 5.08), (X + 10.16, Y + 5.08), (X + 10.16, Y), o['out'])

    # ===== (e) Alimentación del LM324 + desacople =====
    X, Y = 185.42, 175.26
    texto('Alimentación del LM324 (+-12 V), desacople de 100 nF junto al integrado', X - 20, Y - 20, size=1.3)
    p = simbolo('Amplifier_Operational', 'LM324', 'U1', 'LM324', X, Y, unit=5,
                ref_at=(X + 2.54, Y - 1.27), val_at=(X + 2.54, Y + 1.27))
    vmas = min(p.values(), key=lambda c: c[1])
    vmenos = max(p.values(), key=lambda c: c[1])
    cable(vmas, (vmas[0], Y - 12.7)); potencia('+12V', vmas[0], Y - 12.7)
    cable(vmenos, (vmenos[0], Y + 12.7)); potencia('-12V', vmenos[0], Y + 12.7)
    cx = X + 12.7
    simbolo('Device', 'C', 'C1', '100nF', cx, Y - 7.62, ref_at=(cx + 4.5, Y - 8.89), val_at=(cx + 4.5, Y - 6.35))
    simbolo('Device', 'C', 'C2', '100nF', cx, Y + 7.62, ref_at=(cx + 4.5, Y + 6.35), val_at=(cx + 4.5, Y + 8.89))
    cable((vmas[0], Y - 12.7), (cx, Y - 12.7), (cx, Y - 11.43)); union(vmas[0], Y - 12.7)
    cable((vmenos[0], Y + 12.7), (cx, Y + 12.7), (cx, Y + 11.43)); union(vmenos[0], Y + 12.7)
    cable((cx, Y - 3.81), (cx, Y), (cx, Y + 3.81)); cable((cx, Y), (cx + 15.24, Y))
    potencia('GND', cx + 15.24, Y); union(cx, Y)

    # ===== Conectores =====
    X, Y = 330.2, 40.64
    texto('Conectores', X - 25, Y - 8)
    j = simbolo('Connector_Generic', 'Conn_01x03', 'J1', 'Fuente +12 / GND / -12', X, Y + 5.08,
                ref_at=(X + 4.0, Y + 2.54), val_at=(X + 4.0, Y + 5.08), ref_just='left', val_just='left')
    cable(j['1'], (X - 10.16, Y + 2.54), (X - 10.16, Y - 2.54)); potencia('+12V', X - 10.16, Y - 2.54)
    cable((X - 10.16, Y - 2.54), (X - 17.78, Y - 2.54)); potencia('PWR_FLAG', X - 17.78, Y - 2.54)
    union(X - 10.16, Y - 2.54)
    cable(j['3'], (X - 10.16, Y + 7.62), (X - 10.16, Y + 12.7), (X - 10.16, Y + 15.24)); potencia('-12V', X - 10.16, Y + 15.24)
    cable((X - 10.16, Y + 12.7), (X - 15.24, Y + 12.7)); potencia('PWR_FLAG', X - 15.24, Y + 12.7)
    union(X - 10.16, Y + 12.7)
    cable(j['2'], (X - 22.86, Y + 5.08), (X - 22.86, Y + 12.7), (X - 22.86, Y + 15.24)); potencia('GND', X - 22.86, Y + 15.24)
    cable((X - 22.86, Y + 12.7), (X - 27.94, Y + 12.7)); potencia('PWR_FLAG', X - 27.94, Y + 12.7)
    union(X - 22.86, Y + 12.7)

    Y = 78.74
    j = simbolo('Connector_Generic', 'Conn_01x03', 'J2', 'ESP32 / MCP4725 (3V3, OUT, GND)', X, Y + 2.54,
                ref_at=(X + 4.0, Y + 1.27), val_at=(X + 4.0, Y + 3.81), ref_just='left', val_just='left')
    pins = sorted(j.items(), key=lambda kv: int(kv[0]))
    for (n, c), net in zip(pins, ['+3V3', 'DAC', 'GND']):
        if net == '+3V3':
            cable(c, (X - 12.7, c[1]))
            cable((X - 12.7, c[1]), (X - 12.7, c[1] - 5.08)); potencia('+3V3', X - 12.7, c[1] - 5.08)
            cable((X - 12.7, c[1]), (X - 20.32, c[1]), (X - 20.32, c[1] - 5.08))
            potencia('PWR_FLAG', X - 20.32, c[1] - 5.08); union(X - 12.7, c[1])
        elif net == 'GND':
            cable(c, (X - 12.7, c[1]), (X - 12.7, c[1] + 5.08)); potencia('GND', X - 12.7, c[1] + 5.08)
        else:
            cable(c, (X - 15.24, c[1])); etiqueta(net, X - 15.24, c[1], 'izq')

    # ===== Notas =====
    x0, y0 = 255, 130
    texto('Cómo funciona', x0, y0, size=1.6)
    for k, t in enumerate([
            'Vm = %s * (DAC - 1.65 V)' % G_TXT,
            'DAC = 0 V     ->  Vm = -7.8 V  (gira en un sentido)',
            'DAC = 1.65 V  ->  Vm =  0 V    (motor parado)',
            'DAC = 3.3 V   ->  Vm = +7.8 V  (gira en el otro sentido)',
            '',
            'Firmware (MCP4725, 12 bits):  raw = 2048 + Vm / %s * 4096 / 3.3' % G_TXT,
            'IMPORTANTE: grabar la EEPROM del MCP4725 en 2048, no en 0.',
            'Con 0 V el motor arranca a -7.8 V al encender, antes del firmware.',
            '',
            'Más voltaje: R4 = R6 = 56k da +-9.2 V (el LM324 debe llegar a ~10.6 V).',
            'GND del ESP32 = GND de la fuente +-12 V (tierra común).',
            'D1, D2: protegen los transistores de los picos del motor.']):
        texto(t, x0, y0 + 5 + 4.5 * k, size=1.3, bold=False)


def escribir(carpeta):
    construir()
    sch = ['kicad_sch', ['version', '20250610'], ['generator', '"eeschema"'],
           ['generator_version', '"10.0"'], ['uuid', q(ROOT_UUID)], ['paper', '"A3"'],
           ['title_block', ['title', q('Motor en ambos sentidos: DAC + Vref -> diferencial -> push-pull')],
            ['date', q('2026-11-13')], ['rev', '"1"'],
            ['comment', '1', q('Vm = %s (DAC - 1.65 V),  DAC = MCP4725 del ESP32' % G_TXT)],
            ['comment', '2', q('Para identificar la función de transferencia de la planta')]],
           ['lib_symbols'] + list(LIBS.values())]
    sch += ITEMS
    sch += [['sheet_instances', ['path', '"/"', ['page', '"1"']]], ['embedded_fonts', 'no']]
    with open(os.path.join(carpeta, PROJECT + '.kicad_sch'), 'w', encoding='utf-8') as f:
        f.write(dump(sch) + '\n')
    pro = os.path.join(carpeta, PROJECT + '.kicad_pro')
    if not os.path.exists(pro):
        with open(pro, 'w', encoding='utf-8') as f:
            f.write('{\n  "meta": {\n    "filename": "%s.kicad_pro",\n    "version": 3\n  }\n}\n' % PROJECT)


if __name__ == '__main__':
    escribir(os.path.dirname(os.path.abspath(__file__)))
    print('Esquemático generado:', PROJECT + '.kicad_sch')
