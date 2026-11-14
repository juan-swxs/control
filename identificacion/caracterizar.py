"""Caracterización de la planta (motor + push-pull + piñón-cremallera con resortes).

Usa el firmware de main/ (comandos OPEN <V>, STOP, RUN, REF <mm>; DATA cada 20 ms).
Guarda en datos/:
  reposo.csv      sensor con el motor parado (deriva y ruido)
  estatica.csv    escalera lenta de voltaje: equilibrio x(V) y zona muerta, ida y vuelta
  escalones.csv   escalones de voltaje desde el reposo, ambos sentidos (para tfest)
Uso: python caracterizar.py [COM10]
"""
import csv
import os
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM10'
X_MAX, X_MIN = 9.0, -5.8          # recorrido seguro medido (mm)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'datos')
os.makedirs(OUT, exist_ok=True)

s = serial.Serial()
s.port, s.baudrate, s.timeout = PORT, 115200, 0.01
s.dtr = False
s.rts = False
s.open()
buf = b''
rows = []
last = {}
tag = 'inicio'


def cmd(c):
    s.write((c + '\n').encode())


def pump(t, guard=True):
    """Lee telemetría t segundos; si guard y se sale del recorrido, para el motor."""
    global buf
    end = time.time() + t
    while time.time() < end:
        buf += s.read(4096)
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            line = line.decode('utf-8', 'replace').strip()
            if not line.startswith('DATA'):
                if line.startswith('ERR'):
                    print('  ', line)
                continue
            f = line.split(',')
            try:
                d = dict(t=float(f[1]) / 1000, x=float(f[2]), v=float(f[3]), r=float(f[4]),
                         vm=float(f[5]), raw=int(f[6]), modo=int(f[11]))
            except (ValueError, IndexError):
                continue
            d['prueba'] = tag
            rows.append(d)
            last.update(d)
            if guard and d['modo'] == 2 and (d['x'] > X_MAX or d['x'] < X_MIN):
                cmd('STOP')
                print(f'   limite: x={d["x"]:.2f}, STOP')
                return False
    return True


def save(name, data):
    with open(os.path.join(OUT, name), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=['prueba', 't', 'x', 'v', 'r', 'vm', 'raw', 'modo'])
        w.writeheader()
        w.writerows(data)
    print(f'  -> datos/{name} ({len(data)} muestras)')


def centrar():
    global tag
    tag = 'centrar'
    cmd('RUN')
    pump(0.1)
    cmd('REF 0')
    pump(2.5, False)
    cmd('STOP')
    pump(0.8, False)


pump(0.5, False)

# 1. Reposo
print('1) reposo')
centrar()
tag = 'reposo'
n0 = len(rows)
pump(3, False)
save('reposo.csv', rows[n0:])
xs = [r['x'] for r in rows[n0:]]
print(f'   x reposo: media {sum(xs)/len(xs):.3f} mm, rango {max(xs)-min(xs):.3f} mm')

# 2. Escalera estática, cada lado: sube hasta 7.5 V (o hasta el límite) y baja
print('2) escalera estatica')
n0 = len(rows)
niveles = [round(0.25 * k, 2) for k in range(0, 31)]       # 0 .. 7.5 V
for lado in (+1, -1):
    centrar()
    tope = len(niveles) - 1
    for k, V in enumerate(niveles):
        tag = f'est{lado:+d}_sube_{V:.2f}'
        cmd(f'OPEN {lado*V:.2f}')
        ok = pump(1.2)
        print(f'   {lado*V:+5.2f} V sube: x={last["x"]:6.2f}')
        if not ok:
            tope = k - 1
            break
    for V in niveles[tope::-1]:
        tag = f'est{lado:+d}_baja_{V:.2f}'
        cmd(f'OPEN {lado*V:.2f}')
        pump(1.2, False)
        print(f'   {lado*V:+5.2f} V baja: x={last["x"]:6.2f}')
cmd('STOP')
pump(0.3, False)
save('estatica.csv', rows[n0:])

# 3. Escalones desde el reposo
print('3) escalones')
n0 = len(rows)
for V in (3, -3, 4, -4, 5, -5, 6, -6, 7, -7):
    centrar()
    tag = f'esc{V:+d}'
    pump(0.3, False)                    # muestras previas con u = 0
    cmd(f'OPEN {V}')
    pump(1.5)
    cmd('STOP')
    pump(0.6, False)
    seg = [r for r in rows if r['prueba'] == tag]
    print(f'   {V:+d} V: x {seg[0]["x"]:.2f} -> max {max(r["x"] for r in seg):.2f}'
          f' / min {min(r["x"] for r in seg):.2f}')
save('escalones.csv', rows[n0:])

centrar()
cmd('RUN')
pump(0.1, False)
cmd('REF 0')
pump(2, False)
print(f'fin: x={last["x"]:.2f} mm (lazo cerrado en 0)')
s.close()
