"""Cálculos reproducibles del laboratorio 4 (Python + NumPy; SciPy opcional).

Ejecutar: python estados/calculos_lab4.py
Lee únicamente identificacion/modelo.json y la referencia; escribe solo el .tex.
La simulación exacta usa los modos continuos de A; no usa MATLAB.
"""
from pathlib import Path
import json
import numpy as np

BASE = Path(__file__).resolve().parent
datos = json.loads((BASE.parent / 'identificacion/modelo.json').read_text(encoding='utf-8'))
# Precisión de diseño solicitada; a0 del JSON se denomina a2 en el documento.
b, a1, a2 = round(datos['b0'], 2), round(datos['a1'], 3), round(datos['a0'], 3)
A = np.array([[0., 1.], [-a2, -a1]])
B = np.array([[0.], [b]])
C = np.array([[1., 0.]])
Tmuestreo, zeta, tsd = .01, .8, .5
wn = 4 / (zeta * tsd)
plejano = 10 * zeta * wn
dominante = np.array([1., 2*zeta*wn, wn**2])
tso = tsd / 10
wno = 4 / (zeta*tso)
po = np.array([1., 2*zeta*wno, wno**2])


def controlabilidad(a, bmat):
    return np.column_stack([np.linalg.matrix_power(a, k) @ bmat for k in range(len(a))])


def phi(a, coef):
    resultado = np.zeros_like(a)
    for c in coef:
        resultado = resultado @ a + c*np.eye(len(a))
    return resultado


def matriz_m(coef):
    n = len(coef)-1
    return np.array([[coef[n-1-i-j] if i+j < n else 0.
                      for j in range(n)] for i in range(n)])


def f(x, cifras=7):
    if abs(x) < 1e-9:
        return '0'
    texto = format(float(x), f'.{cifras}g')
    if 'e' in texto:
        mantisa, exponente = texto.split('e')
        return mantisa + r'\times10^{' + str(int(exponente)) + '}'
    return texto


def mat(x):
    x = np.atleast_2d(x)
    return r'\begin{bmatrix}' + r' \\ '.join(' & '.join(f(v) for v in fila) for fila in x) + r'\end{bmatrix}'


def pol(coef, variable='s'):
    n = len(coef)-1
    partes = []
    for i, v in enumerate(coef):
        if abs(v) < 1e-9:
            continue
        grado = n-i
        potencia = (variable if grado == 1 else variable+'^{'+str(grado)+'}') if grado else ''
        termino = ('' if grado and v == 1 else f(v)) + potencia
        partes.append(termino)
    return ' + '.join(partes)


def gather(*lineas):
    return '\\begin{gather*}\n' + '\\\\\n'.join(lineas) + '\n\\end{gather*}\n'


# Respuesta exacta al escalón unitario sobre una malla de 10 microsegundos.
# x(t)=xeq-exp(A t)xeq; diagonalización para evaluar todos los instantes.
lam, vect = np.linalg.eig(A)
xeq = -np.linalg.solve(A, B).ravel()
pesos = (C @ vect).ravel() * np.linalg.solve(vect, xeq)
kdc = b/a2
tiempos = np.linspace(0., 2., 200001)
respuesta = kdc - np.real(np.exp(tiempos[:, None]*lam) @ pesos)
try:
    from scipy.signal import StateSpace, step
    _, respuesta_scipy = step(StateSpace(A, B, C, np.zeros((1, 1))), T=tiempos)
    np.testing.assert_allclose(respuesta, respuesta_scipy, atol=1e-10)
    motor_simulacion = 'NumPy y scipy.signal.step'
except ImportError:
    motor_simulacion = 'NumPy: simulación modal exacta (SciPy no disponible)'
fuera = np.flatnonzero(np.abs(respuesta-kdc) > .02*abs(kdc))
assert fuera.size and fuera[-1]+1 < len(tiempos)
lo, hi = tiempos[fuera[-1]], tiempos[fuera[-1]+1]
def margen(t):
    return abs(np.real(np.exp(t*lam) @ pesos)) - .02*abs(kdc)
for _ in range(55):
    medio = (lo+hi)/2
    if margen(medio) > 0:
        lo = medio
    else:
        hi = medio
tsla = hi
ts_aprox = 4 / abs(lam[0].real)
assert np.max(np.abs(respuesta[fuera[-1]+1:]-kdc)) <= .02*abs(kdc)

S = controlabilidad(A, B)
O = np.vstack((C, C@A))
assert np.linalg.matrix_rank(S) == np.linalg.matrix_rank(O) == 2
resultados = []
for m in (1, 2, 3):
    n = m+2
    ah = np.zeros((n, n)); ah[:2, :2] = A
    for i in range(2, n-1):
        ah[i, i+1] = 1
    ah[-1, :2] = -C.ravel()
    bh = np.zeros((n, 1)); bh[:2] = B
    sh = controlabilidad(ah, bh)
    pd = dominante.copy()
    for _ in range(m):
        pd = np.convolve(pd, [1., plejano])
    pa = np.r_[1., a1, a2, np.zeros(m)]
    kh = np.r_[(pd[2]-a2)/b, (pd[1]-a1)/b, -pd[3:][::-1]/b]
    ph = phi(ah, pd)
    ultima = np.eye(n)[-1]
    fila = np.linalg.solve(sh.T, ultima)
    kack = fila @ ph
    mh = matriz_m(pa)
    trans = sh @ mh
    delta = (pd[1:]-pa[1:])[::-1]
    ktrans = np.linalg.solve(trans.T, delta)
    np.testing.assert_allclose(kh, kack, rtol=1e-10, atol=1e-9)
    np.testing.assert_allclose(kh, ktrans, rtol=1e-10, atol=1e-9)
    acl = ah-bh@kh[None, :]
    eig = np.linalg.eigvals(acl)
    np.testing.assert_allclose(np.poly(acl), pd, rtol=1e-9, atol=1e-6)
    objetivos = np.r_[np.roots(dominante), np.full(m, -plejano)]
    # Los polos repetidos son sensibles al redondeo del algoritmo de autovalores.
    from itertools import permutations
    error_polos = min(max(abs(eig-np.array(p))) for p in permutations(objetivos))
    assert error_polos < .01, (m, eig)
    resultados.append(dict(m=m, ah=ah, bh=bh, sh=sh, pd=pd, pa=pa, kh=kh,
                           ph=ph, fila=fila, mh=mh, trans=trans, delta=delta,
                           eig=eig, error_polos=error_polos))

L = np.array([po[1]-a1, po[2]-a2-a1*(po[1]-a1)])[:, None]
mo = matriz_m([1., a1, a2])
Q = np.linalg.inv(mo@O)
deltao = np.array([po[2]-a2, po[1]-a1])[:, None]
Ltrans = Q @ deltao
phio = phi(A, po)
Lack = phio @ np.linalg.solve(O, np.array([[0.], [1.]]))
np.testing.assert_allclose(L, Ltrans, atol=1e-10)
np.testing.assert_allclose(L, Lack, atol=1e-10)
ao = A-L@C
eig_o = np.linalg.eigvals(ao)
np.testing.assert_allclose(np.poly(ao), po, atol=1e-10)
ad_o = np.eye(2)+Tmuestreo*ao
eig_do = np.linalg.eigvals(ad_o)
assert np.all(np.abs(eig_do) < 1)

# Conservar preámbulo, tipografía, márgenes y título de la referencia.
referencia = (BASE/'referencia_calculos_lab4_anterior.tex').read_text(encoding='utf-8')
doc = referencia[referencia.index('\\documentclass'):referencia.index('\\subsection*{1.')]
doc += r'\subsection*{1. Planta}'+'\n'
doc += 'La entrada $u$ es el voltaje efectivo del motor, sin zona muerta, y la salida $x$ está en mm. '
doc += r'Se usan $b='+f(b)+r'$, $a_1='+f(a1)+r'$ y $a_2='+f(a2)+r'$ (campo \texttt{a0} del JSON); solo se redondea la presentación.'+'\n'
doc += gather(r'G(s)=\frac{X(s)}{U(s)}=\frac{'+f(b)+r'}{s^2+'+format(a1,'.2f')+'s+'+format(a2,'.1f')+'}',
              r's^2+'+f(a1)+'s+'+f(a2)+r'=0\;\Rightarrow\;s='+f(lam[0].real,5)+r'\pm '+f(abs(lam[0].imag),4)+'j',
              r'K_{dc}=\frac{'+f(b)+'}{'+f(a2)+'}='+f(kdc,5)+r'~\text{mm/V},\qquad t_{s,la}(2\%)='+f(tsla,7)+r'~\text{s}\simeq '+format(tsla,'.3f')+r'~\text{s}',
              r'x_1=x,\quad x_2=\dot x', 'A='+mat(A)+r',\quad B='+mat(B)+r',\quad C='+mat(C)+r',\quad D=0')
doc += r'El asentamiento se calcula simulando el escalón unitario con una malla de $10~\mu$s, valor final $b/a_2$ y banda de $\pm2\%$; se refina por bisección la última entrada definitiva en la banda.'+'\n'
doc += (r'El valor nominal $0.211$ s corresponde a la aproximación $4/|\Re(p)|='+f(ts_aprox,7)+r'$ s, no al asentamiento exacto al $2\%$: en ese instante el error relativo todavía es $'+f(abs(np.real(np.exp(ts_aprox*lam) @ pesos))/kdc*100,5)+r'\%$.'+'\n')
doc += r'\subsection*{2. Controlabilidad y observabilidad}'+'\n'
doc += gather(r'S=[B\;\;AB]='+mat(S)+r',\quad\det S='+f(np.linalg.det(S))+r'\ne0',
              r'\mathcal O=\begin{bmatrix}C\\CA\end{bmatrix}='+mat(O)+r',\quad\det\mathcal O='+f(np.linalg.det(O))+r'\ne0')
doc += r'\subsection*{3. Especificaciones}'+'\n'
doc += gather(r't_{sd}='+f(tsd)+r'~\text{s},\quad\zeta_d='+f(zeta)+r',\quad\omega_{nd}=\frac{4}{\zeta_dt_{sd}}='+f(wn)+r'~\text{rad/s}',
              pol(dominante)+r'=0\;\Rightarrow\;s=-'+f(zeta*wn)+r'\pm '+f(wn*np.sqrt(1-zeta*zeta))+'j',
              r'p_{\text{lejano}}=10\zeta_d\omega_{nd}='+f(plejano))
doc += (r'El lazo abierto tiene un asentamiento nominal de $0.21$ s: pedir el $53\%$ de ese valor llevaría el polo lejano a $-'+format(40/(.53*ts_aprox),'.0f')+r'\simeq-360$ rad/s, incompatible con $T=10$ ms; con $t_{sd}=0.5$ s quedan $p_{\text{lejano}}T='+f(plejano*Tmuestreo)+r'$ y $|p_o|T='+f(wno*Tmuestreo)+'$.\n')
doc += (r'Con el asentamiento simulado, la misma regla daría $-'+format(40/(.53*tsla),'.1f')+r'$ rad/s; se conserva la decisión de diseño de $t_{sd}=0.5$ s.'+'\n')

for d, nombre in zip(resultados, ('Escalón', 'Rampa', 'Parábola')):
    m = d['m']; n=m+2; kh=d['kh']; pd=d['pd']; ah=d['ah']; bh=d['bh']
    doc += '\\subsection*{'+str(m+3)+'. Controlador '+nombre+' ($m='+str(m)+'$)}\n'
    derivadas = [r'\rho', r'\dot\rho', r'\ddot\rho']
    izquierda = [r'\dot\rho', r'\ddot\rho', r'\dddot\rho'][m-1]
    terminos = ['K_i'+derivadas[m-1-i] if i==0 else 'K_{i'+str(i+1)+'}'+derivadas[m-1-i] for i in range(m)]
    doc += gather(izquierda+r'=r-y,\qquad u=-K\mathbf x+'+'+'.join(terminos),
                  r'\mathbf z=[x_1\;x_2\;' + r'\;'.join(derivadas[:m])+r']^T,\qquad\hat K=[K_1\;K_2\;' + r'\;'.join('-K_i' if j==1 else '-K_{i'+str(j)+'}' for j in range(m,0,-1))+']',
                  r'\hat A='+mat(ah)+r',\quad\hat B='+mat(bh),
                  r'\hat S='+mat(d['sh']), r'\det\hat S='+f(np.linalg.det(d['sh']))+r'\ne0',
                  r'P_d(s)=('+pol(dominante)+r')(s+'+f(plejano)+')'+('^{'+str(m)+'}' if m>1 else '')+'='+pol(pd))
    doc += r'\paragraph{Método 1: asignación de polos}'+'\n'
    acltexto = [[f(v) for v in fila] for fila in ah]
    acltexto[1] = ['-'+f(a2)+'-'+f(b)+'K_1', '-'+f(a1)+'-'+f(b)+'K_2'] + [f(b)+('K_i' if j==1 else 'K_{i'+str(j)+'}') for j in range(m,0,-1)]
    acllatex = r'\begin{bmatrix}'+r'\\'.join('&'.join(fila) for fila in acltexto)+r'\end{bmatrix}'
    char = 's^{'+str(n)+'}+('+f(a1)+'+'+f(b)+'K_2)s^{'+str(n-1)+'}+('+f(a2)+'+'+f(b)+'K_1)s^{'+str(m)+'}'
    for j in range(1,m+1):
        pot = m-j
        char += '+'+f(b)+('K_i' if j==1 else 'K_{i'+str(j)+'}')+('s^{'+str(pot)+'}' if pot>1 else 's' if pot else '')
    doc += gather(r'A^*=\hat A-\hat B\hat K='+acllatex)
    if m == 1:
        doc += gather(r'sI-A^*=\begin{bmatrix}s&-1&0\\'+f(a2)+'+'+f(b)+r'K_1&s+'+f(a1)+'+'+f(b)+r'K_2&-'+f(b)+r'K_i\\1&0&s\end{bmatrix}')
    # Separar el determinante para respetar el ancho del formato original.
    doc += gather(r'\begin{aligned}\det(sI-A^*)&='+char[:char.index('+'+f(b)+'K_i')]+r'\\&'+char[char.index('+'+f(b)+'K_i'):]+r'\end{aligned}')
    ecuaciones = [f(a1)+'+'+f(b)+'K_2='+f(pd[1])+r'&\Rightarrow K_2='+f(kh[1]),
                  f(a2)+'+'+f(b)+'K_1='+f(pd[2])+r'&\Rightarrow K_1='+f(kh[0])]
    for j in range(1,m+1):
        ki = 'K_i' if j==1 else 'K_{i'+str(j)+'}'
        ecuaciones.append(f(b)+ki+'='+f(pd[2+j])+r'&\Rightarrow '+ki+'='+f(pd[2+j]/b))
    doc += '\\begin{align*}\n'+ '\\\\\n'.join(ecuaciones)+'\n\\end{align*}\n'
    doc += gather(r'\hat K='+mat(kh))
    doc += r'\paragraph{Método 2: Ackermann}'+'\n'
    doc += gather(r'\phi(\hat A)='+pol(pd,r'\hat A').replace(' + '+f(pd[-1]),' + '+f(pd[-1])+'I'),
                  r'\phi(\hat A)='+mat(d['ph']),
                  r'[0\;\cdots\;0\;1]\hat S^{-1}='+mat(d['fila']),
                  r'\hat K=[0\;\cdots\;0\;1]\hat S^{-1}\phi(\hat A)='+mat(kh))
    doc += r'\paragraph{Método 3: matriz de transformación}'+'\n'
    doc += gather(r'\det(sI-\hat A)='+pol(d['pa']), r'\hat M='+mat(d['mh']),
                  r'T=\hat S\hat M='+mat(d['trans']),r'T^{-1}='+mat(np.linalg.inv(d['trans'])),
                  r'[\alpha_'+str(n)+r'-a_'+str(n)+r'\;\cdots\;\alpha_1-a_1]='+mat(d['delta']),
                  r'\hat K='+mat(d['delta'])+r'T^{-1}='+mat(kh),
                  r'\operatorname{eig}(\hat A-\hat B\hat K)=\{-'+f(plejano)+r'\;('+str(m)+r'\text{ veces}),\;-'+f(zeta*wn)+r'\pm '+f(wn*np.sqrt(1-zeta*zeta))+'j\}')

doc += r'\subsection*{7. Observador}'+'\n'
doc += gather(r't_{s,o}=\frac{t_{sd}}{10}='+f(tso)+r'~\text{s},\quad\zeta_o='+f(zeta)+r',\quad\omega_{no}=\frac{4}{\zeta_ot_{s,o}}='+f(wno)+r'~\text{rad/s}',
              r'P_o(s)='+pol(po)+r'\;\Rightarrow\;s=-'+f(zeta*wno)+r'\pm '+f(wno*np.sqrt(1-zeta*zeta))+'j')
doc += r'\paragraph{Método 1}'+'\n'
doc += gather(r'A-LC=\begin{bmatrix}-L_1&1\\-'+f(a2)+r'-L_2&-'+f(a1)+r'\end{bmatrix}',
              r'sI-(A-LC)=\begin{bmatrix}s+L_1&-1\\'+f(a2)+r'+L_2&s+'+f(a1)+r'\end{bmatrix}',
              r'\det=s^2+(L_1+'+f(a1)+')s+('+f(a1)+'L_1+'+f(a2)+'+L_2)')
doc += '\\begin{align*}\nL_1+'+f(a1)+'='+f(po[1])+r'&\Rightarrow L_1='+f(L[0,0])+r'\\'+'\n'+f(a1)+'('+f(L[0,0])+')+'+f(a2)+'+L_2='+f(po[2])+r'&\Rightarrow L_2='+f(L[1,0])+'\n\\end{align*}\n'
doc += r'\paragraph{Método 2: matriz de transformación}'+'\n'
doc += gather('M='+mat(mo)+r',\quad Q=(M\mathcal O)^{-1}='+mat(Q), 'L=Q'+mat(deltao)+'='+mat(L))
doc += r'\paragraph{Método 3: Ackermann}'+'\n'
doc += gather(r'\phi(A)=A^2+'+f(po[1])+'A+'+f(po[2])+'I='+mat(phio),
              r'L=\phi(A)\mathcal O^{-1}\begin{bmatrix}0\\1\end{bmatrix}='+mat(L),
              r'\operatorname{eig}(A-LC)=\{-'+f(zeta*wno)+r'\pm '+f(wno*np.sqrt(1-zeta*zeta))+'j\}')
doc += r'\subsection*{8. Error en estado estable}'+'\n'
coef_rampa = resultados[0]['pd'][2]/resultados[0]['pd'][-1]
coef_parabola = resultados[1]['pd'][2]/resultados[1]['pd'][-1]
doc += gather(r'\frac{E(s)}{R(s)}=\frac{s^m[s^2+(a_1+bK_2)s+(a_2+bK_1)]}{P_d(s)}',
              r'm=1,\quad r=\dot r\,t:\quad e_{ss}='+f(coef_rampa)+r'\dot r,\quad\dot r=2~\text{mm/s}\Rightarrow e_{ss}='+f(coef_rampa*2)+r'~\text{mm}',
              r'm=2,\quad r=\tfrac12\ddot r\,t^2:\quad e_{ss}='+f(coef_parabola)+r'\ddot r',
              r'm=2,\quad r=2t:\quad e_{ss}=0,\qquad m=3,\quad r=\tfrac12\ddot r\,t^2:\quad e_{ss}=0')
doc += r'Para una parábola de aceleración constante $\ddot r$ en mm/s$^2$, el error de $m=1$ no tiene límite finito si $\ddot r\ne0$; su término creciente es $'+f(coef_rampa)+r'\ddot r\,t$. Los resultados suponen el modelo lineal sin saturación.'+'\n'
doc += r'\subsection*{9. Implementación digital}'+'\n'
doc += r'El ESP32 mide la cuenta acumulada $N[k]$ del encoder. Con $T=10$ ms, referencia y posición en mm, velocidad en mm/s y $u$ en V efectivos:'+'\n'
doc += gather(r'x[k]='+f(.06712)+r'N[k],\qquad v_{\mathrm{dif}}[k]=\frac{x[k]-x[k-1]}{'+f(Tmuestreo)+'}',
              r'v[k]='+f(1-.3)+r'v[k-1]+'+f(.3)+r'v_{\mathrm{dif}}[k]',
              r'\rho[k]=\rho[k-1]+'+f(Tmuestreo)+r'(r[k]-x[k])\qquad\text{(Euler hacia atrás)}',
              r'u[k]=-K_1x[k]-K_2v[k]+K_i\rho[k]',
              r'u[k]='+f(-resultados[0]['kh'][0])+r'x[k]-'+f(resultados[0]['kh'][1])+r'v[k]+'+f(-resultados[0]['kh'][2])+r'\rho[k]')
doc += gather(r'u_0(u)=\begin{cases}0~\text{V},&u>0\\1.1~\text{V},&u<0\\0,&u=0\end{cases},\qquad\operatorname{sign}(0)=0',
              r'V_{m,0}[k]=u[k]+u_0(u[k])\operatorname{sign}(u[k])',
              r'V_m[k]=\min(7.5,\max(-7.5,V_{m,0}[k]))',
              r'\mathrm{DAC}[k]=\operatorname{clip}_{[0,4095]}\!\left(\operatorname{round}\!\left(2048+\frac{V_m[k]}{4.7}\frac{4096}{3.3}\right)\right)')
doc += r'Para el observador se usa $y[k]=x[k]$ y $u[k]$ efectivo; si actúa la saturación se reconstruye a partir del voltaje aplicado descontando la zona muerta. La recursión siguiente usa Euler hacia adelante y los dos estados del instante $k$ simultáneamente:'+'\n'
doc += gather(r'\hat{\mathbf x}[k+1]=\hat{\mathbf x}[k]+T[(A-LC)\hat{\mathbf x}[k]+Bu[k]+Ly[k]]',
              r'\hat{\mathbf x}[k+1]='+mat(ad_o)+r'\hat{\mathbf x}[k]+'+mat(Tmuestreo*B)+r'u[k]+'+mat(Tmuestreo*L)+r'y[k]',
              r'\hat x_1[k+1]='+f(ad_o[0,0])+r'\hat x_1[k]+'+f(ad_o[0,1])+r'\hat x_2[k]+'+f(Tmuestreo*L[0,0])+r'y[k]',
              r'\hat x_2[k+1]='+f(ad_o[1,0])+r'\hat x_1[k]+'+f(ad_o[1,1])+r'\hat x_2[k]+'+f(Tmuestreo*b)+r'u[k]+'+f(Tmuestreo*L[1,0])+r'y[k]',
              r'\operatorname{eig}(I+T(A-LC))=\{'+f(eig_do[0].real)+r'\pm '+f(abs(eig_do[0].imag))+r'j\},\quad |\lambda|='+f(abs(eig_do[0]))+'<1')
doc += r'Por tanto, el error del observador discretizado converge en el modelo ideal. Esta comprobación corresponde al observador; la saturación, la zona muerta y el filtro EMA añaden efectos al controlador implementado.'+'\n'
doc += r'Las tres fórmulas de cada controlador y del observador coinciden numéricamente. Se comprueban también los polinomios característicos y los autovalores usando ganancias sin redondear; la separación numérica de polos repetidos se tolera hasta $0.01$ rad/s.'+'\n'
doc += '\\end{document}\n'
(BASE/'calculos_lab4.tex').write_text(doc, encoding='utf-8')

np.set_printoptions(precision=10, suppress=True)
print('Coeficientes de diseño:', b, a1, a2)
print('Simulación:', motor_simulacion)
print('Polos planta:', lam, 'Kdc:', kdc, 'ts,la (2%):', tsla)
for d in resultados:
    print(f"m={d['m']}: K_hat={d['kh']}; polos={d['eig']}; error polos={d['error_polos']:.3g}")
print('L=', L.ravel(), '; polos observador=', eig_o)
print('Polos observador digital=', eig_do, '; módulos=', abs(eig_do))
print('Errores: rampa m=1=', coef_rampa*2, '; parábola m=2=', coef_parabola, '* aceleración')
print('VERIFICACIONES CORRECTAS. Documento:', BASE/'calculos_lab4.tex')
