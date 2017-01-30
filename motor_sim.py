import matplotlib.pyplot as plt
import math

# Variables de simulacion.
ti = 0. # Tiempo inicial.
tf = 2. # Tiempo final.
ts = 0.001 # Tiempo de muestreo.
t = ti # Tiempo actual de simulacion.

# Variables del motor
#K = 145.47 # Ganancia de DC del motor.
K = 2266.667
#tau = 0.087 # Constante de tiempo,
tau = 0.0136
w = 0. # Velocidad inicial del motor.
wr = 1000.0 # Velocidad deseada.

# Vectores de solucion.
W = [w] # Vector de soluciones.
T = [t] # Vector de tiempo.
U = [0.] # Vector de voltaje.
Wr = [wr / 1000.0] # Vector de velicidad deseada.

# Variables de PID
e = 0. # Error.
ie = 0. # Integral del error.
de = 0. # Derivada del error.
prev_e = 0. # Variable auxiliar: error anterior.

# Comportamiento deseado.
Ts = 0.2 # Tiempo de estabilizacion
Pos = 0.05 # Sobretiro maximo de 5%.

# Parametros de sistema de segundo orden.
z = abs(math.log(Pos)) / math.sqrt((math.log(Pos))**2 + math.pi**2)
wn = 4 / (z * Ts)

# Calcular ganancias en funcion de
# respuesta deseada.

kp = (2 * z * wn * tau) / K
ki = (tau * wn ** 2 - 1) / K
kd = 0.
print('System Parameters: DC Gain: {}, Time Costant: {}'.format(K, tau))
print('System Response: Natural freq: {}, Damping coefficient: {}'.format(wn, z))
print('Design parameters -> Max Overshoot: {}, Settling time: {}'.format(Pos, Ts))
print('Controller gains -> Kp: {}, Ki: {}, Kd: {}'.format(kp, ki, kd))

while t <= tf:
  # Calcular error
  e = wr - w

  # Calcular derivada
  de = (e - prev_e) / ts
  prev_e = e

  # Calcular integral
  ie += e * ts

  # Calcular PID
  Vin = kp * e + kd * de + ki * ie

  # Calcular dinamica del motor
  dw = (K * Vin - w) / tau

  # Integrar para obtener velocidad.
  w += dw * ts

  t += ts # Siguiente muestra de tiempo.

  # Guardar en vectores para graficar.
  W.append(w / 1000.) # Convertir a RPM x 1000
  U.append(Vin)
  Wr.append(wr / 1000.) # Convertir a RPM x 1000
  T.append(t)

# Desplegar resultados.
plt.plot(T, W, linewidth = 2.)
plt.title('Respuesta del Motor con control PD')
plt.xlabel('Tiempo (s)')
plt.ylabel('Velocidad (RPM x 1000)')
plt.grid()
plt.show()
