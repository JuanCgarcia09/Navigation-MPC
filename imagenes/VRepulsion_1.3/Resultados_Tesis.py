# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


#Leer Excel y Determinar variables importantes
plt.close("all")
Data = "Monaco.xlsx"
df = pd.read_excel(Data)
valores = df[["x","y","theta","v","phi","rsx","rsy","lsx","lsy"]]

posicionx = valores["x"]
posiciony = valores["y"]
rx = valores["rsx"]
ry = valores["rsy"]
lx = valores["lsx"]
ly = valores["lsy"]
theta = valores["theta"]
v = valores["v"]
phi = valores["phi"]

tiempo = np.linspace(0,len(posicionx)*0.01,len(posicionx))

#Promedios Distancias
DistanciaMuroD = np.sqrt(((posicionx-rx)**2)+((posiciony-ry)**2))
DistanciaMuroI = np.sqrt(((posicionx-lx)**2)+((posiciony-ly)**2))
 
promedioMuroD = sum(DistanciaMuroD)/len(DistanciaMuroD)
promedioMuroI = sum(DistanciaMuroI)/len(DistanciaMuroI)

print(promedioMuroD)
print(promedioMuroI)
print(promedioMuroD + promedioMuroI)
print(len(posicionx)*0.01)

fig,ax = plt.subplots()
ax.plot(posicionx, posiciony, label = "Trayectoria", color = "red" )
ax.legend()
ax.set_xlabel('Tiempo (s)')
ax.set_ylabel('Amplitud (m)')
ax.set_title('Trayectoria')
ax.grid()

#Angulo del vehiculo
fig1, ax1 = plt.subplots()
ax1.plot(tiempo,theta,label = "Ángulo del vehículo", color = "blue")
ax1.legend()
ax1.set_xlabel('Tiempo (s)')
ax1.set_ylabel('Amplitud (radianes)')
ax1.set_title('Ángulo del vehículo')

# Velocidad del Vehículo
fig2, ax2 = plt.subplots()
ax2.plot(tiempo,v, label = "Velocidad", color = "gray")
ax2.legend()
ax2.set_xlabel('Tiempo (s)')
ax2.set_ylabel('Amplitud (m/s)')
ax2.set_title('Velocidad del vehículo')

# Ángulo de dirección
fig3, ax3 = plt.subplots()
ax3.plot(tiempo,phi, label = "Ángulo de dirección", color = "black")
ax3.legend()
ax3.set_xlabel('Tiempo (s)')
ax3.set_ylabel('Amplitud (radianes)')
ax3.set_title('Ángulo de dirección')

# Distancia a los muros desde la derecha
fig4, ax4 = plt.subplots()
ax4.plot(tiempo,DistanciaMuroD, label = "Distancia a los muros R", color = "green")
ax4.legend()
ax4.set_xlabel('Tiempo (s)')
ax4.set_ylabel('Amplitud (m)')
ax4.set_title('Distancia del vehículo a los muros de la derecha')

# Distancia a los muros desde la izquierda
fig5, ax5 = plt.subplots()
ax5.plot(tiempo, DistanciaMuroI, label = "Distancia a los muros L", color = "orange")
ax5.legend()
ax5.set_xlabel('Tiempo (s)')
ax5.set_ylabel('Amplitud (m)')
ax5.set_title('Distancia a los muros de la izquierda')

plt.show()