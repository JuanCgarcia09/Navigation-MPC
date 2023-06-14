#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 13 14:11:10 2023

@author: k3lso
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Leer el archivo Excel
df = pd.read_excel('Track3.xlsx')

# Obtener los valores de la columna deseada
columna = df['phi']

# Crear una lista de números enteros para usar como índices
indices = list(range(len(columna)))

# Realizar el ajuste de la línea de tendencia utilizando polyfit de NumPy
coeficientes = np.polyfit(indices, columna, 1)  # Ajuste de una línea recta (grado 1)

# Crear una función de línea de tendencia a partir de los coeficientes
linea_tendencia = np.poly1d(coeficientes)

# Crear una figura y ejes
fig, ax = plt.subplots()

# Graficar los puntos originales
ax.scatter(indices, columna, label='Datos originales')

# Graficar la línea de tendencia
ax.plot(indices, linea_tendencia(indices), color='red', label='Línea de tendencia')

# Agregar leyendas y títulos
ax.legend()
ax.set_xlabel('Índice')
ax.set_ylabel('Valor')
ax.set_title('Línea de tendencia')

# Mostrar la gráfica
plt.show()