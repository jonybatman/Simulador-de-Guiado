# Simulador de Intercepción (GNC 3-DOF)

Este proyecto implementa un motor de simulación para la interceptación de objetivos en un entorno tridimensional, integrando modelos de guiado, navegación y control (GNC) con física aeroespacial.

## Arquitectura del Sistema

El software se divide en tres pilares fundamentales que emulan el funcionamiento de un sistema de defensa real:

### 1. Estimación de Estado: Filtro de Kalman (LKF)
El módulo TargetEstimator implementa un Filtro de Kalman lineal para procesar mediciones ruidosas de la posición del blanco.
*   Modelo Cinemático: Utiliza una matriz de transición de estado (F) de velocidad constante.
*   Proceso de Corrección: El filtro calcula la ganancia de Kalman (K) para balancear la incertidumbre del modelo de predicción frente al ruido del sensor de medición (Radar/Lidar). Esto permite una estimación suave de la trayectoria del objetivo incluso con interferencias.

### 2. Ley de Guiado: Navegación Proporcional Pura (PPN)
La lógica de interceptación se basa en la Navegación Proporcional, una ley de control donde el comando de aceleración es proporcional a la velocidad de rotación de la línea de mira (LOS Rate).
*   Ecuación de Control: accel_cmd = N * V_speed * LOS_rate, donde N es la constante de navegación (fijada en 4.0).
*   Restricciones Físicas: El comando de aceleración se proyecta perpendicularmente al vector velocidad y se satura a 25 Gs para respetar los límites estructurales del vehículo (VehicleLimits.G_LIMIT).

### 3. Modelo Físico y Aerodinámico
La simulación resuelve las ecuaciones de movimiento de Newton mediante integración numérica (RK1/Euler):
*   Variación Atmosférica: Implementa el modelo de densidad del aire basado en la altura de escala (H_scale), donde la densidad (rho) disminuye exponencialmente con la altitud.
*   Dinámica de Masa: El sistema simula la pérdida de masa del vehículo durante la fase de empuje (Burn Time), afectando directamente la relación Empuje/Peso.
*   Resistencia Aerodinámica (Drag): Se modela el coeficiente de arrastre (Cd) con una función que aumenta al alcanzar velocidades supersónicas (Mach > 1.0).

## Validación del Sistema: Análisis de Monte Carlo

Para determinar la fiabilidad del algoritmo, el script ejecuta una campaña de validación estadística:
1.  Iteraciones: 1000 escenarios con posiciones iniciales del blanco aleatorias.
2.  Métrica CEP 90: Define el radio del círculo en el que el 90% de los intentos resultan en impacto o aproximación crítica.
3.  Probabilidad de Intercepción (Pk): Ratio de éxito basado en el umbral de proximidad de la espoleta (Prox Fuse).

## Requisitos Técnicos
*   Python 3.8+
*   NumPy (para álgebra lineal y operaciones vectoriales)
*   Standard Library: dataclasses, typing, time.

## Ejecución
Para correr la suite de validación completa:
python main.py
