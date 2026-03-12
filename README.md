# Simulador-de-Guiado
Simulador de Guiado y Navegación Aeroespacial

🚀 Flight Simulator (3-DOF)

Este proyecto es un simulador de vuelo técnico que implementa los pilares de GNC (Guidance, Navigation, and Control). Utiliza modelos físicos reales para simular la interceptación de un objetivo mediante algoritmos de navegación proporcional y filtros de estimación.

🧠 ¿Qué hace este código?

El script simula un escenario donde un misil debe alcanzar un blanco móvil. Para lograrlo, el sistema pasa por cuatro etapas constantes:

Navegación (Filtro de Kalman): El misil no "ve" perfectamente al objetivo. El código implementa un Filtro de Kalman que toma mediciones ruidosas de un radar y estima la posición real del blanco, eliminando errores de medición.

Guiado (ProNav): Utiliza Navegación Proporcional Pura (ProNav). Es el algoritmo estándar en la industria aeroespacial que calcula la aceleración necesaria para interceptar un objetivo basándose en el cambio de la línea de mira (Line-of-Sight).

Física y Aerodinámica: No es un movimiento lineal simple. El simulador calcula:

Arrastre (Drag): Resistencia del aire basada en la densidad atmosférica (que cambia con la altura) y el efecto de la barrera del sonido (Mach).
Propulsión: Pérdida de masa de combustible en tiempo real según el empuje del motor.
Gravedad y Atmósfera: Modelo de densidad exponencial y gravedad estándar.

Simulación Monte Carlo: Ejecuta cientos de pruebas con variables aleatorias para obtener estadísticas de éxito (Probabilidad de intercepción y error de precisión CEP).

🛠️ Tecnologías utilizadas

Python 3.x

NumPy: Para todo el cálculo matricial y álgebra lineal del Filtro de Kalman.
Dataclasses: Para una gestión limpia y profesional de las constantes físicas.
Tipado estricto (Type Hinting): Código robusto y fácil de leer para otros desarrolladores.

📊 Cómo leer los resultados

Al ejecutarlo, verás un reporte de misión:

Probabilidad de Intercepción: Qué tan efectivo es el algoritmo bajo estrés.
Miss Distance: La distancia promedio a la que pasó el interceptor del blanco.
CEP 90: El radio dentro del cual cae el 90% de los intentos (indicador de precisión militar).
