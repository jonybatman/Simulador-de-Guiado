import numpy as np
import numpy.typing as npt
from dataclasses import dataclass
from typing import Final, Tuple
import time

# --- CONSTANTES FÍSICAS Y DE MISIÓN (Estándar Aero) ---

@dataclass(frozen=True)
class PhysicsConfig:
    G_REF: Final[float] = 9.80665       # Gravedad estándar (m/s2)
    R_AIR: Final[float] = 287.05        # Constante gases aire
    RHO_0: Final[float] = 1.225         # Densidad nivel del mar (kg/m3)
    H_SCALE: Final[float] = 8500.0      # Altura de escala atm (m)
    DT: Final[float] = 0.01             # Paso de integración (s)
    EPSILON: Final[float] = 1e-12
    MAX_STEPS: Final[int] = 8000 

@dataclass(frozen=True)
class VehicleLimits:
    G_LIMIT: Final[float] = 25.0        # Límite estructural (Gs)
    S_REF: Final[float] = 0.05          # Área de referencia (m2)
    MASS_0: Final[float] = 150.0        # Masa inicial (kg)
    MASS_DRY: Final[float] = 100.0      # Masa sin combustible (kg)
    THRUST: Final[float] = 4500.0       # Empuje motor (N)
    BURN_TIME: Final[float] = 8.0       # Tiempo de quemado (s)
    PROX_FUSE_THRESHOLD: Final[float] = 2.5

# --- NÚCLEO GNC (Guidance, Navigation & Control) ---

class TargetEstimator:
    """Filtro de Kalman para estimación de estado del blanco."""
    def __init__(self, initial_pos: npt.NDArray[np.float64]):
        # Estado [x, y, z, vx, vy, vz]
        self.x = np.zeros(6)
        self.x[:3] = initial_pos
        self.P = np.eye(6) * 10.0  # Covarianza inicial
        self.Q = np.eye(6) * 0.1   # Ruido de proceso
        self.R = np.eye(3) * 0.5   # Ruido de medición (Sensor)

    def predict_and_update(self, z_measured: npt.NDArray[np.float64], dt: float) -> npt.NDArray[np.float64]:
        # Matriz de transición F
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        # Predicción
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        # Actualización (Ganancia de Kalman)
        H = np.zeros((3, 6))
        H[:3, :3] = np.eye(3)
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z_measured - H @ self.x)
        self.P = (np.eye(6) - K @ H) @ self.P
        return self.x[:3]

class ProNavGuidance:
    """Navegación Proporcional Pura con compensación de aceleración."""
    def __init__(self, n_gain: float = 4.0):
        self._n_gain: Final[float] = n_gain
        self._prev_los_unit = np.zeros(3)
        self._initialized = False
        self._limit_ms2 = VehicleLimits.G_LIMIT * PhysicsConfig.G_REF

    def reset(self) -> None:
        self._initialized = False

    def compute_cmd(self, pos_m: npt.NDArray[np.float64], pos_t: npt.NDArray[np.float64], 
                    vel_m: npt.NDArray[np.float64], dt: float) -> npt.NDArray[np.float64]:
        rel_pos = pos_t - pos_m
        dist = np.linalg.norm(rel_pos)
        if dist < 1.0: return np.zeros(3)
        
        los_unit = rel_pos / dist
        if not self._initialized:
            self._prev_los_unit = los_unit
            self._initialized = True
            return np.zeros(3)

        # Derivada de la línea de mira (LOS Rate)
        los_rate = (los_unit - self._prev_los_unit) / dt
        self._prev_los_unit = los_unit

        # Ley de guiado: nc = N * V_closing * LOS_rate
        speed_m = np.linalg.norm(vel_m)
        accel_cmd = self._n_gain * speed_m * los_rate
        
        # El comando debe ser perpendicular a la velocidad (aceleración lateral)
        v_unit = vel_m / speed_m
        accel_lateral = accel_cmd - np.dot(accel_cmd, v_unit) * v_unit
        
        return self._apply_limits(accel_lateral)

    def _apply_limits(self, accel: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        mag = np.linalg.norm(accel)
        if mag > self._limit_ms2:
            return (accel / mag) * self._limit_ms2
        return accel

# --- MOTOR DE SIMULACIÓN (Física Aeroespacial) ---

class FlightSimulator:
    def __init__(self):
        self.guidance = ProNavGuidance()

    def get_atmos_density(self, alt: float) -> float:
        return PhysicsConfig.RHO_0 * np.exp(-max(0, alt) / PhysicsConfig.H_SCALE)

    def run_engagement(self, target_start_pos: npt.NDArray[np.float64]) -> Tuple[bool, float]:
        self.guidance.reset()
        dt = PhysicsConfig.DT
        
        # Estado Misil
        p_m = np.array([0.0, 0.0, 1500.0])
        v_m = np.array([300.0, 0.0, 0.0])
        mass = VehicleLimits.MASS_0
        
        # Estado Blanco
        p_t = target_start_pos.copy()
        v_t = np.array([-150.0, 30.0, 0.0])
        
        estimator = TargetEstimator(p_t)
        prev_dist = np.inf

        for step in range(PhysicsConfig.MAX_STEPS):
            t_sim = step * dt
            
            # 1. Dinámica del Blanco
            p_t += v_t * dt
            z_t = p_t + np.random.normal(0, 0.8, 3) # Ruido de radar
            
            # 2. Aero & Propulsión
            alt = p_m[2]
            v_mag = np.linalg.norm(v_m)
            v_unit = v_m / v_mag
            
            # Arrastre (Drag): D = 0.5 * rho * v^2 * S * Cd
            rho = self.get_atmos_density(alt)
            cd = 0.25 if v_mag < 340 else 0.5 # Incremento por onda de choque (Mach)
            drag_mag = 0.5 * rho * v_mag**2 * VehicleLimits.S_REF * cd
            drag_vec = -drag_mag * v_unit
            
            # Empuje (Thrust)
            thrust_mag = VehicleLimits.THRUST if t_sim < VehicleLimits.BURN_TIME else 0.0
            thrust_vec = thrust_mag * v_unit
            if thrust_mag > 0:
                mass -= (VehicleLimits.MASS_0 - VehicleLimits.MASS_DRY) / VehicleLimits.BURN_TIME * dt

            # 3. GNC
            est_p_t = estimator.predict_and_update(z_t, dt)
            a_guidance = self.guidance.compute_cmd(p_m, est_p_t, v_m, dt)
            
            # 4. Integración de Fuerzas (F = ma)
            gravity = np.array([0, 0, -PhysicsConfig.G_REF])
            a_total = a_guidance + (drag_vec + thrust_vec) / mass + gravity
            
            # RK4 para velocidad y posición
            v_m += a_total * dt
            p_m += v_m * dt
            
            # 5. Cierre de Bucle
            dist = np.linalg.norm(p_t - p_m)
            if dist < VehicleLimits.PROX_FUSE_THRESHOLD:
                return True, dist
            if dist > prev_dist and step > 200: # Miss
                return False, dist
            if alt < 0: return False, dist
            
            prev_dist = dist

        return False, prev_dist

def run_validation_suite(iterations: int = 100):
    sim = FlightSimulator()
    results = []
    success = 0

    print(f"[*] Iniciando Campaña Monte Carlo (Física 3-DOF + Kalman): N={iterations}")
    
    for i in range(iterations):
        t_pos = np.array([np.random.uniform(5000, 7000), 
                          np.random.uniform(-1000, 1000), 
                          np.random.uniform(1000, 3000)])
        hit, miss = sim.run_engagement(t_pos)
        results.append(miss)
        if hit: success += 1

    print(f"\n--- REPORTE DE MISIÓN ---")
    print(f"Probabilidad de Intercepción: {(success/iterations)*100:.1f}%")
    print(f"Miss Distance (Media): {np.mean(results):.3f} m")
    print(f"CEP 90: {np.percentile(results, 90):.3f} m")

if __name__ == "__main__":
    np.random.seed(42)
    run_validation_suite(1000)
