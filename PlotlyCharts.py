import numpy as np
import numpy.typing as npt
from dataclasses import dataclass
from typing import Final, Tuple, List
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# --- (TUS CLASES ORIGINALES SE MANTIENEN IGUAL) ---
# [PhysicsConfig, VehicleLimits, TargetEstimator, ProNavGuidance permanecen idénticos]

@dataclass(frozen=True)
class PhysicsConfig:
    G_REF: Final[float] = 9.80665
    R_AIR: Final[float] = 287.05
    RHO_0: Final[float] = 1.225
    H_SCALE: Final[float] = 8500.0
    DT: Final[float] = 0.01
    EPSILON: Final[float] = 1e-12
    MAX_STEPS: Final[int] = 8000 

@dataclass(frozen=True)
class VehicleLimits:
    G_LIMIT: Final[float] = 25.0
    S_REF: Final[float] = 0.05
    MASS_0: Final[float] = 150.0
    MASS_DRY: Final[float] = 100.0
    THRUST: Final[float] = 4500.0
    BURN_TIME: Final[float] = 8.0
    PROX_FUSE_THRESHOLD: Final[float] = 2.5

class TargetEstimator:
    def __init__(self, initial_pos: npt.NDArray[np.float64]):
        self.x = np.zeros(6); self.x[:3] = initial_pos
        self.P = np.eye(6) * 10.0; self.Q = np.eye(6) * 0.1; self.R = np.eye(3) * 0.5
    def predict_and_update(self, z_measured: npt.NDArray[np.float64], dt: float) -> npt.NDArray[np.float64]:
        F = np.eye(6); F[:3, 3:] = np.eye(3) * dt
        self.x = F @ self.x; self.P = F @ self.P @ F.T + self.Q
        H = np.zeros((3, 6)); H[:3, :3] = np.eye(3)
        S = H @ self.P @ H.T + self.R; K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z_measured - H @ self.x)
        self.P = (np.eye(6) - K @ H) @ self.P
        return self.x[:3]

class ProNavGuidance:
    def __init__(self, n_gain: float = 4.0):
        self._n_gain = n_gain; self._prev_los_unit = np.zeros(3); self._initialized = False
        self._limit_ms2 = VehicleLimits.G_LIMIT * PhysicsConfig.G_REF
    def reset(self): self._initialized = False
    def compute_cmd(self, pos_m, pos_t, vel_m, dt):
        rel_pos = pos_t - pos_m; dist = np.linalg.norm(rel_pos)
        if dist < 1.0: return np.zeros(3)
        los_unit = rel_pos / dist
        if not self._initialized: self._prev_los_unit = los_unit; self._initialized = True; return np.zeros(3)
        los_rate = (los_unit - self._prev_los_unit) / dt; self._prev_los_unit = los_unit
        speed_m = np.linalg.norm(vel_m); accel_cmd = self._n_gain * speed_m * los_rate
        v_unit = vel_m / speed_m; accel_lateral = accel_cmd - np.dot(accel_cmd, v_unit) * v_unit
        mag = np.linalg.norm(accel_lateral)
        return (accel_lateral / mag * self._limit_ms2) if mag > self._limit_ms2 else accel_lateral

class FlightSimulator:
    def __init__(self): self.guidance = ProNavGuidance()
    def get_atmos_density(self, alt): return PhysicsConfig.RHO_0 * np.exp(-max(0, alt) / PhysicsConfig.H_SCALE)

    def run_engagement(self, target_start_pos: npt.NDArray[np.float64]) -> dict:
        """Modificado para devolver telemetría básica de la ejecución."""
        self.guidance.reset()
        dt = PhysicsConfig.DT
        p_m, v_m, mass = np.array([0.0, 0.0, 1500.0]), np.array([300.0, 0.0, 0.0]), VehicleLimits.MASS_0
        p_t, v_t = target_start_pos.copy(), np.array([-150.0, 30.0, 0.0])
        estimator = TargetEstimator(p_t)
        prev_dist = np.inf
        
        trajectory_m = []

        for step in range(PhysicsConfig.MAX_STEPS):
            t_sim = step * dt
            p_t += v_t * dt
            z_t = p_t + np.random.normal(0, 0.8, 3)
            alt = p_m[2]; v_mag = np.linalg.norm(v_m); v_unit = v_m / v_mag
            rho = self.get_atmos_density(alt)
            cd = 0.25 if v_mag < 340 else 0.5
            drag_vec = -0.5 * rho * v_mag**2 * VehicleLimits.S_REF * cd * v_unit
            thrust_mag = VehicleLimits.THRUST if t_sim < VehicleLimits.BURN_TIME else 0.0
            thrust_vec = thrust_mag * v_unit
            if thrust_mag > 0: mass -= (VehicleLimits.MASS_0 - VehicleLimits.MASS_DRY) / VehicleLimits.BURN_TIME * dt
            
            est_p_t = estimator.predict_and_update(z_t, dt)
            a_guidance = self.guidance.compute_cmd(p_m, est_p_t, v_m, dt)
            a_total = a_guidance + (drag_vec + thrust_vec) / mass + np.array([0, 0, -PhysicsConfig.G_REF])
            
            v_m += a_total * dt; p_m += v_m * dt
            trajectory_m.append(p_m.copy())
            
            dist = np.linalg.norm(p_t - p_m)
            if dist < VehicleLimits.PROX_FUSE_THRESHOLD:
                return {"hit": True, "miss_dist": dist, "pos_final": p_m, "path": np.array(trajectory_m)}
            if (dist > prev_dist and step > 200) or alt < 0:
                return {"hit": False, "miss_dist": dist, "pos_final": p_m, "path": np.array(trajectory_m)}
            prev_dist = dist
        return {"hit": False, "miss_dist": prev_dist, "pos_final": p_m, "path": np.array(trajectory_m)}

# --- GENERADOR DE VISUALIZACIONES (PLOTLY) ---

def generate_aerospace_dashboard(df: pd.DataFrame, sample_paths: List[np.ndarray]):
    # 1. Gráfico de Dispersión de Impactos (CEP Analysis)
    fig_cep = px.scatter(df, x='final_y', y='final_z', color='hit', 
                         title='Análisis de Precisión en Punto de Intercepción (CEP)',
                         labels={'final_y': 'Desviación Lateral (m)', 'final_z': 'Altitud (m)'},
                         color_discrete_map={True: '#00cc96', False: '#ef553b'},
                         template='plotly_dark')
    fig_cep.add_shape(type="circle", x0=-2.5, y0=1500-2.5, x1=2.5, y1=1500+2.5, 
                      line_color="White", fillcolor="rgba(255, 255, 255, 0.1)")

    # 2. Trayectorias 3D (Muestra de 15 vuelos)
    fig_3d = go.Figure()
    for i, path in enumerate(sample_paths[:15]):
        fig_3d.add_trace(go.Scatter3d(x=path[:,0], y=path[:,1], z=path[:,2],
                                     mode='lines', line=dict(width=2),
                                     name=f'Vuelo {i+1}', opacity=0.7))
    fig_3d.update_layout(title="Visualización de Trayectorias 3D (Muestra Monte Carlo)",
                         scene=dict(xaxis_title='X (m)', yaxis_title='Y (m)', zaxis_title='Z (m)'),
                         template='plotly_dark')

    # 3. Histograma de Miss Distance
    fig_hist = px.histogram(df, x='miss_dist', color='hit', nbins=50,
                            title='Distribución de Distancia de Fallo (Miss Distance)',
                            template='plotly_dark', barmode='overlay')

    # Mostrar gráficos
    fig_cep.show()
    fig_3d.show()
    fig_hist.show()

def run_monte_carlo_analysis(iterations: int = 500):
    sim = FlightSimulator()
    data_list = []
    paths = []

    print(f"[*] Ejecutando Simulación Monte Carlo: N={iterations}...")
    
    for _ in range(iterations):
        t_pos = np.array([np.random.uniform(5000, 7000), 
                          np.random.uniform(-1000, 1000), 
                          np.random.uniform(1000, 3000)])
        res = sim.run_engagement(t_pos)
        
        data_list.append({
            "hit": res["hit"],
            "miss_dist": res["miss_dist"],
            "final_x": res["pos_final"][0],
            "final_y": res["pos_final"][1],
            "final_z": res["pos_final"][2]
        })
        if len(paths) < 20: paths.append(res["path"])

    df = pd.DataFrame(data_list)
    
    # Estadísticas de Ingeniería
    cep_90 = np.percentile(df['miss_dist'], 90)
    hit_rate = df['hit'].mean() * 100
    
    print(f"\n--- REPORTE TÉCNICO ---")
    print(f"Probabilidad de Intercepción (Pk): {hit_rate:.2f}%")
    print(f"CEP 90 (Circular Error Probable): {cep_90:.3f} m")
    
    generate_aerospace_dashboard(df, paths)

if __name__ == "__main__":
    np.random.seed(42)
    run_monte_carlo_analysis(1000)
