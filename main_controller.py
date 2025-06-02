import datetime
from typing import List, Tuple

from controller import Robot

# ===== CONFIGURACIÓN GENERAL =====
STEP_DURATION = 64
SPEED_LIMIT = 6.28

# ===== INICIALIZACIÓN DEL ROBOT =====
robot = Robot()

# Inicializar sensores de proximidad
proximity = [robot.getDevice(f"ps{i}") for i in range(8)]
for sensor in proximity:
    sensor.enable(STEP_DURATION)

# Inicializar cámara
cam = robot.getDevice("camera")
cam.enable(STEP_DURATION)

# Inicializar motores
motor_left = robot.getDevice("left wheel motor")
motor_right = robot.getDevice("right wheel motor")
motor_left.setPosition(float("inf"))
motor_right.setPosition(float("inf"))
motor_left.setVelocity(0)
motor_right.setVelocity(0)

# Inicializar LEDs
led_devices = []
for idx in range(10):
    try:
        led_devices.append(robot.getDevice(f"led{idx}"))
    except Exception:
        continue


def update_leds(status: str) -> None:
    """
    Actualiza los LEDs según el estado del robot.
    Los LEDs del e-puck son típicamente de un solo color (rojo).

    Args:
        status (str): Estado actual del robot ('go', 'turning', 'obstacle', 'goal', 'stuck')
    """
    # Para LEDs de un solo color, usar valores 0 (apagado) o 1 (encendido)
    led_patterns = {
        "go": [1, 0, 1, 0, 1, 0, 1, 0],  # Patrón alternado
        "turning": [1, 1, 1, 1, 0, 0, 0, 0],  # Frente encendido
        "obstacle": [1, 1, 1, 1, 1, 1, 1, 1],  # Todos encendidos
        "goal": [0, 1, 0, 1, 0, 1, 0, 1],  # Patrón alternado inverso
        "stuck": [1, 0, 0, 0, 0, 0, 0, 0],  # Solo uno encendido
    }

    pattern = led_patterns.get(status, [0] * 8)

    # Aplicar patrón a los LEDs disponibles
    for i, led in enumerate(led_devices):
        if i < len(pattern):
            led.set(pattern[i])
        else:
            led.set(0)


def get_prox(idx: int) -> float:
    """
    Obtiene el valor del sensor de proximidad especificado.

    Args:
        idx (int): Índice del sensor de proximidad (0-7)

    Returns:
        float: Valor actual del sensor de proximidad
    """
    return proximity[idx].getValue()


def show_current_results(current_time: float, is_final: bool = False) -> None:
    """
    Muestra y guarda los resultados actuales de la navegación.

    Args:
        current_time (float): Tiempo actual de la simulación
        is_final (bool): Si es el reporte final o periódico
    """
    total_time = current_time - start

    # Mostrar en consola
    prefix = (
        "===== RESULTADOS FINALES ====="
        if is_final
        else "===== REPORTE PERIÓDICO ====="
    )
    print(f"\n{prefix}")
    print(f"Tiempo transcurrido  : {total_time:.2f} s")
    print(f"Distancia aprox.     : {travelled:.2f} unidades")
    print(f"Colisiones           : {crashes}")
    print(f"Meta alcanzada       : {'Sí' if goal_done else 'No'}")
    print(
        f"Estado actual        : {'Atascado' if stuck_loops >= MAX_STUCK else 'Navegando'}"
    )

    # Guardar en archivo
    suffix = "final" if is_final else "periodico"
    fecha_actual = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    try:
        filename = f"resultados_navegacion_{suffix}_{fecha_actual}.txt"
        with open(filename, "w", encoding="utf-8") as f:
            f.write(f"{prefix}\n")
            f.write(f"Tiempo transcurrido  : {total_time:.2f} s\n")
            f.write(f"Distancia aprox.     : {travelled:.2f} unidades\n")
            f.write(f"Colisiones           : {crashes}\n")
            f.write(f"Meta alcanzada       : {'Sí' if goal_done else 'No'}\n")
            f.write(
                f"Estado actual        : {'Atascado' if stuck_loops >= MAX_STUCK else 'Navegando'}\n"
            )
            f.write(f"Iteraciones atascado : {stuck_loops}\n")

        if not is_final:
            print(f"📁 Reporte guardado: {filename}")

    except Exception as e:
        print(f"⚠️ No se pudo guardar el archivo: {e}")


# ===== MÉTRICAS =====
start = robot.getTime()
travelled = 0.0
crashes = 0
goal_done = False
stuck_loops = 0
last_report_time = start
REPORT_INTERVAL = 10.0  # Mostrar resultados cada 10 segundos


# ===== LÓGICA DE NAVEGACIÓN =====
def main_navigation() -> None:
    """
    Función principal de navegación que implementa el comportamiento de seguir la pared derecha.
    Controla el movimiento del robot basándose en los sensores de proximidad y maneja
    las colisiones y obstáculos.

    Modifica las variables globales: crashes, goal_done, stuck_loops
    """
    global crashes, goal_done, stuck_loops
    # Sensores principales
    left_front = get_prox(0)
    right_front = get_prox(7)
    right_side = get_prox(2)
    # Detección de obstáculos
    obstacle_front = left_front > 80 or right_front > 80
    right_is_clear = right_side < 60
    # Colisión
    if left_front > 100 and right_front > 100:
        crashes += 1
        update_leds("obstacle")
        motor_left.setVelocity(-0.5 * SPEED_LIMIT)
        motor_right.setVelocity(-0.5 * SPEED_LIMIT)
        stuck_loops += 1
        for _ in range(5):
            robot.step(STEP_DURATION)
        return
    # Movimiento
    if obstacle_front:
        update_leds("turning")
        motor_left.setVelocity(0.4 * SPEED_LIMIT)
        motor_right.setVelocity(-0.4 * SPEED_LIMIT)
        stuck_loops += 1
    elif right_is_clear:
        update_leds("turning")
        motor_left.setVelocity(0.6 * SPEED_LIMIT)
        motor_right.setVelocity(0.2 * SPEED_LIMIT)
        stuck_loops = 0
    else:
        update_leds("go")
        motor_left.setVelocity(0.5 * SPEED_LIMIT)
        motor_right.setVelocity(0.5 * SPEED_LIMIT)
        stuck_loops = 0


# ===== BUCLE PRINCIPAL =====
print("Controlador en ejecución.")
MAX_STUCK = 50

while robot.step(STEP_DURATION) != -1 and not goal_done and stuck_loops < MAX_STUCK:
    main_navigation()

    # Calcular distancia
    v_l = motor_left.getVelocity()
    v_r = motor_right.getVelocity()
    v_mean = (v_l + v_r) / 2
    travelled += abs(v_mean * (STEP_DURATION / 1000))

    # Mostrar resultados periódicos
    current_time = robot.getTime()
    if current_time - last_report_time >= REPORT_INTERVAL:
        show_current_results(current_time, is_final=False)
        last_report_time = current_time

# Parar motores
motor_left.setVelocity(0)
motor_right.setVelocity(0)

# Estado final si está atascado
if stuck_loops >= MAX_STUCK and not goal_done:
    update_leds("stuck")
    print("⚠️ Robot atascado. Detenido por seguridad.")

# ===== RESULTADOS =====
end = robot.getTime()
show_current_results(end, is_final=True)
