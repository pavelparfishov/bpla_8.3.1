# mission_planner.py

from uav_control import UAVControl
import time
from typing import List, Tuple
import logging

logger = logging.getLogger(__name__)

class MissionPlanner:
    """
    Класс для планирования и выполнения миссий БПЛА.
    """

    def __init__(self, connection_string: str):
        """
        Инициализация планировщика миссий.

        Args:
            connection_string (str): Строка подключения MAVLink.
        """
        self.uav = UAVControl(connection_string)

    def execute_mission(self, waypoints: List[Tuple[float, float, float]]) -> None:
        """
        Выполнение миссии по заданным точкам.

        Args:
            waypoints (List[Tuple[float, float, float]]): Список точек (lat, lon, alt).
        """
        try:
            # Проверка наличия точек в миссии
            if not waypoints:
                raise ValueError("Список точек миссии пуст")

            # Взведение и взлёт
            self.uav.arm()
            self.uav.set_mode('GUIDED')
            self.uav.takeoff(waypoints[0][2])

            # Ожидание набора высоты с периодической проверкой телеметрии
            for _ in range(10):  # Максимум 10 проверок
                telemetry = self.uav.get_telemetry()
                if telemetry and abs(telemetry.get('alt', 0) - waypoints[0][2]) < 1.0:
                    logger.info("Достигнута заданная высота")
                    break
                time.sleep(1)
            else:
                raise TimeoutError("Не удалось достичь заданной высоты")

            # Полёт по точкам
            for idx, waypoint in enumerate(waypoints):
                logger.info(f"Переходим к точке {idx + 1}: {waypoint}")
                self.uav.goto(*waypoint)

                # Ожидание достижения точки с проверкой телеметрии
                reached = False
                for _ in range(20):  # Максимум 20 проверок
                    telemetry = self.uav.get_telemetry()
                    if telemetry:
                        lat_diff = abs(telemetry.get('lat', 0.0) - waypoint[0])
                        lon_diff = abs(telemetry.get('lon', 0.0) - waypoint[1])
                        alt_diff = abs(telemetry.get('alt', 0.0) - waypoint[2])
                        if lat_diff < 0.0001 and lon_diff < 0.0001 and alt_diff < 1.0:
                            reached = True
                            logger.info(f"Достигнута точка {idx + 1}")
                            break
                    time.sleep(1)
                if not reached:
                    logger.error(f"Не удалось достичь точки {idx + 1}")
                    raise Exception(f"Не удалось достичь точки {idx + 1}")

            # Возвращение и посадка
            self.uav.set_mode('RTL')
            logger.info("Возвращение домой и посадка")

            # Ожидание завершения посадки с периодической проверкой телеметрии
            for _ in range(10):
                telemetry = self.uav.get_telemetry()
                if telemetry and telemetry.get('alt', 0) < 0.5:
                    logger.info("Посадка завершена")
                    break
                time.sleep(1)
            else:
                logger.warning("Посадка не подтверждена")

            self.uav.disarm()
        except Exception as e:
            logger.error(f"Ошибка во время выполнения миссии: {e}")
            try:
                self.uav.disarm()
            except Exception as disarm_error:
                logger.error(f"Ошибка при разоружении: {disarm_error}")
            raise
