"""Reusable OnRobot RG6 Modbus TCP driver. Importing never moves hardware."""

import math

from pymodbus.client import ModbusTcpClient


class OnRobotGripper:
    """Control an RG6 using widths in millimetres and forces in newtons.

    Call connect() before use and disconnect() when finished, or use a context
    manager. Movement commands return without waiting for physical completion.
    """

    def __init__(
        self,
        ip: str = "195.148.65.33",
        port: int = 502,
        device_id: int = 65,
        timeout: float = 2.0,
    ) -> None:
        self.device_id = device_id
        self.client = ModbusTcpClient(ip, port=port, timeout=timeout)

    @property
    def is_connected(self) -> bool:
        """Whether the local client has an open connection."""
        return self.client.connected

    def connect(self) -> None:
        if not self.client.connect():
            self.client.close()
            raise ConnectionError("Could not connect to the OnRobot gripper.")

    def disconnect(self) -> None:
        self.client.close()

    def _require_connection(self) -> None:
        if not self.is_connected:
            raise ConnectionError("OnRobot gripper is not connected. Call connect() first.")

    @staticmethod
    def _to_register(value: float, name: str) -> int:
        if not math.isfinite(value) or not 0 <= value <= 6553.5:
            raise ValueError(
                f"{name} must be finite and fit an unsigned 16-bit register "
                "at 0.1-unit resolution."
            )
        return int(value * 10)

    def move_rg6(self, width_mm: float, force_n: float) -> None:
        """Send a move command using the existing RG6 register map.

        Width must be 0–160 mm and force 25–120 N. Finger geometry may
        further restrict the usable width range.
        """
        if not 0 <= width_mm <= 160 or not 25 <= force_n <= 120:
            raise ValueError("RG6 requires width 0–160 mm and force 25–120 N")
        target_width = self._to_register(width_mm, "width_mm")
        target_force = self._to_register(force_n, "force_n")
        self._require_connection()
        response = self.client.write_registers(
            address=0,
            values=[target_force, target_width, 1],
            device_id=self.device_id,
        )
        if response.isError():
            raise RuntimeError(f"OnRobot move command failed: {response}")

    def get_width(self) -> float:
        """Read the current gripper width in millimetres."""
        self._require_connection()
        response = self.client.read_holding_registers(
            address=267,
            count=1,
            device_id=self.device_id,
        )
        if response.isError():
            raise RuntimeError(f"OnRobot width read failed: {response}")
        return response.registers[0] / 10.0

    def __enter__(self) -> "OnRobotGripper":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.disconnect()
