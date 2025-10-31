"""Gripper stress tester: send multiple random MoveGripper requests.

This node calls the `/move_gripper` service repeatedly with random, valid
parameters to exercise the DH gripper server. By default, it sends 100 calls.

Options (env/CLI):
- --count N: number of calls (default: 100)
- --service-name NAME: service to call (default: /move_gripper)
- --min-pos, --max-pos: bounds for target_position (default: 0..1000)
- --min-force, --max-force: bounds for target_force (default: 20..100)
- --sleep SEC: delay between calls (default: 0.05s)
- --timeout SEC: per-call timeout waiting for response (default: 3s)
"""

from __future__ import annotations

import argparse
import random
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from spark_state_machine_interfaces.srv import MoveGripper


class GripperStressTester(Node):
    """A simple client that hammers the MoveGripper service with random inputs."""

    def __init__(
        self,
        service_name: str,
        count: int,
        min_pos: int,
        max_pos: int,
        min_force: int,
        max_force: int,
        sleep_sec: float,
        timeout_sec: float,
    ) -> None:
        super().__init__('gripper_stress_tester')
        self._service_name = service_name
        self._count = count
        self._min_pos = min_pos
        self._max_pos = max_pos
        self._min_force = min_force
        self._max_force = max_force
        self._sleep_sec = sleep_sec
        self._timeout_sec = timeout_sec

        self._client = self.create_client(MoveGripper, self._service_name)
        self.get_logger().info(
            f"Waiting for service {self._service_name!r}..."
        )
        if not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f"Service {self._service_name} not available after 5s; exiting."
            )
            raise RuntimeError('service unavailable')
        self.get_logger().info("Service is available; starting stress calls.")

    def run(self) -> None:
        success_cnt = 0
        fail_cnt = 0
        for i in range(1, self._count + 1):
            pos = random.randint(self._min_pos, self._max_pos)
            force = random.randint(self._min_force, self._max_force)

            req = MoveGripper.Request()
            req.target_position = int(pos)
            req.target_force = int(force)

            t0 = time.perf_counter()
            future = self._client.call_async(req)
            # Spin until done or timeout
            end_time = t0 + self._timeout_sec
            while rclpy.ok() and not future.done() and time.perf_counter() < end_time:
                rclpy.spin_once(self, timeout_sec=0.05)

            duration = time.perf_counter() - t0
            if not future.done():
                self.get_logger().warn(
                    f"[{i}/{self._count}] pos={pos}, force={force} -> TIMEOUT after {duration:.2f}s"
                )
                fail_cnt += 1
            else:
                try:
                    res = future.result()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(
                        f"[{i}/{self._count}] pos={pos}, force={force} -> exception: {exc}"
                    )
                    fail_cnt += 1
                else:
                    status = 'OK' if res.success else 'ERR'
                    self.get_logger().info(
                        f"[{i}/{self._count}] pos={pos}, force={force} -> {status} in {duration:.2f}s: {res.message}"
                    )
                    success_cnt += 1 if res.success else 0

            if self._sleep_sec > 0:
                time.sleep(self._sleep_sec)

        self.get_logger().info(
            f"Completed {self._count} calls: success={success_cnt}, failures/timeouts={fail_cnt}"
        )


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--service-name', default='/move_gripper', help='Service name to call')
    parser.add_argument('--count', type=int, default=100, help='Number of calls to send')
    parser.add_argument('--min-pos', type=int, default=0, help='Minimum position value')
    parser.add_argument('--max-pos', type=int, default=1000, help='Maximum position value')
    parser.add_argument('--min-force', type=int, default=20, help='Minimum force value')
    parser.add_argument('--max-force', type=int, default=100, help='Maximum force value')
    parser.add_argument('--sleep', type=float, default=0.05, help='Sleep seconds between calls')
    parser.add_argument('--timeout', type=float, default=3.0, help='Per-call timeout seconds')
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    args = parse_args(argv)
    rclpy.init()
    node: Optional[GripperStressTester] = None
    try:
        node = GripperStressTester(
            service_name=args.service_name,
            count=args.count,
            min_pos=args.min_pos,
            max_pos=args.max_pos,
            min_force=args.min_force,
            max_force=args.max_force,
            sleep_sec=0.50,
            timeout_sec=args.timeout,
        )
        node.run()
    except RuntimeError:
        # Service not available; message already logged
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
