from typing import List, Any, Protocol


class Publisher(Protocol):
    def send(self, message: dict) -> None:
        ...


def to_float(value: Any) -> float:
    try:
        return float(value)
    except (ValueError, TypeError):
        raise ValueError(f"Invalid float value: {value}")


class Twist:
    def __init__(self, publisher: Publisher, topic: str = "/cmd_vel"):
        self.publisher = publisher
        self.topic = topic
        self._advertised = False

    def _ensure_advertised(self):
        if not self._advertised:
            self.publisher.send({
                "op": "advertise",
                "topic": self.topic,
                "type": "geometry_msgs/TwistStamped"
            })
            self._advertised = True

    def publish(self, linear: List[Any], angular: List[Any]):
        self._ensure_advertised()
        linear_f = [to_float(val) for val in linear]
        angular_f = [to_float(val) for val in angular]

        msg = {
            "op": "publish",
            "topic": self.topic,
            "msg": {
                "header": {},
                "twist": {
                    "linear": {"x": linear_f[0], "y": 0, "z": 0},
                    "angular": {"x": 0, "y": 0, "z": angular_f[2]}
                }
            }
        }
        self.publisher.send(msg)
        
        return msg

    def publish_sequence(self, linear_seq: List[Any], angular_seq: List[Any], duration_seq: List[Any]):
        import time
        linear_flat = [to_float(val) for sublist in linear_seq for val in sublist]
        angular_flat = [to_float(val) for sublist in angular_seq for val in sublist]
        duration_f = [to_float(val) for val in duration_seq]

        for i in range(len(duration_f)):
            linear = linear_flat[i*3:(i+1)*3]
            angular = angular_flat[i*3:(i+1)*3]
            self.publish(linear, angular)
            time.sleep(duration_f[i])
