# append to the bottom (or keep a dedicated file if you prefer)
class OnlineMetric:
    """Time-weighted mean accumulator."""
    def __init__(self) -> None:
        self.sum = 0.0
        self.t = 0.0
    def accumulate(self, value: float, dt: float) -> None:
        self.sum += value * dt
        self.t   += dt
    def mean(self) -> float:
        return self.sum / max(self.t, 1e-9)

class MetricBag:
    def __init__(self) -> None:
        self._m = {}
    def accumulate(self, name: str, value: float, dt: float) -> None:
        self._m.setdefault(name, OnlineMetric()).accumulate(value, dt)
    def summary(self) -> dict:
        return {k: v.mean() for k, v in self._m.items()}
