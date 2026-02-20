"""Tests for SimConfig: TOML loading and speed limit conversion."""
import pathlib
import tempfile

import pytest
from traffic_sim.config import SimConfig


class TestSpeedLimitsMs:
    def test_standard_3_lane_conversion(self):
        """Default config converts 130/110/90 km/h correctly."""
        cfg = SimConfig()
        limits = cfg.speed_limits_ms(3)
        assert len(limits) == 3
        assert limits[0] == pytest.approx(130.0 / 3.6, abs=0.01)
        assert limits[1] == pytest.approx(110.0 / 3.6, abs=0.01)
        assert limits[2] == pytest.approx(90.0 / 3.6, abs=0.01)

    def test_fewer_lanes_than_config(self):
        """Requesting 2 lanes from a 3-lane config returns first 2."""
        cfg = SimConfig()
        limits = cfg.speed_limits_ms(2)
        assert len(limits) == 2
        assert limits[0] == pytest.approx(130.0 / 3.6, abs=0.01)
        assert limits[1] == pytest.approx(110.0 / 3.6, abs=0.01)

    def test_more_lanes_than_config_extrapolates(self):
        """Extra lanes are appended with decreasing limits."""
        cfg = SimConfig()
        limits = cfg.speed_limits_ms(4)
        assert len(limits) == 4
        assert limits[3] < limits[2]   # fourth lane slower than third
        assert limits[3] > 0.0         # still a positive value

    def test_single_lane(self):
        """Works correctly with exactly 1 lane."""
        cfg = SimConfig()
        limits = cfg.speed_limits_ms(1)
        assert len(limits) == 1
        assert limits[0] == pytest.approx(130.0 / 3.6, abs=0.01)

    def test_custom_limits(self):
        """Custom km/h list converts correctly."""
        cfg = SimConfig()
        cfg.lane_speed_limits_kmh = [180.0, 130.0]
        limits = cfg.speed_limits_ms(2)
        assert limits[0] == pytest.approx(180.0 / 3.6, abs=0.01)
        assert limits[1] == pytest.approx(130.0 / 3.6, abs=0.01)


class TestFromToml:
    def _write_toml(self, content: str) -> str:
        f = tempfile.NamedTemporaryFile(suffix=".toml", mode="w", delete=False)
        f.write(content)
        f.close()
        return f.name

    def test_empty_file_uses_all_defaults(self):
        path = self._write_toml("")
        cfg = SimConfig.from_toml(path)
        assert cfg.ramp.onramp_rate    == pytest.approx(0.5)
        assert cfg.ramp.offramp_prob   == pytest.approx(0.30)
        assert cfg.lane_change.cooldown_s == pytest.approx(3.0)

    def test_ramp_section_overrides(self):
        path = self._write_toml("[ramp]\nonramp_rate = 2.0\nmin_gap_m = 30.0\n")
        cfg = SimConfig.from_toml(path)
        assert cfg.ramp.onramp_rate == pytest.approx(2.0)
        assert cfg.ramp.min_gap_m   == pytest.approx(30.0)
        assert cfg.ramp.offramp_prob == pytest.approx(0.30)  # default unchanged

    def test_lane_change_section_overrides(self):
        path = self._write_toml("[lane_change]\nincentive_m = 15.0\nsafety_gap_m = 10.0\n")
        cfg = SimConfig.from_toml(path)
        assert cfg.lane_change.incentive_m  == pytest.approx(15.0)
        assert cfg.lane_change.safety_gap_m == pytest.approx(10.0)
        assert cfg.lane_change.cooldown_s   == pytest.approx(3.0)  # default

    def test_road_speed_limits_loaded(self):
        path = self._write_toml(
            "[road]\nlane_speed_limits_kmh = [160, 130, 100]\n"
        )
        cfg = SimConfig.from_toml(path)
        assert cfg.lane_speed_limits_kmh == [160, 130, 100]

    def test_project_config_toml_loads(self):
        """The project's own config.toml loads without error."""
        project_root = pathlib.Path(__file__).parent.parent
        cfg = SimConfig.from_toml(str(project_root / "config.toml"))
        assert cfg.ramp.onramp_rate > 0
        assert cfg.ramp.ramp_length_m > 0
        assert cfg.ramp.max_queue > 0

    def test_missing_file_raises(self):
        with pytest.raises(FileNotFoundError):
            SimConfig.from_toml("/nonexistent/path/config.toml")
