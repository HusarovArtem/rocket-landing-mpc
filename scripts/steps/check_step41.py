from rocket_landing.params import VehicleParams as RealVehicleParams
from rocket_landing.controllers.mpc import MpcWeights as RealMpcWeights

p_default = RealVehicleParams()
p_from_yaml = RealVehicleParams(config_path="configs/vehicle_nominal.yaml")

for field in ("initial_mass", "dry_mass", "isp", "g0", "max_thrust",
              "min_thrust", "g","max_tilt_angle"):
    default_value = getattr(p_default, field)
    yaml_value = getattr(p_from_yaml, field)
    print(f"{field}: default={default_value} from_yaml={yaml_value}   "
          f"match={default_value == yaml_value}")

print()


w_default = RealMpcWeights()
w_from_yaml = RealMpcWeights(config_path="configs/mpc_weights_default.yaml")

for field in ("Qr", "Qv", "R", "S", "Qf", "w_m"):
    default_value = getattr(w_default, field)
    yaml_value = getattr(w_from_yaml, field)
    print(f"{field}: default={default_value} from_yaml={yaml_value}   "
          f"match={default_value == yaml_value}")
