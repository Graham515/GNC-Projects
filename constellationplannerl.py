#Planner.py
import math
import csv
import pandas as pd
import os

# -------------------- Configuration --------------------
CONFIG = {
    "orbit_type": "MEO",   # LEO, SSO, POLAR, MEO, GEO, MOLNIYA, TUNDRA, HEO, HAPS
    "num_sats": 12
}
CSV_LOG = "auto_constellation.csv"

# -------------------- Earth constants --------------------
MU = 3.986004418e14     # m^3/s^2
R_EARTH = 6371000.0     # m

# -------------------- Orbit presets --------------------
ORBIT_PRESETS = {
    'LEO':       (550.0, 53.0, 0.0),
    'SSO':       (600.0, 97.6, 0.0),
    'POLAR':     (800.0, 90.0, 0.0),
    'MEO':       (20200.0, 55.0, 0.0),
    'GEO':       (35786.0, 0.0, 0.0),
    'MOLNIYA':   ((600.0, 39700.0), 63.4, None),
    'TUNDRA':    (None, 63.4, 0.27),
    'HEO':       ((300.0, 36000.0), 63.0, None),
    'HAPS':      (2000.0, 70.0, 0.0),
    'CUSTOM':    (None, None, None)
}

# -------------------- Utility --------------------
def two_body_period_minutes(a_m):
    return 2*math.pi*math.sqrt(a_m**3 / MU)/60.0

def circular_velocity_kms(a_m):
    return math.sqrt(MU / a_m)/1000.0

def choose_planes_and_spacing(N):
    # favor more planes for coverage
    planes = min(N, max(1, int(math.ceil(math.sqrt(N)*1.5))))
    sats_per_plane = int(math.ceil(N/planes))
    return planes, sats_per_plane

# -------------------- Read existing CSV --------------------
def read_existing_log():
    if os.path.exists(CSV_LOG):
        df = pd.read_csv(CSV_LOG)
        if not df.empty:
            return df
    return pd.DataFrame()

# -------------------- Constellation builder --------------------
def make_constellation(orbit_type, N, orbit_id):
    orbit_type_u = orbit_type.strip().upper()
    if orbit_type_u not in ORBIT_PRESETS:
        raise ValueError(f"Unknown orbit type '{orbit_type_u}'")
    preset = ORBIT_PRESETS[orbit_type_u]

    altitude = preset[0]
    inclination = preset[1]
    ecc_preset = preset[2]

    # compute semi-major axis and perigee/apogee
    if orbit_type_u=='TUNDRA':
        a_m = 42164000.0
        ecc = ecc_preset
        rp_m = a_m*(1-ecc)
        ra_m = a_m*(1+ecc)
    elif isinstance(altitude, tuple):
        perigee_km, apogee_km = altitude
        rp_m = R_EARTH + perigee_km*1000.0
        ra_m = R_EARTH + apogee_km*1000.0
        a_m = 0.5*(rp_m+ra_m)
        ecc = (ra_m-rp_m)/(ra_m+rp_m)
    else:
        alt_km = float(altitude)
        a_m = R_EARTH + alt_km*1000.0
        ecc = 0.0 if ecc_preset is None else ecc_preset
        rp_m = ra_m = a_m

    if inclination is None: inclination=53.0

    planes, sats_per_plane = choose_planes_and_spacing(N)
    raan_step = 360.0 / planes if planes>0 else 0.0

    sat_list=[]
    for sat_idx in range(N):
        plane = sat_idx // sats_per_plane
        sat_in_plane = sat_idx % sats_per_plane

        base_RAAN = (plane * raan_step) % 360.0
        # stagger new orbits by orbit_id to avoid overlap
        RAAN_deg = (base_RAAN + (orbit_id-1)*raan_step/2) % 360.0

        plane_phase = (plane%2)*(360.0/(sats_per_plane*2.0))
        mean_anomaly_deg = (360.0/sats_per_plane)*sat_in_plane + plane_phase
        mean_anomaly_deg %= 360.0

        sat = {
            "sat_id": None,  # will assign after combining with log
            "plane": plane+1,
            "sat_in_plane": sat_in_plane+1,
            "orbit_type": orbit_type_u,
            "semi_major_axis_m": round(a_m,3),
            "eccentricity": round(ecc,6),
            "inclination_deg": round(inclination,6),
            "RAAN_deg": round(RAAN_deg,6),
            "arg_periapsis_deg": 0.0,
            "mean_anomaly_deg": round(mean_anomaly_deg,6),
            "orbital_period_min": round(two_body_period_minutes(a_m),6),
            "orbital_velocity_kms": round(circular_velocity_kms(a_m),6),
            "altitude_m": None,
            "perigee_alt_m": None,
            "apogee_alt_m": None
        }

        if ecc>0:
            sat["perigee_alt_m"]=round(rp_m-R_EARTH,3)
            sat["apogee_alt_m"]=round(ra_m-R_EARTH,3)
        else:
            sat["altitude_m"]=round(a_m-R_EARTH,3)

        sat_list.append(sat)

    return sat_list

# -------------------- Save CSV --------------------
def save_csv(new_sats, existing_df):
    # assign sat_ids sequentially
    if not existing_df.empty:
        max_id = existing_df["sat_id"].max()
    else:
        max_id = 0

    for i,sat in enumerate(new_sats):
        sat["sat_id"] = max_id + i +1

    combined_df = pd.concat([existing_df, pd.DataFrame(new_sats)], ignore_index=True)
    combined_df.to_csv(CSV_LOG, index=False)
    return combined_df

# -------------------- Main --------------------
def main():
    orbit_type = CONFIG.get("orbit_type","LEO")
    N = int(CONFIG.get("num_sats",10))

    existing_df = read_existing_log()
    # orbit_id = 1 + number of distinct previous orbits
    orbit_id = 1 + existing_df["orbit_type"].nunique() if not existing_df.empty else 1

    new_sats = make_constellation(orbit_type, N, orbit_id)
    combined_df = save_csv(new_sats, existing_df)

    print(f"\nSaved CSV: {os.path.abspath(CSV_LOG)}")
    print(f"\nSatellites grouped by plane (classic orbital elements):")
    display_cols=["sat_id","plane","sat_in_plane","semi_major_axis_m","eccentricity","inclination_deg",
                  "RAAN_deg","arg_periapsis_deg","mean_anomaly_deg","altitude_m","perigee_alt_m","apogee_alt_m"]
    print(combined_df.sort_values(["plane","sat_in_plane"]).loc[:,display_cols].to_string(index=False))

if __name__=="__main__":
    main()
