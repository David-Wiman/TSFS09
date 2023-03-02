total_fuel_mass_kg = trapz(t, m_dot_fi);
total_fuel_volume_dm3 = 1000*total_fuel_mass_kg/750; % 750 kg/m3
total_driven_distance_km = VehicleDistance(end)/1000;

dm3_per_km = total_fuel_volume_dm3/total_driven_distance_km;

dm3_per_10_km = 10*dm3_per_km;

dm3_per_100_km = 100*dm3_per_km