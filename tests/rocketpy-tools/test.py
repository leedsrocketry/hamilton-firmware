from lura_rockets import G2_IRW
from rocketpy import Flight, Environment


location = [55.7265712, -4.8117083, 243]

new_env = Environment(latitude=location[0], longitude=location[1],
                          elevation=location[2])
new_env.set_date((2024, 10, 2, 9, 0, 0))
new_env.set_atmospheric_model(type="Forecast", file="GFS")

# G2_IRW.rocket.draw()

flight = Flight(
    rocket=G2_IRW.rocket,
    environment=new_env,
    rail_length = 3,
    inclination = 88,
    heading = 0,
    max_time = 600,
    verbose = True
)
    
accelerometer_data = flight.rocket.sensors.get_components()[0].measured_data
barometer_data = flight.rocket.sensors.get_components()[1].measured_data


with open("./output_data.csv", "w+") as f:
    f.write("time,accel_x,accel_y,accel_z\n")
    for acc_line, baro_line in zip(accelerometer_data, barometer_data):
        time = acc_line[0]
        print(acc_line[0], baro_line[0])
        # altitude = flight.altitude(baro_line[0])
        # temperature = new_env.temperature(altitude)

        # f.write(f"{acc_line[0]:.3f},{int(acc_line[1]*1000/9.81)},{int(acc_line[2]*1000/9.81)},{int(acc_line[3]*1000/9.81)},{baro_line[1]:.3f},{temperature:.3f}\n")
    