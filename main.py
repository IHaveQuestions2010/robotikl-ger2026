from hub import motion_sensor, port
import motor, motor_pair, color_sensor, color, force_sensor
import runloop
from hub import port
import color_sensor, motor
import runloop

buff = False

# CONFIG

reflectivnessTarget = 50
sensorPort = port.C
sensorMotorPort = port.A

async def followLine():
    while buff == False:
        await runloop.sleep_ms(50)
    while True:
        reflectivnessReading = color_sensor.reflection(sensorPort)
        reflectivnessError = reflectivnessTarget - reflectivnessReading
        steering = int(reflectivnessError * 1.8)
        motor.run_for_degrees(sensorMotorPort, -steering, 90)

        await runloop.sleep_ms(50)

def turn(pair, angle, turnVelocity):
    motor_pair.move(pair, angle, velocity=-turnVelocity)
    
    #if angle > 5:
    #    motor_pair.move_tank(pair, -turnVelocity, turnVelocity)
    #if angle < -5:
    #    motor_pair.move_tank(pair, turnVelocity, -turnVelocity)

async def main():
    await motor.run_to_absolute_position(sensorMotorPort, 0, 180, direction=motor.SHORTEST_PATH)
    global buff
    buff = True
    while True:
        sensorAngle = motor.absolute_position(sensorMotorPort)
        turn(motor_pair.PAIR_1, int(2.5*sensorAngle), 120)
        await runloop.sleep_ms(50)


motor_pair.pair(motor_pair.PAIR_1, port.F, port.D)
runloop.run(main(), followLine())
