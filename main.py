from hub import port
import motor, motor_pair, color_sensor,color
import runloop

# INIT
runSensorFollowLine = False
motor_pair.pair(motor_pair.PAIR_1, port.F, port.D)


# CONFIG
reflectivnessTarget = 50
sensorPort = port.C
sensorMotorPort = port.A
pair = motor_pair.PAIR_1

"""
async def noLineFound():
        if motor.absolute_position(sensorMotorPort) >= 90:
            motor.stop(sensorMotorPort)
            while color_sensor.reflection(sensorPort) >= 91:
                motor.run_for_degrees(sensorMotorPort, -1, 90)
                if motor.absolute_position(sensorMotorPort) <= -90:
                    await motor.run_to_absolute_position(sensorMotorPort, 0, 90, direction=motor.SHORTEST_PATH)
                    await motor_pair.move_tank_for_time(pair, -110, -110, 2000)
                    break
"""

async def sensorFollowLine():
    integral = 0
    last_error = 0

    Kp = 1.8
    Ki = 0.01
    Kd = 0.8

    while True:
        while runSensorFollowLine == False:
            await runloop.sleep_ms(50)
        # https://issssse.github.io/spikeguide/#lls-help-python-examples-pid-control
        reflectivnessReading = color_sensor.reflection(sensorPort)
        reflectivnessError = reflectivnessTarget - reflectivnessReading

        P = Kp * reflectivnessError

        integral += reflectivnessError
        I = Ki * integral

        derivative= reflectivnessError - last_error
        D = Kd * derivative

        last_error = reflectivnessError
        steering = int(P + I + D)
        motor.run_for_degrees(sensorMotorPort, -steering, 90)

        await runloop.sleep_ms(50)

async def main():
    await motor.run_to_absolute_position(sensorMotorPort, 0, 180, direction=motor.SHORTEST_PATH)
    # Allows followLine() to run only after the sensor is centered
    global runSensorFollowLine
    runSensorFollowLine = True
    while True:
        if color_sensor.color(sensorPort) == color.RED:
            print("WIN")
            motor_pair.stop(pair)
            motor.stop(sensorMotorPort)
            break
        elif color_sensor.reflection(sensorPort) <= 90:
            # When sensorAngle increases the robot will turn more cancelling it out so it follows the line
            sensorAngle = motor.absolute_position(sensorMotorPort)
            motor_pair.move(pair, int(2.5*sensorAngle), velocity=-120)
        else:
            hasCheckedLeft = False
            motor_pair.stop(pair)
            if motor.absolute_position(sensorMotorPort) >= 90:
                if hasCheckedLeft == True:
                    await motor.run_to_absolute_position(sensorMotorPort, 0, 90, direction=motor.SHORTEST_PATH)
                    await motor_pair.move_tank_for_time(pair, -110, -110, 2000)
                    hasCheckedLeft = False
                else:
                    runSensorFollowLine = False
                    await motor.run_to_absolute_position(sensorMotorPort, 90, 90, direction=motor.SHORTEST_PATH)
                    runSensorFollowLine = True
                    hasCheckedLeft = True
        await runloop.sleep_ms(50)

runloop.run(main(), sensorFollowLine())
