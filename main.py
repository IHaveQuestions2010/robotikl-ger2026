from hub import port
import motor, motor_pair, color_sensor,color
import runloop

# INIT
pauseFollowLine = True
motor_pair.pair(motor_pair.PAIR_1, port.F, port.D)


# CONFIG
reflectivnessTarget = 50
sensorPort = port.C
sensorMotorPort = port.A
pair = motor_pair.PAIR_1


async def sensorFollowLine():
    integral = 0
    last_error = 0

    Kp = 1.8
    Ki = 0.01
    Kd = 0.8
    while pauseFollowLine == True:
        await runloop.sleep_ms(50)
    while True:
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
    global pauseFollowLine
    pauseFollowLine = False
    while True:
        if color_sensor.color(sensorPort) == color.RED:
            print("WIN")
            pauseFollowLine = True
            motor_pair.stop(pair)
            motor.stop(sensorMotorPort)
            break
        # Triggers when the line is detected
        elif color_sensor.reflection(sensorPort) <= 90:
            pauseFollowLine = False
            # When sensorAngle increases the robot will turn more cancelling it out so it follows the line
            sensorAngle = motor.absolute_position(sensorMotorPort)
            motor_pair.move(pair, int(2.5*sensorAngle), velocity=-120)
        # Triggers when the robot loses the line
        else:
            # Intended logic
            # 1. Stop moving forward
            # 2. Try to find the line on the left side
                # 2a. If the line is found, follow it
                # 2b. If the line was not found try to find it on the right side
            # 3. Move forward for two seconds or until the line is found

            motor.stop(pair)
            if motor.absolute_position(sensorMotorPort) >= 90:
                pauseFollowLine = True
                motor.stop(sensorMotorPort)
                motor.run_to_absolute_position(sensorMotorPort, -90, 90)
            if motor.absolute_position(sensorMotorPort) <= -90:
                await motor.run_to_absolute_position(sensorMotorPort, 0, 90, direction=motor.SHORTEST_PATH)
                await motor_pair.move_tank_for_time(pair, -110, -110, 2000)
        await runloop.sleep_ms(50)

runloop.run(main(), sensorFollowLine())
