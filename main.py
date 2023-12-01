from remote.coppelia import Mirobot, Conveyor, Client, Camera, ProximitySensor, Object


if __name__ == '__main__':
    # Initialize robot and sensors
    client = Client.connect()
    miro = Mirobot("./Mirobot")
    conveyor = Conveyor()
    camera = Camera()
    prox_sensor = ProximitySensor()

    # Cup Objects
    colors = ["Blue", "Red", "White", "Green"]
    cups = {colors[i]: Object(f"./Cup{colors[i]}/pickPos", True)
            for i in range(4)}

    # Counter of Blocks
    block_counter = {
        "Blue": 0,
        "Red": 0,
        "Green": 0,
        "White": 0
    }

    # Default block height
    block_height = 0.02

    # Default positions
    homing = [0.19822, 0, 0.22858]
    pickup_position = [0.27515,  -0.00976,  0.08999]

    try:
        while True:
            conveyor.set_velocity(0.03)
            color = camera.color_classification()

            if color:
                # Read proximity Sensor
                obj_detection = prox_sensor.read()
                while not obj_detection:
                    obj_detection = prox_sensor.read()
                conveyor.set_velocity(0)

                # Move tip to block
                x, y, z = pickup_position
                miro.move([x, y, z])
                miro.set_gripper(1)

                # Move block up
                miro.move([x, y, z+0.1])

                # Move tip to homing
                miro.move(homing)

                # Move block to desired color cup
                x, y, z = cups[color].position
                miro.move([x, y, 0.220])
                miro.move(
                    [x, y, z + block_counter[color]*block_height])

                # Leave block
                miro.set_gripper(0)
                miro.move([x, y, 0.220])
                miro.move(homing)

                block_counter[color] += 1

    except KeyboardInterrupt:
        print("Robot Stoped.")
