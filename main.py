from remote.coppelia import Mirobot, Conveyor, Camera, Client, ProximitySensor, Object

if __name__ == '__main__':
    # Initialize robot and sensors
    client = Client()
    miro = Mirobot(client.id)
    conveyor = Conveyor(client.id)
    camera = Camera(client.id)
    sensor = ProximitySensor(client.id)

    # Cup Objects
    colors = ["Blue", "Red", "White", "Green"]
    cups = {colors[i]: Object(
        client.id, f"./Cup{colors[i]}/pickPos", True) for i in range(4)}

    # Counter of Blocks
    block_counter = {
        "Blue": 0,
        "Red": 0,
        "Green": 0,
        "White": 0
    }

    # Default block height
    block_height = 0.02

    try:
        while True:
            conveyor.set_velocity(0.03)
            color = camera.color_classification()
            homing = [0.19822, 0, 0.22858]

            if color:
                print("The color detected is {}".format(color))
                # Read proximity Sensor
                obj_detection = sensor.read()
                while not obj_detection:
                    obj_detection = sensor.read()
                conveyor.set_velocity(0)

                # Move tip to block
                x, y, z = [0.27515,  -0.00976,  0.08999]
                miro.move(position=[x, y, z])
                miro.set_gripper(1)

                # Move block up
                miro.move([x, y, z+0.1])
                # Move tip to homing
                miro.move(homing)

                # Move block to desired color cup
                x, y, z = list(cups[color].position)
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
