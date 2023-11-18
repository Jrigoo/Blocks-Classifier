from remote.coppelia import Coppelia

if __name__ == '__main__':
    coppelia = Coppelia()
    block_counter = {
        "Blue": 0,
        "Red": 0,
        "Green": 0,
        "White": 0
    }
    block_height = 0.02

    try:
        while True:
            coppelia.set_conveyor(0.05)
            color = coppelia.classify_blocks()

            if color:
                print("The color detected is {}".format(color))

                # Leemos el sensor de proximidad
                obj_detection = coppelia.read_proximity_sensor()
                while not obj_detection:
                    obj_detection = coppelia.read_proximity_sensor()
                coppelia.set_conveyor(0)

                # Movemos el mirobot A regoger el bloque
                x, y, z = [0.27515,  -0.00976,  0.08999]
                coppelia.move_robot([x, y, z])

                # Gripper ON
                coppelia.set_gripper(1)
                coppelia.move_robot([x, y, z+0.1])
                coppelia.move_robot([0.19822, 0, 0.22858])

                # Movemos el robot al cup 1
                x, y, z = list(coppelia.cups_pos[color])
                coppelia.move_robot([x, y, 0.220])
                coppelia.move_robot(
                    [x, y, z + block_counter[color]*block_height])
                coppelia.set_gripper(0)
                coppelia.move_robot([x, y, 0.220])
                coppelia.move_robot([0.19822, 0, 0.22858])

                block_counter[color] += 1

    except KeyboardInterrupt:
        print("closed.")
