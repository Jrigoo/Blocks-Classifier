from remote.mirobot import Mirobot

if __name__ == '__main__':
    miro = Mirobot()
    block_counter = {
        "Blue": 0,
        "Red": 0,
        "Green": 0,
        "White": 0
    }
    block_height = 0.02

    try:
        while True:
            miro.set_conveyor(0.04)
            color = miro.classify_blocks()
            homing = [0.19822, 0, 0.22858]

            if color:
                print("The color detected is {}".format(color))
                # Read proximity Sensor
                obj_detection = miro.read_proximity_sensor()
                while not obj_detection:
                    obj_detection = miro.read_proximity_sensor()
                miro.set_conveyor(0)

                # Move tip to block
                x, y, z = [0.27515,  -0.00976,  0.08999]
                miro.move_robot([x, y, z])
                miro.set_gripper(1)

                # Move block up
                miro.move_robot([x, y, z+0.1])
                # Move tip to homing
                miro.move_robot(homing)

                # Move block to desired color cup
                x, y, z = list(miro.cups_pos[color])
                miro.move_robot([x, y, 0.220])
                miro.move_robot(
                    [x, y, z + block_counter[color]*block_height])
                
                # Leave block
                miro.set_gripper(0)
                miro.move_robot([x, y, 0.220])
                miro.move_robot(homing)

                block_counter[color] += 1

    except KeyboardInterrupt:
        print("Robot Stoped.")
