
from robomaster import robot
from robomaster import version
import time
from robomaster import robot
from robomaster import camera
from robomaster import led


if __name__ == "__main__":

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    version_ = ep_robot.get_version()
    print(f"Robot Version {version_}")

    if version_ == None:
        ep_robot.close()
    else:
        ep_robot.play_audio(filename="gameboy_startup.wav")

    
    ep_chassis= ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_led = ep_robot.led

    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_720P)
    # def pos_stamp(inputXY):
    #     stamp = list(inputXY)
    #     return 

    # def arm_move(statusquoXY, targetXY):
    #     ep_arm.move(x=(targetXY[0]-statusquoXY[0])).wait_for_completed()
    #     ep_arm.move(y=(targetXY[1]-statusquoXY[1])).wait_for_completed()

    # def arm_adjuster_comfy(sub_info):
    #     xy_target = (70, 75)
    #     pos_x, pos_y = sub_info
    #     print(f"Robotic Arm: pos x:{pos_x}, pos y:{pos_y}")
    #     arm_move(sub_info,xy_target)
        
    # ep_arm.sub_position(freq=1, callback=arm_adjuster_comfy)
    # time.sleep(3)
    # ep_arm.unsub_position()

    def led_color(c="w"):
        if c == "w":
            ep_led.set_led(comp=led.COMP_ALL, r=255, g=255, b=255, effect=led.EFFECT_ON)
        elif c == "g":
            ep_led.set_led(comp=led.COMP_ALL, r=255, g=255, b=255, effect=led.EFFECT_ON)
        elif c == "r":
            ep_led.set_led(comp=led.COMP_ALL, r=255, g=255, b=255, effect=led.EFFECT_ON)
        else:
            pass

    def walzer_dance():
        # x_speed, y_speed = 0.2, 0.2

        def moving(movetime, x=0, y=0, z=0, timeout=5):
            ep_chassis.drive_speed(x=x, y=y, z=z, timeout=timeout)
            time.sleep(movetime)
        
        def wiggle():
            moving(0.4,z=-45)
            moving(0.4,z=45)
            moving(0.4,z=-45)
            moving(0.4,z=45)

        # def wheeldrag(speed, side="left", slp=1):
        #         if side == "right":
        #             # Turn left rear wheel
        #             ep_chassis.drive_wheels(w1=0, w2=0, w3=speed, w4=0)
        #             time.sleep(slp)
        #         else:
        #             # Turn right rear wheel
        #             ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=speed)
        #             time.sleep(slp)

        # wiggle()
        ep_robot.play_audio(filename="Strauss_Donau.wav")

        moving(3,x=0.2)
        wiggle() # 1.6
        moving(3,y=0.2)
        wiggle()
        moving(3,x=-0.2)
        wiggle()
        moving(3,y=-0.2)
        wiggle()
        moving(3,x=-0.2)
        wiggle()
        moving(3,x=0.2)
        wiggle()
        moving(1,y=0.1)

        # wheeldrag(0.5, side="right", slp=2)
        # wheeldrag(0.5, side="right", slp=2)
        speed1 = 100
        speed2 = 50

        # left front, right front, left rear, right rear

        ep_chassis.drive_wheels(w1=speed1, w2=0, w3=0, w4=0)
        time.sleep(3)
        ep_chassis.drive_wheels(w1=0, w2=speed1, w3=0, w4=0)
        time.sleep(3)

        moving(5,z=60)
        moving(5,z=-60)
        moving(1,x=-0.5)
        moving(5,z=60)
        moving(5,z=-60)
        moving(1,x=0.5)

    def baile_mexicano():

        def bandera_de_mexico_led(duracion_en_segundos):
            N_ = int(round(duracion_en_segundos / 1.5))
            colores = ["g","w","r"]
            for _ in range(0, N_): # 9 segundos
                    for j in range(len(colores)): # 1.5 segundos
                        led_color(f"{colores[j]}") # verde, blanco, rojo
                        time.sleep(0.5)
        
        ep_robot.play_audio(filename="trump_quote_mexico.wav")

        bandera_de_mexico_led(10)

        value, x_val, y_val, z_val  = 0.5, 0.5, 0.5, 30
        movez = 0.5 # seconds

        def movimiento_de_culo(t_s):
            for _ in range(2):
                ep_chassis.drive_speed(x=0, y=0, z=z_val, timeout=5)
                time.sleep(t_s)
                ep_chassis.drive_speed(x=0, y=0, z=-z_val, timeout=5)
                time.sleep(t_s)

        def x_move(movetime, value):
            ep_chassis.drive_speed(x=value, timeout=5)
            time.sleep(movetime)
        
        def y_move(movetime, value):
            ep_chassis.drive_speed(y=value, timeout=5)
            time.sleep(movetime)

        n = 0
        # direction = (-1)**n
        for c in range(2):
            ep_robot.play_audio(filename="mexican_hat_dance.wav")
            for n in range(2):
                led_color("g")
                x_move(movez,value*((-1)**n))
                led_color("w")
                y_move(movez,value*((-1)**(n+c)))
                led_color("r")
                movimiento_de_culo(movez)
                led_color("g")
                x_move(movez,value*((-1)**(n+1)))
                led_color("w")
                y_move(movez,value*((-1)**(n+1+c)))
                led_color("r")
                movimiento_de_culo(movez)
            ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5) # parando las ruedas
            time.sleep(0.5)
        bandera_de_mexico_led(10)


    def sub_info_handler(batter_info, ep_robot):
        percent = batter_info
        print("Battery: {0}%.".format(percent))
        ep_led = ep_robot.led
        brightness = int(percent * 255 / 100)
        ep_led.set_led(comp="all", r=brightness, g=brightness, b=brightness)


    while True:

        def move_sound():
            ep_robot.play_audio(filename="demo2.wav").wait_for_completed()

        print("Choose between move/sound/dance/battery/exit/")
        init_command = input("Input... ")

        if init_command == "move":
            print("Choose movement x y z & int(); e.g.: x 5")
            command = input("")
            if command[0] == "x":
                ep_chassis.move(x=int(command.split(" ")[-1])/10)
                # move_sound()
            elif command[0] == "y":
                ep_chassis.move(y=int(command.split(" ")[-1])/10)
                # move_sound()
            elif command[0] == "z":
                ep_chassis.move(z=int(command.split(" ")[-1]))
                # ep_robot.play_audio(filename="demo1.wav").wait_for_completed()
            else:
                print("input wrong")
                pass

        elif init_command == "exit":
            break

        elif init_command == "battery":
                ep_battery = ep_robot.battery
                ep_battery.sub_battery_info(1, sub_info_handler, ep_robot)
                time.sleep(1.5)
                ep_battery.unsub_battery_info()

        elif init_command == "dance":
            dance_choice = input("Choose sound (mexico/walzer)")
            if dance_choice == "mexico":
                baile_mexicano()
            elif dance_choice == "walzer":
                walzer_dance()
            else:
                pass

        elif init_command == "sound":
            s_c = input("Choose sound (bomb/allah/mexican)")
            if s_c == "bomb":
                ep_robot.play_audio(filename="bomb_countdown.wav").wait_for_completed()
            elif s_c == "allah":
                ep_robot.play_audio(filename="allahu_akbar.wav").wait_for_completed()
            elif s_c == "mexican":
                ep_robot.play_audio(filename="mexican_hat_dance.wav").wait_for_completed()
            else:
                print("input wrong")
                pass

        else:
            continue
        # elif init_command == "pick":
            
            
    ep_camera.stop_video_stream()
    ep_robot.close()
