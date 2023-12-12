import pygame
import sys

def test_joystick(joystick):
    print(f"Number of axes: {joystick.get_numaxes()}")
    print(f"Number of balls: {joystick.get_numballs()}")
    print(f"Number of buttons: {joystick.get_numbuttons()}")
    print(f"Number of hats: {joystick.get_numhats()}")

    while True:
        pygame.event.pump()  # Pump events to update joystick state

        axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        balls = [joystick.get_ball(i) for i in range(joystick.get_numballs())]
        buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
        hats = [joystick.get_hat(i) for i in range(joystick.get_numhats())]

        print(f"axes {axes} balls {balls} buttons {buttons} hats {joystick.get_hat(0)}")
        # print(round(joystick.get_axis(2)),type(round(joystick.get_axis(2))))
        # if joystick.get_axis(2):
        #     print(1)
        # else:
        #     print(0)

if __name__ == "__main__":
    pygame.init()

    # Initialize the joystick
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick detected.")
        sys.exit()

    # Choose the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    try:
        test_joystick(joystick)
    except KeyboardInterrupt:
        pygame.quit()
        sys.exit()
