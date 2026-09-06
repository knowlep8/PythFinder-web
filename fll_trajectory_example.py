
import pythfinder
import pygame

def main():
    """Main function to run the FLL trajectory example."""
    sim = None
    try:
        # 1. Create a Simulator instance
        sim = pythfinder.Simulator()

        # 2. Define a starting pose for the robot (x, y, angle)
        #    The origin (0,0) is the center of the field.
        start_pose = pythfinder.Pose(x=-50, y=-50, head=45)

        # 3. Create a TrajectoryBuilder, passing preset=1 to use the FLL table.
        #    The preset is automatically applied.
        trajectory_builder = pythfinder.TrajectoryBuilder(
            sim=sim, 
            start_pose=start_pose, 
            preset=1  # 1 = FLL Table
        )

        # 4. Build a simple trajectory
        trajectory = (trajectory_builder
                      .inLineCM(75)      # Move forward 75 cm
                      .turnToDeg(90)     # Turn to a 90-degree heading
                      .inLineCM(50)      # Move forward 50 cm
                      .build())

        # 5. Follow the trajectory in the simulator
        #    The script will first animate the trajectory and then wait.
        trajectory.follow(sim, wait=True)

        print("Trajectory finished. You can now manually control the robot.")
        print("Close the window to exit.")

        # 6. Keep the simulator running for manual control
        while sim.RUNNING():
            sim.update()

    except pygame.error as e:
        if "display Surface quit" not in str(e):
            raise e
    finally:
        # 7. Clean up pygame
        if sim and not sim.RUNNING():
            print("Simulator window closed.")
        pygame.quit()

if __name__ == "__main__":
    main()
