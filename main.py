#Author: Nicholas Massad
#Date: 28/02/2023

import game_engine as ge
import a_star as astar
from rrt import RRT2D, RRTStar2D, InformedRRTStar2D
from fmt import FMTStar2D
from bit_star import BitStar
import pygame


# Main function
def main():
    # Create pygame window and draw grid
    window_width = 900
    window_height = 900
    grid_size = 30
    environement = {"width": window_width, "height": window_height, "grid_size": grid_size}
    game_engine = ge.GameEngine(window_width, window_height, grid_size)
    game_engine.draw_grid()
    game_engine.create_boundary()
    start_pos = None
    end_pos = None
    progress = False
    algo = 1
    path = None
    optimize_time = None


    # Main loop
    while True:
        # Check for events
        for event in pygame.event.get():
            # If the user closes the window, quit the program
            if event.type == pygame.QUIT:
                pygame.quit()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 3:
                    x, y = pygame.mouse.get_pos()
                    game_engine.add_obstacle(x, y)
                if event.button == 1:
                    x, y = pygame.mouse.get_pos()
                    if start_pos is not None:
                        game_engine.remove_obstacle(start_pos[0], start_pos[1])
                    start_pos = game_engine.add_start_point(x, y)
                if event.button == 2:
                    x, y = pygame.mouse.get_pos()
                    if end_pos is not None:
                        game_engine.remove_obstacle(end_pos[0], end_pos[1])
                    end_pos = game_engine.add_end_point(x, y)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c:
                    game_engine.clear()
                if event.key == pygame.K_x:
                    x, y = pygame.mouse.get_pos()
                    game_engine.remove_obstacle(x, y)
                if event.key == pygame.K_SPACE:
                    if algo == 1:
                        a_star = astar.AStar2D(environement, game_engine)
                        path = a_star.find_path(start_pos, end_pos, progress)
                    if algo == 2:
                        rrt = RRT2D(environement, game_engine, 1000)
                        path = rrt.find_path((start_pos[0] + grid_size/2, start_pos[1] + grid_size/2),
                                                (end_pos[0] + grid_size/2, end_pos[1] +  grid_size/2), progress)
                    if algo == 3:
                        rrt_star = RRTStar2D(environement, game_engine, 1000)
                        path = rrt_star.find_path((start_pos[0] + grid_size/2, start_pos[1] + grid_size/2),
                                                (end_pos[0] + grid_size/2, end_pos[1] +  grid_size/2), progress, optimize_time)
                    if algo == 4:
                        informedRRTStar = InformedRRTStar2D(environement, game_engine, 1000)
                        path = informedRRTStar.find_path((start_pos[0] + grid_size/2, start_pos[1] + grid_size/2),
                                                (end_pos[0] + grid_size/2, end_pos[1] +  grid_size/2), progress, optimize_time)
                    if algo == 5:
                        fmt = FMTStar2D(environement, game_engine, radius_multiplier=1.8)
                        path = fmt.find_path((start_pos[0] + grid_size/2, start_pos[1] + grid_size/2),
                                                (end_pos[0] + grid_size/2, end_pos[1] +  grid_size/2), progress, 150)
                    if algo == 6:
                        bit_star = BitStar(environement, game_engine, radius_multiplier=1.8, K=150)
                        path = bit_star.find_path((start_pos[0] + grid_size/2, start_pos[1] + grid_size/2),
                                                (end_pos[0] + grid_size/2, end_pos[1] +  grid_size/2), progress, optimize_time)
                    if path is not None and algo == 1:
                        game_engine.draw_search_path(path, True)
                    elif path is not None:
                        game_engine.draw_search_path(path)
                    else:
                        print("No path found")
                if event.key == pygame.K_p:
                    progress = not progress
                    print("Progress: " + str(progress))
                    print("Progress: True will slow down the algorithm do to animation") # the major reason of the slow down is due to the sleep in the animation function
                if event.key == pygame.K_z:
                    game_engine.clear(True)
                if event.key == pygame.K_1:
                    algo = 1
                    print("A*")
                if event.key == pygame.K_2:
                    algo = 2
                    print("RRT")
                if event.key == pygame.K_3:
                    algo = 3
                    print("RRT*")
                if event.key == pygame.K_4:
                    algo = 4
                    print("Informed RRT*")
                if event.key == pygame.K_5:
                    algo = 5
                    print("FMT*")
                if event.key == pygame.K_6:
                    algo = 6
                    print("BIT*")
                if event.key == pygame.K_o:
                    if optimize_time is None:
                        optimize_time = 0.2
                        print("optimization time set to 0.2 seconds")
                    elif optimize_time == 0.2:
                        optimize_time = 0.5
                        print("optimization time set to 0.5 seconds")
                    elif optimize_time == 0.5:
                        optimize_time = 1
                        print("optimization time set to 1 second")
                    elif optimize_time == 1:
                        optimize_time = 3
                        print("optimization time set to 3 seconds")
                    elif optimize_time == 3:
                        optimize_time = 10
                        print("optimization time set to 10 seconds")
                    elif optimize_time == 10:
                        optimize_time = 20
                        print("optimization time set to 20 seconds")
                    elif optimize_time == 20:
                        optimize_time = None
                        print("No optimization")
            game_engine.draw_grid()

        # Update the pygame window
        pygame.display.update()

# Run the main function
if __name__ == "__main__":
    main()

