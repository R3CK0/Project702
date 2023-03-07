#Author: Nicholas Massad
#Date: 28/02/2023

import game_engine as ge
import a_star as astar
from rrt import RRT2D, RRTStar2D
from fmt import FMTStar2D
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
                                                (end_pos[0] + grid_size/2, end_pos[1] +  grid_size/2), progress)
                    if algo == 4:
                        fmt = FMTStar2D(environement, game_engine, radius_multiplier=1.8)
                        path = fmt.find_path((start_pos[0] + grid_size/2, start_pos[1] + grid_size/2),
                                                (end_pos[0] + grid_size/2, end_pos[1] +  grid_size/2), progress, 300)
                    if path is not None and algo == 1:
                        game_engine.draw_search_path(path, True)
                    elif path is not None:
                        game_engine.draw_search_path(path)
                    else:
                        print("No path found")
                if event.key == pygame.K_p:
                    progress = not progress
                    print("Progress: " + str(progress))
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
                    print("FMT*")
            game_engine.draw_grid()

        # Update the pygame window
        pygame.display.update()

# Run the main function
if __name__ == "__main__":
    main()

