import pygame, sys
import numpy as np
import math
import time

def rotate_vector(v, angles):
    """     Rotate an n-dimensional vector v by a list of angles for each dimension.     """
    # Create rotation matrix
    if type(angles) is float:
        angles = np.asarray(angles)
        angles = np.append(angles, 0)
    else:
        angles = np.asarray(angles)
    rotation_matrix = np.identity(len(v))
    for i in range(len(v)):
        for j in range(i+1, len(v)):
            rotation_matrix[i][i] = math.cos(angles[i])
            rotation_matrix[i][j] = -math.sin(angles[i])
            rotation_matrix[j][i] = math.sin(angles[i])
            rotation_matrix[j][j] = math.cos(angles[i])
            # Rotate vector
    rotated_vector = np.matmul(rotation_matrix, v)
    return rotated_vector

pygame.init()
window = pygame.display.set_mode((400, 400))
clock = pygame.time.Clock()
def draw_rect_angle(surface, color, rect, angle, width=0):
    target_rect = pygame.Rect(rect)
    shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
    pygame.draw.rect(shape_surf, color, (0, 0, *target_rect.size), width)
    rotated_surf = pygame.transform.rotate(shape_surf, angle)
    surface.blit(rotated_surf, rotated_surf.get_rect(center = target_rect.center))
def draw_ellipse_angle(surface, color, rect, angle, width=0):
    target_rect = pygame.Rect(rect)
    shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
    pygame.draw.ellipse(shape_surf, color, (0, 0, *target_rect.size), width)
    rotated_surf = pygame.transform.rotate(shape_surf, angle)
    surface.blit(rotated_surf, rotated_surf.get_rect(center = target_rect.center))

angle = 00
run = True
while run:
    clock.tick(60)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
    window_center = window.get_rect().center
    window.fill((0, 0, 0))
    draw_rect_angle(window, (255, 255, 255), (75, 150, 250, 100), angle, 2)
    draw_ellipse_angle(window, (255, 255, 255), (75, 150, 250, 100), angle, 2)
    angle += 1
    pygame.display.update()
    time.sleep(0.1)
pygame.quit()