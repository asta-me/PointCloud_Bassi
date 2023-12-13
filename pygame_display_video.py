# -*- coding: utf-8 -*-
"""
Created on Wed Dec 13 12:42:45 2023

SLM Control as second Screen with Pygame

@author: astam
"""


import pygame
import time
from screeninfo import get_monitors
import os
from tqdm import tqdm


#%% Run Test Img
# Get the list of monitors
screens_list = get_monitors()

# Find the non-primary screen and get its dimensions and position
win_x_topleft, win_y_topleft, win_width, win_height = [(screen.x, screen.y, screen.width, screen.height) for screen in get_monitors() if not screen.is_primary][0]

# Initialize Pygame
pygame.init()

# Set the initial position for the window
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (win_x_topleft, win_y_topleft)

# Create a surface with specified dimensions
screen = pygame.display.set_mode((win_width, win_height))

# Set window caption
pygame.display.set_caption("Pygame Window")

# Define the position for the image on the screen
dest = (0.5 * (win_width - 1080), 0)

# Load the image
imgpath = r'C:\Users\astam\Desktop\Python\PointCloud_Bassi\test_Escaping_Planet__04_to_09\Escaping_planet_test_9.bmp'
img_surf = pygame.image.load(imgpath)

# Blit the image onto the screen at the specified position
screen.blit(img_surf, dest)

# Update the main window
pygame.display.flip()

# Attendi finché l'utente non preme Invio
waiting_for_input = True
while waiting_for_input:
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN and event.key == pygame.K_RETURN:   
            waiting_for_input = False
        
# Quit Pygame
pygame.quit()

#%% Run Video
pygame.init()
screen = pygame.display.set_mode((win_width, win_height))

clock = pygame.time.Clock()
framerate=20;
# Load the frames
for i in tqdm(range (1, 501)) :
    imgpath = r'C:\Users\astam\Desktop\Python\PointCloud_Bassi\test_Escaping_Planet__04_to_09\Escaping_planet_test_'+str(i)+'.bmp'
    img_surf = pygame.image.load(imgpath)
    screen.blit(img_surf, dest)
    pygame.display.flip()
    clock.tick(framerate)
pygame.quit()
