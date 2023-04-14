import pygame
import serial

ser = serial.Serial('COM10', 115200)
print(ser)

pygame.init()

WINDOW_SIZE = (1000, 800)
screen = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption('Click Window')
bg = pygame.image.load("controls.png")
screen.blit(bg, (0, 0))
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            # Close the program if the user clicks the X button
            pygame.quit()
            quit()
        elif event.type == pygame.KEYDOWN:
            ser.write(bytes(pygame.key.name(event.key) + ' down', 'utf-8'))
        elif event.type == pygame.KEYUP:
            ser.write(bytes(pygame.key.name(event.key) + ' up', 'utf-8'))
    while ser.in_waiting:
        print(ser.read().decode(), end='')
    pygame.display.update()