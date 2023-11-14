import pygame
from time import sleep

def init():
    pygame.init()
    win = pygame.display.set_mode((100, 100))


def getKey(keyName):
    ans = False
    for eye in pygame.event.get():
        pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame, 'K_{}'.format(keyName))
    if keyInput[myKey]:
        #print(f'Key {keyName} was pressed')
        ans = True
    pygame.display.update()
    return ans


def main():
    if getKey('a'):
        print('a was pressed')
        pass
    if getKey('LEFT'):
        print('LEFT was pressed')
        pass


if __name__ == '__main__':
    init()
    while True:
        main()
        sleep(.02)